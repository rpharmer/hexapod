#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/constraint-cholesky.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/delassus-operator.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/solvers/admm-solver.hpp>
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/multibody/joint.hpp>

#include "hexapod_dynamics_constants.hpp"

namespace minphys3d::demo {
namespace {

Eigen::Vector3d ToEigen(const Vec3& v) {
    return {v.x, v.y, v.z};
}

Vec3 FromEigen(const Eigen::Vector3d& v) {
    return {v.x(), v.y(), v.z()};
}

Eigen::Matrix3d ToEigen(const Mat3& m) {
    Eigen::Matrix3d out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = m.m[row][col];
        }
    }
    return out;
}

Eigen::Matrix3d ToEigenRotation(const Quat& q) {
    return ToEigen(RotationMatrix(q));
}

pinocchio::SE3 BodyPose(const Body& body) {
    return {ToEigenRotation(body.orientation), ToEigen(body.position)};
}

pinocchio::Inertia BodyInertia(const Body& body) {
    Mat3 inertia_local{};
    if (!InvertMat3(body.invInertiaLocal, inertia_local)) {
        throw std::runtime_error("dynamic body has singular local inertia");
    }
    return {body.mass, Eigen::Vector3d::Zero(), ToEigen(inertia_local)};
}

Quat FromEigenRotation(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond q(rotation);
    return Normalize(Quat{q.w(), q.x(), q.y(), q.z()});
}

bool IsFinite(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

std::uint64_t PersistentContactId(const Manifold& manifold, const Contact& contact) {
    std::uint64_t value = manifold.pairKey();
    value ^= contact.key + 0x9e3779b97f4a7c15ULL + (value << 6U) + (value >> 2U);
    return value;
}

Eigen::Matrix3d ContactFrameRotation(const Vec3& inputNormal) {
    const Vec3 normal = Normalize(inputNormal);
    Vec3 tangent0{};
    Vec3 tangent1{};
    if (!World::ComputeStableTangentFrame(normal, Vec3{}, tangent0, tangent1)) {
        tangent0 = std::abs(normal.y) < 0.9 ? Normalize(Cross({0.0, 1.0, 0.0}, normal))
                                           : Normalize(Cross({1.0, 0.0, 0.0}, normal));
        tangent1 = Normalize(Cross(normal, tangent0));
    }
    Eigen::Matrix3d rotation;
    rotation.col(0) = ToEigen(tangent0);
    rotation.col(1) = ToEigen(tangent1);
    rotation.col(2) = ToEigen(normal);
    return rotation;
}

} // namespace

struct PinocchioHexapodModel::Impl {
    struct BodyBinding {
        std::uint32_t bodyId = World::kInvalidBodyId;
        pinocchio::JointIndex jointId = 0;
        pinocchio::FrameIndex frameId = 0;
    };

    pinocchio::Model model{};
    pinocchio::Data data{model};
    std::vector<BodyBinding> bodies{};
    std::array<pinocchio::JointIndex, 18> wireJoints{};
    std::array<std::uint32_t, 18> wireServoIds{};
    std::unordered_map<std::uint32_t, pinocchio::JointIndex> bodyJoints{};

    struct WarmImpulse {
        Eigen::Vector3d value = Eigen::Vector3d::Zero();
        double dt = 0.0;
    };
    std::unordered_map<std::uint64_t, WarmImpulse> contactWarmStarts{};
    std::vector<double> lastGoodQ{};
    std::vector<double> lastGoodV{};
    std::uint64_t totalWarmStartResets = 0;
    std::uint64_t totalRetries = 0;
    std::uint64_t totalRollbacks = 0;
    std::uint64_t totalHeldStates = 0;
    std::uint64_t totalUnsupportedIslands = 0;

    Impl(
        const World& world,
        const HexapodSceneObjects& scene,
        const std::array<std::uint32_t, 18>& servoJointIds) {
        wireServoIds = servoJointIds;

        const Body& chassis = world.GetBody(scene.body);
        const pinocchio::JointIndex root = model.addJoint(
            0,
            pinocchio::JointModelFreeFlyer(),
            pinocchio::SE3::Identity(),
            "chassis_free_flyer");
        model.appendBodyToJoint(root, BodyInertia(chassis), pinocchio::SE3::Identity());
        const pinocchio::FrameIndex chassisFrame = model.addFrame(pinocchio::Frame(
            "chassis_body", root, pinocchio::SE3::Identity(), pinocchio::BODY));
        bodies.push_back({scene.body, root, chassisFrame});
        bodyJoints.emplace(scene.body, root);

        std::unordered_map<std::uint32_t, pinocchio::JointIndex> servoToJoint;
        std::unordered_map<std::uint32_t, pinocchio::SE3> jointRestWorld;
        jointRestWorld.emplace(root, BodyPose(chassis));

        auto appendJoint = [&](std::uint32_t servoId,
                               std::uint32_t childBodyId,
                               pinocchio::JointIndex parentJoint,
                               const std::string& name) {
            const ServoJoint& servo = world.GetServoJoint(servoId);
            const Body& parentBody = world.GetBody(servo.a);
            const Body& childBody = world.GetBody(childBodyId);
            const Vec3 anchor = parentBody.position
                + Rotate(parentBody.orientation, servo.localAnchorA);
            const pinocchio::SE3 jointWorld(Eigen::Matrix3d::Identity(), ToEigen(anchor));
            const pinocchio::SE3 placement = jointRestWorld.at(parentJoint).inverse() * jointWorld;
            const Vec3 axisWorld = Normalize(Rotate(parentBody.orientation, servo.localAxisA));
            const pinocchio::JointIndex jointId = model.addJoint(
                parentJoint,
                pinocchio::JointModelRevoluteUnaligned(ToEigen(axisWorld)),
                placement,
                name);
            const pinocchio::SE3 jointToBody = jointWorld.inverse() * BodyPose(childBody);
            model.appendBodyToJoint(jointId, BodyInertia(childBody), jointToBody);
            const pinocchio::FrameIndex frameId = model.addFrame(pinocchio::Frame(
                name + "_body", jointId, jointToBody, pinocchio::BODY));
            bodies.push_back({childBodyId, jointId, frameId});
            bodyJoints.emplace(childBodyId, jointId);
            servoToJoint.emplace(servoId, jointId);
            jointRestWorld.emplace(jointId, jointWorld);
            return jointId;
        };

        for (std::size_t legIndex = 0; legIndex < scene.legs.size(); ++legIndex) {
            const LegLinkIds& leg = scene.legs[legIndex];
            const std::string prefix = "leg_" + std::to_string(legIndex);
            const pinocchio::JointIndex coxa = appendJoint(
                leg.bodyToCoxaJoint, leg.coxa, root, prefix + "_coxa");
            const pinocchio::JointIndex femur = appendJoint(
                leg.coxaToFemurJoint, leg.femur, coxa, prefix + "_femur");
            (void)appendJoint(
                leg.femurToTibiaJoint, leg.tibia, femur, prefix + "_tibia");
        }

        for (std::size_t i = 0; i < wireServoIds.size(); ++i) {
            wireJoints[i] = servoToJoint.at(wireServoIds[i]);
        }

        model.gravity.linear() = Eigen::Vector3d(0.0, -9.80665, 0.0);
        data = pinocchio::Data(model);
    }
};

PinocchioHexapodModel::PinocchioHexapodModel(
    const World& world,
    const HexapodSceneObjects& scene,
    const std::array<std::uint32_t, 18>& servo_joint_ids)
    : impl_(std::make_unique<Impl>(world, scene, servo_joint_ids)) {}

PinocchioHexapodModel::~PinocchioHexapodModel() = default;
PinocchioHexapodModel::PinocchioHexapodModel(PinocchioHexapodModel&&) noexcept = default;
PinocchioHexapodModel& PinocchioHexapodModel::operator=(PinocchioHexapodModel&&) noexcept = default;

std::size_t PinocchioHexapodModel::configurationSize() const {
    return static_cast<std::size_t>(impl_->model.nq);
}

std::size_t PinocchioHexapodModel::velocitySize() const {
    return static_cast<std::size_t>(impl_->model.nv);
}

std::size_t PinocchioHexapodModel::jointCount() const {
    return impl_->wireJoints.size();
}

bool PinocchioHexapodModel::readState(
    const World& world,
    std::vector<double>& qOut,
    std::vector<double>& vOut) const {
    Eigen::VectorXd q = pinocchio::neutral(impl_->model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv);
    const Body& chassis = world.GetBody(impl_->bodies.front().bodyId);
    q.segment<3>(0) = ToEigen(chassis.position);
    q[3] = chassis.orientation.x;
    q[4] = chassis.orientation.y;
    q[5] = chassis.orientation.z;
    q[6] = chassis.orientation.w;
    const Eigen::Matrix3d worldToBody = ToEigenRotation(chassis.orientation).transpose();
    v.segment<3>(0) = worldToBody * ToEigen(chassis.velocity);
    v.segment<3>(3) = worldToBody * ToEigen(chassis.angularVelocity);

    for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
        const pinocchio::JointIndex pinJoint = impl_->wireJoints[i];
        const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
        const Body& parent = world.GetBody(servo.a);
        const Body& child = world.GetBody(servo.b);
        q[impl_->model.joints[pinJoint].idx_q()] = world.GetServoJointAngle(impl_->wireServoIds[i]);
        const Vec3 axisWorld = Normalize(Rotate(parent.orientation, servo.localAxisA));
        v[impl_->model.joints[pinJoint].idx_v()] =
            Dot(child.angularVelocity - parent.angularVelocity, axisWorld);
    }

    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    qOut.assign(q.data(), q.data() + q.size());
    vOut.assign(v.data(), v.data() + v.size());
    return true;
}

bool PinocchioHexapodModel::writeState(
    World& world,
    const std::vector<double>& qIn,
    const std::vector<double>& vIn) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), static_cast<Eigen::Index>(qIn.size()));
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), static_cast<Eigen::Index>(vIn.size()));
    if (!IsFinite(q) || !IsFinite(v)) {
        return false;
    }
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);
    for (const Impl::BodyBinding& binding : impl_->bodies) {
        Body& body = world.GetBody(binding.bodyId);
        const pinocchio::SE3& pose = impl_->data.oMf[binding.frameId];
        const pinocchio::Motion motion = pinocchio::getFrameVelocity(
            impl_->model,
            impl_->data,
            binding.frameId,
            pinocchio::LOCAL_WORLD_ALIGNED);
        body.position = FromEigen(pose.translation());
        body.orientation = FromEigenRotation(pose.rotation());
        body.velocity = FromEigen(motion.linear());
        body.angularVelocity = FromEigen(motion.angular());
        body.isSleeping = false;
    }
    return true;
}

bool PinocchioHexapodModel::computeFreeAcceleration(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    const std::vector<double>& tauIn,
    std::vector<double>& ddqOut) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()
        || tauIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    const Eigen::Map<const Eigen::VectorXd> tau(tauIn.data(), impl_->model.nv);
    const Eigen::VectorXd ddq = pinocchio::aba(impl_->model, impl_->data, q, v, tau);
    if (!IsFinite(ddq)) {
        return false;
    }
    ddqOut.assign(ddq.data(), ddq.data() + ddq.size());
    return true;
}

bool PinocchioHexapodModel::computeDenseAcceleration(
    const std::vector<double>& qIn,
    const std::vector<double>& vIn,
    const std::vector<double>& tauIn,
    std::vector<double>& ddqOut) {
    if (qIn.size() != configurationSize() || vIn.size() != velocitySize()
        || tauIn.size() != velocitySize()) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qIn.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vIn.data(), impl_->model.nv);
    const Eigen::Map<const Eigen::VectorXd> tau(tauIn.data(), impl_->model.nv);
    Eigen::MatrixXd mass = pinocchio::crba(impl_->model, impl_->data, q);
    mass.triangularView<Eigen::StrictlyLower>() = mass.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::VectorXd bias = pinocchio::nonLinearEffects(impl_->model, impl_->data, q, v);
    const Eigen::VectorXd ddq = mass.ldlt().solve(tau - bias);
    if (!IsFinite(ddq)) {
        return false;
    }
    ddqOut.assign(ddq.data(), ddq.data() + ddq.size());
    return true;
}

bool PinocchioHexapodModel::validateDelassusOracle(
    const World& world,
    double tolerance,
    double& maxRelativeError) {
    maxRelativeError = std::numeric_limits<double>::infinity();
    if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
        return false;
    }

    std::vector<double> qStorage;
    std::vector<double> vStorage;
    if (!readState(world, qStorage, vStorage)) {
        return false;
    }
    const Eigen::Map<const Eigen::VectorXd> q(qStorage.data(), impl_->model.nq);
    const Eigen::Map<const Eigen::VectorXd> v(vStorage.data(), impl_->model.nv);
    pinocchio::crba(impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q, v);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    // Two distinct universe-to-chassis point contacts avoid the rank-deficient
    // single-contact rotational nullspace while exercising all three contact
    // coordinates and the articulated floating-base mass matrix.
    const pinocchio::JointIndex chassis = impl_->bodies.front().jointId;
    const Body& chassisBody = world.GetBody(impl_->bodies.front().bodyId);
    const Eigen::Matrix3d rotation = ContactFrameRotation({0.0, 1.0, 0.0});
    std::vector<pinocchio::ConstraintModel> models;
    models.reserve(2);
    for (const Eigen::Vector3d& offset : {
             Eigen::Vector3d(0.0, -0.08, 0.11),
             Eigen::Vector3d(0.12, -0.13, -0.07)}) {
        const pinocchio::SE3 worldContact(rotation, ToEigen(chassisBody.position) + offset);
        const pinocchio::SE3 chassisPlacement =
            impl_->data.oMi[chassis].inverse() * worldContact;
        models.emplace_back(pinocchio::PointContactConstraintModel(
            impl_->model, 0, worldContact, chassis, chassisPlacement));
    }

    std::vector<pinocchio::ConstraintData> datas;
    datas.reserve(models.size());
    for (const pinocchio::ConstraintModel& model : models) {
        datas.push_back(model.createData());
    }
    for (std::size_t i = 0; i < models.size(); ++i) {
        models[i].calc(impl_->model, impl_->data, datas[i]);
    }

    using RigidDelassus = pinocchio::DelassusOperatorRigidBodySystemsTpl<
        double,
        0,
        pinocchio::JointCollectionDefaultTpl,
        pinocchio::ConstraintModel,
        std::reference_wrapper>;
    constexpr double regularization = 1.0e-10;
    RigidDelassus rigid(
        std::cref(impl_->model),
        std::ref(impl_->data),
        std::cref(models),
        std::cref(datas),
        regularization);
    rigid.compute();

    pinocchio::ConstraintCholeskyDecomposition cholesky(
        impl_->model,
        impl_->data,
        models,
        datas,
        regularization);
    cholesky.compute(impl_->model, impl_->data, models, datas, regularization);
    const Eigen::MatrixXd production = rigid.matrix(false, true);
    const Eigen::MatrixXd oracle =
        cholesky.getDelassusOperatorCholeskyExpression().matrix(false, true);
    if (production.rows() != oracle.rows() || production.cols() != oracle.cols()
        || !production.array().isFinite().all() || !oracle.array().isFinite().all()) {
        return false;
    }
    const double scale = std::max(1.0, oracle.cwiseAbs().maxCoeff());
    maxRelativeError = (production - oracle).cwiseAbs().maxCoeff() / scale;
    return std::isfinite(maxRelativeError) && maxRelativeError <= tolerance;
}

void PinocchioHexapodModel::resetWarmStarts() {
    impl_->totalWarmStartResets += impl_->contactWarmStarts.size();
    impl_->contactWarmStarts.clear();
}

bool PinocchioHexapodModel::stepProximal(
    World& world,
    double dt,
    const ProximalSolverSettings& settings,
    ProximalStepDiagnostics& diagnostics) {
    diagnostics = {};
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        return false;
    }

    std::vector<double> snapshotQ;
    std::vector<double> snapshotV;
    if (!readState(world, snapshotQ, snapshotV)) {
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        return false;
    }
    if (impl_->lastGoodQ.empty()) {
        impl_->lastGoodQ = snapshotQ;
        impl_->lastGoodV = snapshotV;
    }

    auto totalMechanicalEnergy = [&]() {
        const Vec3 gravity = world.GetGravity();
        double energy = 0.0;
        for (const Impl::BodyBinding& binding : impl_->bodies) {
            const Body& body = world.GetBody(binding.bodyId);
            Mat3 inertiaWorld{};
            const Mat3 inverseWorld = body.InvInertiaWorld();
            if (!InvertMat3(inverseWorld, inertiaWorld)) {
                continue;
            }
            energy += 0.5 * body.mass * Dot(body.velocity, body.velocity);
            energy += 0.5 * Dot(body.angularVelocity, inertiaWorld * body.angularVelocity);
            energy -= body.mass * Dot(gravity, body.position);
        }
        return energy;
    };

    bool unsupportedIsland = false;
    auto advanceOnce = [&](double subDt, ProximalStepDiagnostics& out) -> bool {
        std::vector<double> qStorage;
        std::vector<double> vStorage;
        if (!readState(world, qStorage, vStorage)) {
            return false;
        }
        Eigen::Map<const Eigen::VectorXd> q(qStorage.data(), impl_->model.nq);
        Eigen::Map<const Eigen::VectorXd> v(vStorage.data(), impl_->model.nv);
        if (!IsFinite(q) || !IsFinite(v)) {
            return false;
        }

        impl_->model.gravity.linear() = ToEigen(world.GetGravity());
        Eigen::MatrixXd mass = pinocchio::crba(
            impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
        mass.triangularView<Eigen::StrictlyLower>() =
            mass.transpose().triangularView<Eigen::StrictlyLower>();
        if (!mass.array().isFinite().all()) {
            return false;
        }

        Eigen::VectorXd tau = Eigen::VectorXd::Zero(impl_->model.nv);
        constexpr double stallTorque = hexapod_dynamics::kServoMaxTorqueNm;
        constexpr double noLoadSpeed = hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
        constexpr double omegaN = hexapod_dynamics::kServoOmegaN;
        constexpr double zeta = hexapod_dynamics::kServoZeta;
        for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
            const pinocchio::JointIndex joint = impl_->wireJoints[i];
            const Eigen::Index qi = impl_->model.joints[joint].idx_q();
            const Eigen::Index vi = impl_->model.joints[joint].idx_v();
            const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
            const double error = std::remainder(
                servo.targetAngle - q[qi], 6.28318530717958647692);
            const double reflectedInertia = std::max(1.0e-9, mass(vi, vi));
            const double requested = reflectedInertia * omegaN * omegaN * error
                - 2.0 * zeta * omegaN * reflectedInertia * v[vi];
            double available = stallTorque;
            if (requested * v[vi] > 0.0) {
                available *= std::max(0.0, 1.0 - std::abs(v[vi]) / noLoadSpeed);
            }
            tau[vi] = std::clamp(requested, -available, available);
            out.peakActuatorImpulse = std::max(out.peakActuatorImpulse, std::abs(tau[vi]) * subDt);
            out.peakServoTorqueUtilization = std::max(
                out.peakServoTorqueUtilization, std::abs(tau[vi]) / stallTorque);
            out.actuatorWork += tau[vi] * v[vi] * subDt;
        }

        const double energyBefore = totalMechanicalEnergy();
        const Eigen::VectorXd acceleration = pinocchio::aba(
            impl_->model, impl_->data, q, v, tau, pinocchio::Convention::WORLD);
        Eigen::VectorXd vNew = v + subDt * acceleration;
        if (!IsFinite(acceleration) || !IsFinite(vNew)) {
            return false;
        }

        world.PrepareExternalContacts(subDt);
        std::vector<pinocchio::ConstraintModel> constraintModels;
        std::vector<pinocchio::ConstraintData> constraintDatas;
        std::vector<std::uint64_t> contactIds;
        std::vector<double> contactFrictions;
        std::vector<double> contactRestitutions;
        std::vector<double> contactPenetrations;
        std::unordered_set<std::uint64_t> seenContactIds;
        constraintModels.reserve(world.DebugManifolds().size() * 4U);
        contactIds.reserve(world.DebugManifolds().size() * 4U);

        for (const Manifold& manifold : world.DebugManifolds()) {
            const auto aIt = impl_->bodyJoints.find(manifold.a);
            const auto bIt = impl_->bodyJoints.find(manifold.b);
            const bool aRobot = aIt != impl_->bodyJoints.end();
            const bool bRobot = bIt != impl_->bodyJoints.end();
            if (!aRobot && !bRobot) {
                continue;
            }
            ++out.contactManifoldCount;
            if (aRobot != bRobot) {
                const std::uint32_t externalId = aRobot ? manifold.b : manifold.a;
                const Body& external = world.GetBody(externalId);
                if (!external.isStatic && external.invMass > 0.0) {
                    unsupportedIsland = true;
                    return false;
                }
            }

            const Body& bodyA = world.GetBody(manifold.a);
            const Body& bodyB = world.GetBody(manifold.b);
            const double friction = std::max(
                0.0, 0.5 * (bodyA.dynamicFriction + bodyB.dynamicFriction));
            for (const Contact& contact : manifold.contacts) {
                const std::uint64_t contactId = PersistentContactId(manifold, contact);
                if (!seenContactIds.insert(contactId).second) {
                    ++out.duplicateContactCount;
                }
                pinocchio::JointIndex joint1 = 0;
                pinocchio::JointIndex joint2 = 0;
                Vec3 normal = manifold.normal;
                if (aRobot && bRobot) {
                    joint1 = aIt->second;
                    joint2 = bIt->second;
                } else {
                    joint2 = aRobot ? aIt->second : bIt->second;
                    normal = aRobot ? -1.0 * manifold.normal : manifold.normal;
                }

                const pinocchio::SE3 contactWorld(
                    ContactFrameRotation(normal), ToEigen(contact.point));
                const pinocchio::SE3 placement1 = joint1 == 0
                    ? contactWorld
                    : impl_->data.oMi[joint1].inverse() * contactWorld;
                const pinocchio::SE3 placement2 = joint2 == 0
                    ? contactWorld
                    : impl_->data.oMi[joint2].inverse() * contactWorld;
                pinocchio::PointContactConstraintModel pointModel(
                    impl_->model, joint1, placement1, joint2, placement2);
                pointModel.setFriction(friction);
                constraintModels.emplace_back(pointModel);
                contactIds.push_back(contactId);
                contactFrictions.push_back(friction);
                contactRestitutions.push_back(std::max(
                    0.0, 0.5 * (bodyA.restitution + bodyB.restitution)));
                contactPenetrations.push_back(contact.penetration);
            }
        }

        if (!constraintModels.empty()) {
            out.contactConstraintCount = constraintModels.size();
            for (const std::uint64_t contactId : contactIds) {
                out.contactSetSignature ^= contactId + 0x9e3779b97f4a7c15ULL
                    + (out.contactSetSignature << 6U)
                    + (out.contactSetSignature >> 2U);
            }
            constraintDatas.reserve(constraintModels.size());
            for (const pinocchio::ConstraintModel& model : constraintModels) {
                constraintDatas.push_back(model.createData());
            }
            for (std::size_t i = 0; i < constraintModels.size(); ++i) {
                constraintModels[i].calc(impl_->model, impl_->data, constraintDatas[i]);
            }

            using RigidDelassus = pinocchio::DelassusOperatorRigidBodySystemsTpl<
                double,
                0,
                pinocchio::JointCollectionDefaultTpl,
                pinocchio::ConstraintModel,
                std::reference_wrapper>;
            RigidDelassus delassus(
                std::cref(impl_->model),
                std::ref(impl_->data),
                std::cref(constraintModels),
                std::cref(constraintDatas),
                settings.contactRegularization);
            delassus.compute();

            const Eigen::MatrixXd jacobian = pinocchio::getConstraintsJacobian(
                impl_->model, impl_->data, constraintModels, constraintDatas);
            Eigen::VectorXd drift = jacobian * vNew;
            const ContactSolverConfig& contactSettings = world.GetContactSolverConfig();
            for (std::size_t i = 0; i < constraintModels.size(); ++i) {
                const Eigen::Index normalIndex = static_cast<Eigen::Index>(3U * i + 2U);
                const double correction = std::min(
                    std::max(0.0, contactSettings.penetrationBiasMaxSpeed),
                    std::max(0.0, contactSettings.penetrationBiasFactor)
                        * std::max(0.0, contactPenetrations[i] - contactSettings.penetrationSlop)
                        / subDt);
                drift[normalIndex] -= correction;
                const double incomingNormalSpeed = drift[normalIndex];
                const double restitutionCutoff = std::max(
                    contactSettings.restitutionVelocityCutoff,
                    settings.restitutionVelocityCutoff);
                if (incomingNormalSpeed < -restitutionCutoff) {
                    drift[normalIndex] += contactRestitutions[i] * incomingNormalSpeed;
                }
            }

            Eigen::VectorXd warm = Eigen::VectorXd::Zero(drift.size());
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const auto found = impl_->contactWarmStarts.find(contactIds[i]);
                if (found == impl_->contactWarmStarts.end()) {
                    continue;
                }
                Eigen::Vector3d impulse = found->second.value;
                if (found->second.dt > 0.0) {
                    impulse *= subDt / found->second.dt;
                }
                impulse[2] = std::max(0.0, impulse[2]);
                const double tangential = impulse.head<2>().norm();
                const double limit = contactFrictions[i] * impulse[2];
                if (tangential > limit && tangential > 0.0) {
                    impulse.head<2>() *= limit / tangential;
                }
                warm.segment<3>(static_cast<Eigen::Index>(3U * i)) = impulse;
            }

            pinocchio::ADMMConstraintSolver solver(drift.size());
            pinocchio::ADMMSolverSettings solverSettings;
            solverSettings.max_iterations = static_cast<std::size_t>(std::max(1, settings.maxIterations));
            solverSettings.absolute_feasibility_tol = settings.absoluteTolerance;
            solverSettings.relative_feasibility_tol = settings.relativeTolerance;
            solverSettings.absolute_complementarity_tol = settings.absoluteTolerance;
            solverSettings.relative_complementarity_tol = settings.relativeTolerance;
            solverSettings.admm_update_rule = pinocchio::ADMMUpdateRule::SPECTRAL;
            solverSettings.admm_proximal_rule = pinocchio::ADMMProximalRule::MANUAL;
            solverSettings.mu_prox = settings.proximalMu;
            solverSettings.solve_ncp = true;
            solverSettings.stat_record = false;
            pinocchio::ADMMSolverResult result;
            result.resize(static_cast<std::size_t>(drift.size()));
            result.setConstraintImpulseGuess(warm);
            const bool converged = solver.solve(
                delassus,
                drift,
                constraintModels,
                constraintDatas,
                solverSettings,
                result);
            out.iterations = std::max(out.iterations, static_cast<int>(result.iterations));
            out.primalResidual = std::max(out.primalResidual, result.primal_feasibility);
            out.dualResidual = std::max(out.dualResidual, result.dual_feasibility);
            out.complementarityResidual = std::max(
                out.complementarityResidual, result.complementarity);
            if (!converged || !std::isfinite(result.primal_feasibility)
                || !std::isfinite(result.dual_feasibility)
                || !std::isfinite(result.complementarity)) {
                if (jacobian.rows() <= 72) {
                    const Eigen::MatrixXd delassusDense =
                        jacobian * mass.ldlt().solve(jacobian.transpose());
                    if (delassusDense.array().isFinite().all()) {
                        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
                            0.5 * (delassusDense + delassusDense.transpose()));
                        if (eigensolver.info() == Eigen::Success) {
                            const Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
                            const double minEigenvalue = eigenvalues.minCoeff();
                            const double maxEigenvalue = eigenvalues.maxCoeff();
                            if (minEigenvalue > 0.0 && std::isfinite(minEigenvalue)
                                && std::isfinite(maxEigenvalue)) {
                                out.delassusConditionEstimate =
                                    maxEigenvalue / minEigenvalue;
                            }
                        }
                    }
                }
                return false;
            }

            Eigen::VectorXd impulses(drift.size());
            result.retrieveConstraintImpulses(impulses);
            if (!IsFinite(impulses)) {
                return false;
            }
            std::unordered_set<std::uint64_t> activeIds;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const Eigen::Vector3d impulse = impulses.segment<3>(
                    static_cast<Eigen::Index>(3U * i));
                impl_->contactWarmStarts[contactIds[i]] = {impulse, subDt};
                activeIds.insert(contactIds[i]);
                out.peakNormalImpulse = std::max(out.peakNormalImpulse, std::abs(impulse[2]));
                out.peakFrictionImpulse = std::max(
                    out.peakFrictionImpulse, impulse.head<2>().norm());
                if (std::abs(impulse[2]) >= out.peakNormalImpulse) {
                    out.worstContactId = contactIds[i];
                }
            }
            for (auto it = impl_->contactWarmStarts.begin(); it != impl_->contactWarmStarts.end();) {
                if (activeIds.find(it->first) == activeIds.end()) {
                    it = impl_->contactWarmStarts.erase(it);
                    ++impl_->totalWarmStartResets;
                } else {
                    ++it;
                }
            }
            vNew += mass.ldlt().solve(jacobian.transpose() * impulses);
        } else if (!impl_->contactWarmStarts.empty()) {
            impl_->totalWarmStartResets += impl_->contactWarmStarts.size();
            impl_->contactWarmStarts.clear();
        }

        if (!IsFinite(vNew)) {
            return false;
        }
        pinocchio::forwardKinematics(impl_->model, impl_->data, q, vNew);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);
        for (const Impl::BodyBinding& binding : impl_->bodies) {
            const pinocchio::Motion motion = pinocchio::getFrameVelocity(
                impl_->model, impl_->data, binding.frameId, pinocchio::LOCAL_WORLD_ALIGNED);
            const double linearSpeed = motion.linear().norm();
            const double angularSpeed = motion.angular().norm();
            out.preIntegrationLinearSpeed = std::max(out.preIntegrationLinearSpeed, linearSpeed);
            out.preIntegrationAngularSpeed = std::max(out.preIntegrationAngularSpeed, angularSpeed);
            if (!std::isfinite(linearSpeed) || !std::isfinite(angularSpeed)
                || linearSpeed > settings.maxLinearSpeed
                || angularSpeed > settings.maxAngularSpeed) {
                return false;
            }
        }

        const Eigen::VectorXd qNew = pinocchio::integrate(impl_->model, q, subDt * vNew);
        if (!IsFinite(qNew)) {
            return false;
        }
        std::vector<double> qNewStorage(qNew.data(), qNew.data() + qNew.size());
        std::vector<double> vNewStorage(vNew.data(), vNew.data() + vNew.size());
        if (!writeState(world, qNewStorage, vNewStorage)) {
            return false;
        }
        const double energyAfter = totalMechanicalEnergy();
        const double energyDelta = energyAfter - energyBefore;
        if (!std::isfinite(energyDelta)) {
            return false;
        }
        out.mechanicalEnergyDelta += energyDelta;
        return true;
    };

    ProximalStepDiagnostics firstAttempt{};
    if (advanceOnce(dt, firstAttempt)) {
        world.CompleteExternalDynamicsStep();
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::Healthy;
        readState(world, impl_->lastGoodQ, impl_->lastGoodV);
    } else if (unsupportedIsland) {
        ++impl_->totalUnsupportedIslands;
        ++impl_->totalRollbacks;
        writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::UnsupportedIsland;
    } else {
        ++impl_->totalRetries;
        ++impl_->totalRollbacks;
        writeState(world, snapshotQ, snapshotV);
        resetWarmStarts();
        ProximalStepDiagnostics retry{};
        const bool half1 = advanceOnce(0.5 * dt, retry);
        const bool half2 = half1 && advanceOnce(0.5 * dt, retry);
        if (half1 && half2) {
            world.CompleteExternalDynamicsStep();
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::RecoveredRetry;
            readState(world, impl_->lastGoodQ, impl_->lastGoodV);
        } else {
            ++impl_->totalHeldStates;
            ++impl_->totalRollbacks;
            writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
            resetWarmStarts();
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::HeldLastGood;
        }
    }

    diagnostics.warmStartResets = impl_->totalWarmStartResets;
    diagnostics.retries = impl_->totalRetries;
    diagnostics.rollbackCount = impl_->totalRollbacks;
    diagnostics.heldStateCount = impl_->totalHeldStates;
    diagnostics.unsupportedIslandCount = impl_->totalUnsupportedIslands;
    return diagnostics.status == ProximalStepStatus::Healthy
        || diagnostics.status == ProximalStepStatus::RecoveredRetry;
}

} // namespace minphys3d::demo
