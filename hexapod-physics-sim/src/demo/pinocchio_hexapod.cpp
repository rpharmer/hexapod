#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
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

double BoundedEnvDouble(
    const char* name, double fallback, double minimum, double maximum) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end != value && *end == '\0' && std::isfinite(parsed)
            && parsed >= minimum && parsed <= maximum
        ? parsed
        : fallback;
}

Quat FromEigenRotation(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond q(rotation);
    return Normalize(Quat{q.w(), q.x(), q.y(), q.z()});
}

bool IsFinite(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

Eigen::Vector3d ProjectCoulombImpulse(const Eigen::Vector3d& impulse, double friction) {
    const double mu = std::max(0.0, friction);
    const double tangentNorm = impulse.head<2>().norm();
    const double normal = impulse[2];
    if (normal >= 0.0 && tangentNorm <= mu * normal) {
        return impulse;
    }
    if (mu <= 0.0 || tangentNorm <= -normal / mu) {
        return Eigen::Vector3d::Zero();
    }

    const double projectedNormal = (mu * tangentNorm + normal) / (mu * mu + 1.0);
    Eigen::Vector3d projected{};
    projected[2] = std::max(0.0, projectedNormal);
    if (tangentNorm > 0.0) {
        projected.head<2>() = impulse.head<2>()
            * (mu * projected[2] / tangentNorm);
    }
    return projected;
}

std::uint64_t PersistentContactId(const Manifold& manifold, const Contact& contact) {
    if (manifold.contacts.size() == 1U) {
        return manifold.pairKey();
    }
    std::uint64_t value = manifold.pairKey();
    value ^= contact.key + 0x9e3779b97f4a7c15ULL + (value << 6U) + (value >> 2U);
    return value;
}

std::uint64_t ContactOrderKey(std::uint64_t value, const std::uint64_t seed) {
    if (seed == 0) {
        return value;
    }
    value += seed + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
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

struct GroundContactChoice {
    const Manifold* plane = nullptr;
    const Manifold* terrain = nullptr;
    bool useTerrain = false;
};

Vec3 ExternalContactNormal(const Manifold& manifold, bool aRobot) {
    return Normalize(aRobot ? -1.0 * manifold.normal : manifold.normal);
}

bool TerrainMatchesPlane(const World& world,
                         const Manifold& terrain,
                         const Manifold& plane,
                         std::uint32_t robotBody) {
    const bool terrainARobot = terrain.a == robotBody;
    const bool planeARobot = plane.a == robotBody;
    const Vec3 terrainNormal = ExternalContactNormal(terrain, terrainARobot);
    const Vec3 planeNormal = ExternalContactNormal(plane, planeARobot);
    if (Dot(terrainNormal, planeNormal) < 0.9995) {
        return false;
    }

    const std::uint32_t planeBodyId = planeARobot ? plane.b : plane.a;
    const Body& planeBody = world.GetBody(planeBodyId);
    constexpr double kEquivalentSurfaceToleranceM = 0.003;
    for (const Contact& contact : terrain.contacts) {
        const double terrainSurfaceOffset = Dot(planeNormal, contact.point)
            + contact.penetration;
        if (std::abs(terrainSurfaceOffset - planeBody.planeOffset)
            > kEquivalentSurfaceToleranceM) {
            return false;
        }
    }
    return !terrain.contacts.empty();
}

} // namespace

using PinConstraintModels = std::vector<pinocchio::ConstraintModel>;
using PinConstraintDatas = std::vector<pinocchio::ConstraintData>;
using RigidDelassus = pinocchio::DelassusOperatorRigidBodySystemsTpl<
    double,
    0,
    pinocchio::JointCollectionDefaultTpl,
    pinocchio::ConstraintModel,
    std::reference_wrapper>;

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
    std::array<double, 18> wireZeroAngles{};
    std::array<double, 18> wireNominalInertias{};
    double totalRobotMass = 0.0;
    std::unordered_map<std::uint32_t, pinocchio::JointIndex> bodyJoints{};
    PinConstraintModels contactConstraintModels{};
    PinConstraintDatas contactConstraintDatas{};
    std::vector<std::uint64_t> contactTopologyIds{};
    std::unique_ptr<RigidDelassus> contactDelassus{};
    pinocchio::DelassusOperatorDense contactDenseDelassus{};
    double contactDelassusRegularization = 0.0;
    pinocchio::ADMMConstraintSolver contactSolver{72};
    pinocchio::ADMMSolverResult contactSolverResult{};
    std::size_t andersonCapacity = 3;
    std::optional<std::size_t> retryAndersonCapacityOverride{};
    double ratioPrimalDual = 5.0;
    double admmTau = 0.7;
    double spectralRhoPowerInit = 0.2;
    double servoGainScale = 1.0;
    bool warmstartRho = true;
    bool denseAdmm = false;
    std::uint64_t contactOrderSeed = 0;

    struct WarmContact {
        Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double dt = 0.0;
    };
    std::unordered_map<std::uint64_t, WarmContact> contactWarmStarts{};
    std::uint64_t lastContactSetSignature = 0;
    bool haveLastContactSetSignature = false;
    std::vector<double> lastGoodQ{};
    std::vector<double> lastGoodV{};
    std::array<double, 18> lastServoTargets{};
    bool haveLastServoTargets = false;
    std::array<double, 18> commandedServoTargets{};
    bool haveCommandedServoTargets = false;
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
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_ANDERSON_CAPACITY")) {
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (end != value && *end == '\0' && parsed >= 0 && parsed <= 16) {
                andersonCapacity = static_cast<std::size_t>(parsed);
            }
        }
        if (const char* value = std::getenv(
                "HEXAPOD_PINOCCHIO_RETRY_ANDERSON_CAPACITY")) {
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (end != value && *end == '\0' && parsed >= 0 && parsed <= 16) {
                retryAndersonCapacityOverride = static_cast<std::size_t>(parsed);
            }
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_CONTACT_ORDER_SEED")) {
            char* end = nullptr;
            const unsigned long long parsed = std::strtoull(value, &end, 10);
            if (end != value && *end == '\0') {
                contactOrderSeed = static_cast<std::uint64_t>(parsed);
            }
        }
        ratioPrimalDual = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_RATIO_PRIMAL_DUAL", ratioPrimalDual, 0.01, 1000.0);
        admmTau = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_ADMM_TAU", admmTau, 0.01, 1.0);
        spectralRhoPowerInit = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_SPECTRAL_POWER", spectralRhoPowerInit, 0.0, 1.0);
        servoGainScale = BoundedEnvDouble(
            "HEXAPOD_PINOCCHIO_SERVO_GAIN_SCALE", servoGainScale, 0.01, 100.0);
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_WARMSTART_RHO")) {
            warmstartRho = value[0] != '\0' && value[0] != '0';
        }
        if (const char* value = std::getenv("HEXAPOD_PINOCCHIO_DENSE_ADMM")) {
            denseAdmm = value[0] != '\0' && value[0] != '0';
        }

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

        for (const BodyBinding& binding : bodies) {
            totalRobotMass += world.GetBody(binding.bodyId).mass;
        }

        for (std::size_t i = 0; i < wireServoIds.size(); ++i) {
            wireJoints[i] = servoToJoint.at(wireServoIds[i]);
            wireZeroAngles[i] = world.GetServoJointAngle(wireServoIds[i]);
        }

        model.gravity.linear() = Eigen::Vector3d(0.0, -9.80665, 0.0);
        data = pinocchio::Data(model);
    }
};

PinocchioHexapodModel::PinocchioHexapodModel(
    const World& world,
    const HexapodSceneObjects& scene,
    const std::array<std::uint32_t, 18>& servo_joint_ids)
    : impl_(std::make_unique<Impl>(world, scene, servo_joint_ids)) {
    if (!readState(world, impl_->lastGoodQ, impl_->lastGoodV)) {
        throw std::runtime_error("failed to capture initial Pinocchio hexapod state");
    }
    const Eigen::Map<const Eigen::VectorXd> initialQ(
        impl_->lastGoodQ.data(), static_cast<Eigen::Index>(impl_->lastGoodQ.size()));
    Eigen::MatrixXd nominalMass = pinocchio::crba(
        impl_->model, impl_->data, initialQ, pinocchio::Convention::WORLD);
    nominalMass.triangularView<Eigen::StrictlyLower>() =
        nominalMass.transpose().triangularView<Eigen::StrictlyLower>();
    if (!nominalMass.array().isFinite().all()) {
        throw std::runtime_error("failed to compute nominal hexapod joint inertias");
    }
    for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
        const Eigen::Index vi = impl_->model.joints[impl_->wireJoints[i]].idx_v();
        impl_->wireNominalInertias[i] = std::max(1.0e-9, nominalMass(vi, vi));
    }
}

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
        q[impl_->model.joints[pinJoint].idx_q()] = std::remainder(
            world.GetServoJointAngle(impl_->wireServoIds[i]) - impl_->wireZeroAngles[i],
            6.28318530717958647692);
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
    maxRelativeError = 0.0;
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

    const pinocchio::JointIndex chassis = impl_->bodies.front().jointId;
    const Body& chassisBody = world.GetBody(impl_->bodies.front().bodyId);
    const Eigen::Matrix3d rotation = ContactFrameRotation({0.0, 1.0, 0.0});
    constexpr double regularization = 1.0e-10;
    const auto makeContact = [&](pinocchio::JointIndex joint,
                                 const Eigen::Vector3d& worldPoint) {
        const pinocchio::SE3 worldContact(rotation, worldPoint);
        const pinocchio::SE3 jointPlacement =
            impl_->data.oMi[joint].inverse() * worldContact;
        return pinocchio::ConstraintModel(pinocchio::PointContactConstraintModel(
            impl_->model, 0, worldContact, joint, jointPlacement));
    };
    const auto compare = [&](PinConstraintModels models) {
        PinConstraintDatas datas;
        datas.reserve(models.size());
        for (const pinocchio::ConstraintModel& model : models) {
            datas.push_back(model.createData());
        }
        for (std::size_t i = 0; i < models.size(); ++i) {
            models[i].calc(impl_->model, impl_->data, datas[i]);
        }

        RigidDelassus rigid(
            std::cref(impl_->model),
            std::ref(impl_->data),
            std::cref(models),
            std::cref(datas),
            regularization);
        rigid.compute();
        pinocchio::ConstraintCholeskyDecomposition cholesky(
            impl_->model, impl_->data, models, datas, regularization);
        cholesky.compute(impl_->model, impl_->data, models, datas, regularization);
        const Eigen::MatrixXd production = rigid.matrix(false, true);
        const Eigen::MatrixXd oracle =
            cholesky.getDelassusOperatorCholeskyExpression().matrix(false, true);
        if (production.rows() != oracle.rows() || production.cols() != oracle.cols()
            || !production.array().isFinite().all() || !oracle.array().isFinite().all()) {
            return false;
        }
        const double scale = std::max(1.0, oracle.cwiseAbs().maxCoeff());
        const double relativeError =
            (production - oracle).cwiseAbs().maxCoeff() / scale;
        maxRelativeError = std::max(maxRelativeError, relativeError);
        return std::isfinite(relativeError) && relativeError <= tolerance;
    };

    const Eigen::Vector3d chassisPoint =
        ToEigen(chassisBody.position) + Eigen::Vector3d(0.0, -0.08, 0.11);
    const pinocchio::ConstraintModel chassisContact = makeContact(chassis, chassisPoint);
    if (!compare({chassisContact})
        || !compare({
            chassisContact,
            makeContact(
                chassis,
                ToEigen(chassisBody.position) + Eigen::Vector3d(0.12, -0.13, -0.07))})
        || !compare({chassisContact, chassisContact})) {
        return false;
    }

    PinConstraintModels allFeet;
    allFeet.reserve(6);
    for (std::size_t leg = 0; leg < 6; ++leg) {
        const pinocchio::JointIndex tibiaJoint = impl_->wireJoints[3U * leg + 2U];
        allFeet.push_back(makeContact(tibiaJoint, impl_->data.oMi[tibiaJoint].translation()));
    }
    return compare(std::move(allFeet));
}

void PinocchioHexapodModel::resetWarmStarts() {
    impl_->totalWarmStartResets += impl_->contactWarmStarts.size();
    impl_->contactWarmStarts.clear();
    impl_->lastContactSetSignature = 0;
    impl_->haveLastContactSetSignature = false;
    impl_->contactSolverResult.reset();
    impl_->contactSolver.reset();
}

bool PinocchioHexapodModel::synchronizeAfterExternalCorrection(const World& world) {
    std::vector<double> correctedQ;
    std::vector<double> correctedV;
    if (!readState(world, correctedQ, correctedV)) {
        return false;
    }

    resetWarmStarts();
    impl_->lastGoodQ = std::move(correctedQ);
    impl_->lastGoodV = std::move(correctedV);
    impl_->haveLastServoTargets = false;
    impl_->haveCommandedServoTargets = false;
    return true;
}

bool PinocchioHexapodModel::stepProximal(
    World& world,
    double dt,
    const ProximalSolverSettings& settings,
    ProximalStepDiagnostics& diagnostics) {
    using StepClock = std::chrono::steady_clock;
    const auto stepStart = StepClock::now();
    const auto elapsedMs = [](const StepClock::time_point start) {
        return std::chrono::duration<double, std::milli>(StepClock::now() - start).count();
    };
    diagnostics = {};
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        diagnostics.failureReason = ProximalFailureReason::InvalidDt;
        diagnostics.totalStepTimeMs = elapsedMs(stepStart);
        return false;
    }

    std::array<double, 18> servoTargets{};
    bool targetJump = !impl_->haveLastServoTargets;
    for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
        servoTargets[i] = world.GetServoJoint(impl_->wireServoIds[i]).targetAngle;
        if (impl_->haveLastServoTargets) {
            const double delta = std::abs(std::remainder(
                servoTargets[i] - impl_->lastServoTargets[i],
                6.28318530717958647692));
            targetJump = targetJump || delta > 0.25;
        }
    }
    const bool resetCommandedTargets = targetJump && impl_->haveLastServoTargets;
    impl_->lastServoTargets = servoTargets;
    impl_->haveLastServoTargets = true;
    if (resetCommandedTargets) {
        resetWarmStarts();
    }

    std::vector<double> snapshotQ;
    std::vector<double> snapshotV;
    if (!readState(world, snapshotQ, snapshotV)) {
        ++impl_->totalHeldStates;
        ++impl_->totalRollbacks;
        (void)writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        diagnostics.status = ProximalStepStatus::HeldLastGood;
        diagnostics.failureReason = ProximalFailureReason::ReadState;
        diagnostics.warmStartResets = impl_->totalWarmStartResets;
        diagnostics.retries = impl_->totalRetries;
        diagnostics.rollbackCount = impl_->totalRollbacks;
        diagnostics.heldStateCount = impl_->totalHeldStates;
        diagnostics.unsupportedIslandCount = impl_->totalUnsupportedIslands;
        diagnostics.totalStepTimeMs = elapsedMs(stepStart);
        return false;
    }
    if (impl_->lastGoodQ.empty()) {
        impl_->lastGoodQ = snapshotQ;
        impl_->lastGoodV = snapshotV;
    }
    std::array<double, 18> commandedServoTargetsStart = impl_->commandedServoTargets;
    if (!impl_->haveCommandedServoTargets || resetCommandedTargets) {
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            commandedServoTargetsStart[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
    }
    const auto advanceCommandedTargets = [&](std::array<double, 18> targets,
                                             const double targetDt) {
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
            const double delta = std::remainder(
                servoTargets[i] - targets[i], 6.28318530717958647692);
            const double maxStep = std::max(0.0, servo.maxServoSpeed) * targetDt;
            targets[i] += maxStep > 0.0
                ? std::clamp(delta, -maxStep, maxStep)
                : delta;
        }
        return targets;
    };
    const std::array<double, 18> commandedServoTargets =
        advanceCommandedTargets(commandedServoTargetsStart, dt);

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
    auto advanceOnce = [&](double subDt,
                           const std::array<double, 18>& activeServoTargets,
                           const std::size_t andersonCapacity,
                           ProximalStepDiagnostics& out) -> bool {
        const auto dynamicsStart = StepClock::now();
        std::vector<double> qStorage;
        std::vector<double> vStorage;
        if (!readState(world, qStorage, vStorage)) {
            out.failureReason = ProximalFailureReason::ReadState;
            return false;
        }
        Eigen::Map<const Eigen::VectorXd> q(qStorage.data(), impl_->model.nq);
        Eigen::Map<const Eigen::VectorXd> v(vStorage.data(), impl_->model.nv);
        if (!IsFinite(q) || !IsFinite(v)) {
            out.failureReason = ProximalFailureReason::NonFiniteState;
            return false;
        }

        impl_->model.gravity.linear() = ToEigen(world.GetGravity());
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(impl_->model.nv);
        constexpr double stallTorque = hexapod_dynamics::kServoMaxTorqueNm;
        constexpr double noLoadSpeed = hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
        constexpr double omegaN = hexapod_dynamics::kServoOmegaN;
        constexpr double zeta = hexapod_dynamics::kServoZeta;
        for (std::size_t i = 0; i < impl_->wireJoints.size(); ++i) {
            const pinocchio::JointIndex joint = impl_->wireJoints[i];
            const Eigen::Index qi = impl_->model.joints[joint].idx_q();
            const Eigen::Index vi = impl_->model.joints[joint].idx_v();
            const double error = std::remainder(
                activeServoTargets[i] - impl_->wireZeroAngles[i] - q[qi],
                6.28318530717958647692);
            // The natural-frequency gains are calibrated against the nominal
            // standing inertia. Recomputing CRBA every substep changed the
            // controller gains with pose and duplicated ABA's dynamics work.
            const double reflectedInertia = impl_->wireNominalInertias[i];
            const double requested = impl_->servoGainScale
                * (reflectedInertia * omegaN * omegaN * error
                    - 2.0 * zeta * omegaN * reflectedInertia * v[vi]);
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
            out.failureReason = ProximalFailureReason::NonFiniteAcceleration;
            return false;
        }
        out.dynamicsTimeMs += elapsedMs(dynamicsStart);

        const auto contactSetupStart = StepClock::now();
        world.PrepareExternalContacts(subDt);
        out.collisionTimeMs += elapsedMs(contactSetupStart);
        auto constraintAssemblyStart = StepClock::now();
        std::vector<std::uint64_t> contactIds;
        std::vector<double> contactFrictions;
        std::vector<double> contactRestitutions;
        std::vector<double> contactPenetrations;
        std::unordered_set<std::uint64_t> seenContactIds;
        std::unordered_map<std::uint32_t, GroundContactChoice> groundChoices;
        for (const Manifold& candidate : world.DebugManifolds()) {
            const bool aRobot = impl_->bodyJoints.find(candidate.a) != impl_->bodyJoints.end();
            const bool bRobot = impl_->bodyJoints.find(candidate.b) != impl_->bodyJoints.end();
            if (aRobot == bRobot) {
                continue;
            }
            const std::uint32_t robotBody = aRobot ? candidate.a : candidate.b;
            const std::uint32_t externalBody = aRobot ? candidate.b : candidate.a;
            if (world.IsTerrainAttachmentBody(externalBody)) {
                groundChoices[robotBody].terrain = &candidate;
            } else if (world.GetBody(externalBody).shape == ShapeType::Plane) {
                groundChoices[robotBody].plane = &candidate;
            }
        }
        for (auto& [robotBody, choice] : groundChoices) {
            choice.useTerrain = choice.terrain != nullptr
                && (choice.plane == nullptr
                    || !TerrainMatchesPlane(world, *choice.terrain, *choice.plane, robotBody));
        }

        struct AcceptedContactGeometry {
            pinocchio::JointIndex joint1 = 0;
            pinocchio::JointIndex joint2 = 0;
            Vec3 point{};
            Vec3 normal{};
        };
        struct PendingContact {
            AcceptedContactGeometry geometry{};
            std::uint64_t id = 0;
            double friction = 0.0;
            double restitution = 0.0;
            double penetration = 0.0;
        };
        std::vector<AcceptedContactGeometry> acceptedContactGeometry;
        std::vector<PendingContact> pendingContacts;
        contactIds.reserve(world.DebugManifolds().size() * 4U);
        pendingContacts.reserve(world.DebugManifolds().size() * 4U);

        for (const Manifold& manifold : world.DebugManifolds()) {
            const auto aIt = impl_->bodyJoints.find(manifold.a);
            const auto bIt = impl_->bodyJoints.find(manifold.b);
            const bool aRobot = aIt != impl_->bodyJoints.end();
            const bool bRobot = bIt != impl_->bodyJoints.end();
            if (!aRobot && !bRobot) {
                continue;
            }
            if (aRobot != bRobot) {
                const std::uint32_t robotBody = aRobot ? manifold.a : manifold.b;
                const std::uint32_t externalBody = aRobot ? manifold.b : manifold.a;
                const auto choiceIt = groundChoices.find(robotBody);
                if (choiceIt != groundChoices.end()) {
                    const bool isTerrain = world.IsTerrainAttachmentBody(externalBody);
                    const bool isPlane = !isTerrain
                        && world.GetBody(externalBody).shape == ShapeType::Plane;
                    if ((isTerrain && !choiceIt->second.useTerrain)
                        || (isPlane && choiceIt->second.useTerrain)) {
                        continue;
                    }
                }
            }
            if (aRobot && bRobot) {
                const ServoJoint* connectingJoint = nullptr;
                for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
                    const ServoJoint& servo = world.GetServoJoint(impl_->wireServoIds[i]);
                    if ((servo.a == manifold.a && servo.b == manifold.b)
                        || (servo.a == manifold.b && servo.b == manifold.a)) {
                        connectingJoint = &servo;
                        break;
                    }
                }
                if (connectingJoint != nullptr) {
                    continue;
                }
            }
            ++out.contactManifoldCount;
            if (aRobot && bRobot) {
                ++out.robotRobotManifoldCount;
            } else {
                ++out.externalManifoldCount;
            }
            if (aRobot != bRobot) {
                const std::uint32_t externalId = aRobot ? manifold.b : manifold.a;
                const Body& external = world.GetBody(externalId);
                if (!external.isStatic && external.invMass > 0.0) {
                    unsupportedIsland = true;
                    out.failureReason = ProximalFailureReason::UnsupportedIsland;
                    return false;
                }
            }

            const Body& bodyA = world.GetBody(manifold.a);
            const Body& bodyB = world.GetBody(manifold.b);
            const double friction = std::max(
                0.0, 0.5 * (bodyA.dynamicFriction + bodyB.dynamicFriction));
            for (const Contact& contact : manifold.contacts) {
                ++out.contactPointCount;
                if (!std::isfinite(contact.penetration)
                    || contact.penetration > settings.maxContactPenetration) {
                    out.maxContactPenetration = contact.penetration;
                    out.failureReason = ProximalFailureReason::ExtremePenetration;
                    return false;
                }
                out.maxContactPenetration =
                    std::max(out.maxContactPenetration, contact.penetration);
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
                const bool coincident = std::any_of(
                    acceptedContactGeometry.begin(),
                    acceptedContactGeometry.end(),
                    [&](const AcceptedContactGeometry& accepted) {
                        return accepted.joint1 == joint1
                            && accepted.joint2 == joint2
                            && LengthSquared(accepted.point - contact.point) <= 1.0e-10
                            && Dot(accepted.normal, normal) >= 0.999999;
                    });
                if (coincident) {
                    ++out.duplicateContactCount;
                    continue;
                }
                acceptedContactGeometry.push_back({joint1, joint2, contact.point, normal});

                pendingContacts.push_back({
                    {joint1, joint2, contact.point, normal},
                    contactId,
                    friction,
                    std::max(0.0, 0.5 * (bodyA.restitution + bodyB.restitution)),
                    contact.penetration});
            }
        }

        std::sort(
            pendingContacts.begin(),
            pendingContacts.end(),
            [contactOrderSeed = impl_->contactOrderSeed](
                const PendingContact& lhs, const PendingContact& rhs) {
                const std::uint64_t lhsOrder = ContactOrderKey(lhs.id, contactOrderSeed);
                const std::uint64_t rhsOrder = ContactOrderKey(rhs.id, contactOrderSeed);
                if (lhsOrder != rhsOrder) return lhsOrder < rhsOrder;
                if (lhs.id != rhs.id) return lhs.id < rhs.id;
                if (lhs.geometry.joint1 != rhs.geometry.joint1) {
                    return lhs.geometry.joint1 < rhs.geometry.joint1;
                }
                if (lhs.geometry.joint2 != rhs.geometry.joint2) {
                    return lhs.geometry.joint2 < rhs.geometry.joint2;
                }
                if (lhs.geometry.point.x != rhs.geometry.point.x) {
                    return lhs.geometry.point.x < rhs.geometry.point.x;
                }
                if (lhs.geometry.point.y != rhs.geometry.point.y) {
                    return lhs.geometry.point.y < rhs.geometry.point.y;
                }
                return lhs.geometry.point.z < rhs.geometry.point.z;
            });
        for (const PendingContact& pending : pendingContacts) {
            contactIds.push_back(pending.id);
            contactFrictions.push_back(pending.friction);
            contactRestitutions.push_back(pending.restitution);
            contactPenetrations.push_back(pending.penetration);
        }

        if (!pendingContacts.empty()) {
            out.contactConstraintCount = pendingContacts.size();
            for (const std::uint64_t contactId : contactIds) {
                out.contactSetSignature ^= contactId + 0x9e3779b97f4a7c15ULL
                    + (out.contactSetSignature << 6U)
                    + (out.contactSetSignature >> 2U);
            }
            impl_->lastContactSetSignature = out.contactSetSignature;
            impl_->haveLastContactSetSignature = true;

            const std::unordered_set<std::uint64_t> currentContactIds(
                contactIds.begin(), contactIds.end());
            for (auto it = impl_->contactWarmStarts.begin();
                 it != impl_->contactWarmStarts.end();) {
                if (currentContactIds.find(it->first) == currentContactIds.end()) {
                    it = impl_->contactWarmStarts.erase(it);
                    ++impl_->totalWarmStartResets;
                } else {
                    ++it;
                }
            }

            const auto makePointModel = [&](const PendingContact& pending) {
                const pinocchio::SE3 contactWorld(
                    ContactFrameRotation(pending.geometry.normal),
                    ToEigen(pending.geometry.point));
                const pinocchio::SE3 placement1 = pending.geometry.joint1 == 0
                    ? contactWorld
                    : impl_->data.oMi[pending.geometry.joint1].inverse() * contactWorld;
                const pinocchio::SE3 placement2 = pending.geometry.joint2 == 0
                    ? contactWorld
                    : impl_->data.oMi[pending.geometry.joint2].inverse() * contactWorld;
                pinocchio::PointContactConstraintModel pointModel(
                    impl_->model,
                    pending.geometry.joint1,
                    placement1,
                    pending.geometry.joint2,
                    placement2);
                pointModel.setFriction(pending.friction);
                return pinocchio::ConstraintModel(pointModel);
            };

            const bool topologyChanged = contactIds != impl_->contactTopologyIds
                || impl_->contactDelassus == nullptr
                || impl_->contactDelassusRegularization != settings.contactRegularization;
            if (topologyChanged) {
                impl_->contactSolverResult.reset();
                impl_->contactSolver.reset();
                impl_->contactConstraintModels.clear();
                impl_->contactConstraintDatas.clear();
                impl_->contactConstraintModels.reserve(pendingContacts.size());
                impl_->contactConstraintDatas.reserve(pendingContacts.size());
                for (const PendingContact& pending : pendingContacts) {
                    impl_->contactConstraintModels.push_back(makePointModel(pending));
                    impl_->contactConstraintDatas.push_back(
                        impl_->contactConstraintModels.back().createData());
                }
                impl_->contactTopologyIds = contactIds;
                impl_->contactDelassusRegularization = settings.contactRegularization;
                impl_->contactDelassus = std::make_unique<RigidDelassus>(
                    std::cref(impl_->model),
                    std::ref(impl_->data),
                    std::cref(impl_->contactConstraintModels),
                    std::cref(impl_->contactConstraintDatas),
                    settings.contactRegularization);
            } else {
                for (std::size_t i = 0; i < pendingContacts.size(); ++i) {
                    impl_->contactConstraintModels[i] = makePointModel(pendingContacts[i]);
                }
            }

            PinConstraintModels& constraintModels = impl_->contactConstraintModels;
            PinConstraintDatas& constraintDatas = impl_->contactConstraintDatas;
            RigidDelassus& delassus = *impl_->contactDelassus;
            for (std::size_t i = 0; i < constraintModels.size(); ++i) {
                constraintModels[i].calc(impl_->model, impl_->data, constraintDatas[i]);
            }
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            const auto delassusStart = StepClock::now();
            delassus.compute();
            if (impl_->denseAdmm) {
                // The rigid-body operator remains the source of the
                // articulated Delassus matrix and final generalized impulse.
                // Materialising its small contact-space matrix makes repeated
                // ADMM products dense matrix-vector operations.
                impl_->contactDenseDelassus.rebuild(delassus, true);
            }
            out.delassusTimeMs += elapsedMs(delassusStart);
            constraintAssemblyStart = StepClock::now();

            Eigen::VectorXd drift(static_cast<Eigen::Index>(3U * constraintModels.size()));
            pinocchio::evalConstraintJacobianMatrixProduct(
                impl_->model,
                impl_->data,
                constraintModels,
                constraintDatas,
                vNew,
                drift,
                pinocchio::SetTo());
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
            Eigen::VectorXd warmVelocity = Eigen::VectorXd::Zero(drift.size());
            const Vec3 gravity = world.GetGravity();
            const std::size_t supportingContactCount = static_cast<std::size_t>(
                std::count_if(
                    pendingContacts.begin(),
                    pendingContacts.end(),
                    [&](const PendingContact& pending) {
                        return Dot(gravity, pending.geometry.normal) < -1.0e-6;
                    }));
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const auto found = impl_->contactWarmStarts.find(contactIds[i]);
                if (found == impl_->contactWarmStarts.end()) {
                    if (supportingContactCount != 0) {
                        const double gravityNormalAcceleration = std::max(
                            0.0, -Dot(gravity, pendingContacts[i].geometry.normal));
                        warm[static_cast<Eigen::Index>(3U * i + 2U)] =
                            impl_->totalRobotMass * gravityNormalAcceleration * subDt
                            / static_cast<double>(supportingContactCount);
                    }
                    continue;
                }
                Eigen::Vector3d impulse = found->second.impulse;
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
                warmVelocity.segment<3>(static_cast<Eigen::Index>(3U * i)) =
                    found->second.velocity;
            }
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            out.contactSetupTimeMs += elapsedMs(contactSetupStart);

            pinocchio::ADMMSolverSettings solverSettings;
            // Pinocchio numbers iterations from zero and loops while
            // `iterations <= max_iterations`; subtract one so the public
            // configuration remains a true maximum solve count.
            solverSettings.max_iterations = static_cast<std::size_t>(
                std::max(1, settings.maxIterations) - 1);
            solverSettings.absolute_feasibility_tol = settings.absoluteTolerance;
            solverSettings.relative_feasibility_tol = settings.relativeTolerance;
            solverSettings.absolute_complementarity_tol = settings.absoluteTolerance;
            solverSettings.relative_complementarity_tol = settings.relativeTolerance;
            solverSettings.admm_update_rule = pinocchio::ADMMUpdateRule::SPECTRAL;
            solverSettings.admm_proximal_rule = pinocchio::ADMMProximalRule::MANUAL;
            solverSettings.mu_prox = settings.proximalMu;
            solverSettings.anderson_capacity = andersonCapacity;
            solverSettings.ratio_primal_dual = impl_->ratioPrimalDual;
            solverSettings.tau = impl_->admmTau;
            solverSettings.spectral_rho_power_init = impl_->spectralRhoPowerInit;
            solverSettings.warmstart_rho_with_previous_result = impl_->warmstartRho;
            solverSettings.solve_ncp = true;
            solverSettings.stat_record = false;
            pinocchio::ADMMSolverResult& result = impl_->contactSolverResult;
            if (result.constraintSize() != drift.size()) {
                result.resize(static_cast<std::size_t>(drift.size()));
                result.reset();
            }
            result.setConstraintImpulseGuess(warm);
            if (IsFinite(warmVelocity)) {
                result.setConstraintVelocityGuess(warmVelocity);
            }
            const auto admmStart = StepClock::now();
            const bool converged = impl_->denseAdmm
                ? impl_->contactSolver.solve(
                    impl_->contactDenseDelassus,
                    drift,
                    constraintModels,
                    constraintDatas,
                    solverSettings,
                    result)
                : impl_->contactSolver.solve(
                    delassus,
                    drift,
                    constraintModels,
                    constraintDatas,
                    solverSettings,
                    result);
            out.admmTimeMs += elapsedMs(admmStart);
            out.admmRho = result.rho;
            out.iterations = std::max(out.iterations, static_cast<int>(result.iterations));
            out.primalResidual = std::max(out.primalResidual, result.primal_feasibility);
            out.dualResidual = std::max(out.dualResidual, result.dual_feasibility);
            out.complementarityResidual = std::max(
                out.complementarityResidual, result.complementarity);
            Eigen::VectorXd impulses(drift.size());
            result.retrieveConstraintImpulses(impulses);
            if (IsFinite(impulses)) {
                Eigen::VectorXd contactVelocities(drift.size());
                delassus.applyOnTheRight(impulses, contactVelocities, false);
                contactVelocities += drift;
                Eigen::VectorXd deSaxce = Eigen::VectorXd::Zero(drift.size());
                Eigen::VectorXd correctedVelocities = Eigen::VectorXd::Zero(drift.size());
                Eigen::VectorXd projectedDual = Eigen::VectorXd::Zero(drift.size());
                pinocchio::internal::computeDeSaxeCorrection(
                    constraintModels, constraintDatas, contactVelocities, deSaxce);
                correctedVelocities = contactVelocities + deSaxce;
                pinocchio::internal::computeDualConstraintSetProjection(
                    constraintModels, constraintDatas, correctedVelocities, projectedDual);
                out.ncpDualResidual =
                    (projectedDual - correctedVelocities).lpNorm<Eigen::Infinity>();
                pinocchio::internal::computeConicComplementarity(
                    constraintModels,
                    constraintDatas,
                    correctedVelocities,
                    impulses,
                    out.ncpComplementarityResidual);
                double worstContactResidual = -1.0;
                for (std::size_t i = 0; i < contactIds.size(); ++i) {
                    const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                    const Eigen::Vector3d impulse = impulses.segment<3>(offset);
                    const double coneResidual =
                        (impulse - ProjectCoulombImpulse(impulse, contactFrictions[i])).norm();
                    const double dualResidual =
                        (projectedDual - correctedVelocities).segment<3>(offset)
                            .lpNorm<Eigen::Infinity>();
                    out.coneResidual = std::max(out.coneResidual, coneResidual);
                    const double contactResidual = std::max(coneResidual, dualResidual);
                    if (contactResidual > worstContactResidual) {
                        worstContactResidual = contactResidual;
                        out.worstContactId = contactIds[i];
                    }
                }
            }
            Eigen::VectorXd solvedContactVelocities(drift.size());
            result.retrieveConstraintVelocities(solvedContactVelocities);
            const double driftScale = drift.lpNorm<Eigen::Infinity>();
            const double impulseScale = IsFinite(impulses)
                ? impulses.lpNorm<Eigen::Infinity>()
                : std::numeric_limits<double>::infinity();
            const double ncpDualTolerance = settings.absoluteTolerance
                + settings.relativeTolerance * driftScale;
            const double ncpComplementarityTolerance = settings.absoluteTolerance
                + settings.relativeTolerance * impulseScale * driftScale;
            const double coneTolerance = settings.absoluteTolerance
                + settings.relativeTolerance * impulseScale;
            const bool physicallyConverged = IsFinite(impulses)
                && std::isfinite(out.ncpDualResidual)
                && std::isfinite(out.ncpComplementarityResidual)
                && std::isfinite(out.coneResidual)
                && out.ncpDualResidual <= ncpDualTolerance
                && out.ncpComplementarityResidual <= ncpComplementarityTolerance
                && out.coneResidual <= coneTolerance;
            if ((!converged && !physicallyConverged)
                || !std::isfinite(result.primal_feasibility)
                || !std::isfinite(result.dual_feasibility)
                || !std::isfinite(result.complementarity)) {
                // A bounded, non-converged iterate is still useful as a retry
                // initial guess. Preserve only contacts other than the one
                // with the worst residual; no impulse from this solve is
                // applied to the robot state.
                if (IsFinite(impulses) && IsFinite(solvedContactVelocities)) {
                    for (std::size_t i = 0; i < contactIds.size(); ++i) {
                        if (contactIds[i] == out.worstContactId) {
                            continue;
                        }
                        const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                        impl_->contactWarmStarts[contactIds[i]] = {
                            impulses.segment<3>(offset),
                            solvedContactVelocities.segment<3>(offset),
                            subDt};
                    }
                }
                out.failureReason = ProximalFailureReason::SolverNotConverged;
                return false;
            }
            if (!IsFinite(impulses)) {
                out.failureReason = ProximalFailureReason::NonFiniteImpulse;
                return false;
            }
            if (!IsFinite(solvedContactVelocities)) {
                out.failureReason = ProximalFailureReason::NonFiniteVelocity;
                return false;
            }
            std::unordered_set<std::uint64_t> activeIds;
            for (std::size_t i = 0; i < contactIds.size(); ++i) {
                const Eigen::Index offset = static_cast<Eigen::Index>(3U * i);
                const Eigen::Vector3d impulse = impulses.segment<3>(
                    offset);
                const Eigen::Vector3d velocity = solvedContactVelocities.segment<3>(offset);
                impl_->contactWarmStarts[contactIds[i]] = {impulse, velocity, subDt};
                activeIds.insert(contactIds[i]);
                out.peakNormalImpulse = std::max(out.peakNormalImpulse, std::abs(impulse[2]));
                out.peakFrictionImpulse = std::max(
                    out.peakFrictionImpulse, impulse.head<2>().norm());
            }
            for (auto it = impl_->contactWarmStarts.begin(); it != impl_->contactWarmStarts.end();) {
                if (activeIds.find(it->first) == activeIds.end()) {
                    it = impl_->contactWarmStarts.erase(it);
                    ++impl_->totalWarmStartResets;
                } else {
                    ++it;
                }
            }
            // applyOnTheRight above computes both the constraint-space
            // velocity and the associated generalized M^-1 J^T impulse in
            // the articulated operator's workspace. Reuse that generalized
            // result instead of rebuilding and factorizing a dense mass matrix.
            vNew += delassus.getInternalData().ddq;
        } else {
            impl_->contactConstraintModels.clear();
            impl_->contactConstraintDatas.clear();
            impl_->contactTopologyIds.clear();
            impl_->contactDelassus.reset();
            impl_->contactDelassusRegularization = 0.0;
            out.constraintAssemblyTimeMs += elapsedMs(constraintAssemblyStart);
            out.contactSetupTimeMs += elapsedMs(contactSetupStart);
            if (!impl_->contactWarmStarts.empty() || impl_->haveLastContactSetSignature) {
                resetWarmStarts();
            }
        }

        const auto integrationStart = StepClock::now();
        if (!IsFinite(vNew)) {
            out.failureReason = ProximalFailureReason::NonFiniteVelocity;
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
                out.failureReason = ProximalFailureReason::SpeedLimit;
                return false;
            }
        }

        const Eigen::VectorXd qNew = pinocchio::integrate(impl_->model, q, subDt * vNew);
        if (!IsFinite(qNew)) {
            out.failureReason = ProximalFailureReason::NonFiniteConfiguration;
            return false;
        }
        std::vector<double> qNewStorage(qNew.data(), qNew.data() + qNew.size());
        std::vector<double> vNewStorage(vNew.data(), vNew.data() + vNew.size());
        if (!writeState(world, qNewStorage, vNewStorage)) {
            out.failureReason = ProximalFailureReason::WriteState;
            return false;
        }
        const double energyAfter = totalMechanicalEnergy();
        const double energyDelta = energyAfter - energyBefore;
        if (!std::isfinite(energyDelta)) {
            out.failureReason = ProximalFailureReason::NonFiniteEnergy;
            return false;
        }
        out.mechanicalEnergyDelta += energyDelta;
        out.integrationTimeMs += elapsedMs(integrationStart);
        return true;
    };

    ProximalStepDiagnostics firstAttempt{};
    const auto clearFailedSolverState = [&](const std::uint64_t worstContactId) {
        if (worstContactId == 0) {
            resetWarmStarts();
            return;
        }
        impl_->totalWarmStartResets +=
            impl_->contactWarmStarts.erase(worstContactId);
        impl_->contactSolverResult.reset();
        impl_->contactSolver.reset();
    };
    if (advanceOnce(dt, commandedServoTargets, impl_->andersonCapacity, firstAttempt)) {
        world.CompleteExternalDynamicsStep();
        impl_->commandedServoTargets = commandedServoTargets;
        impl_->haveCommandedServoTargets = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::Healthy;
        readState(world, impl_->lastGoodQ, impl_->lastGoodV);
    } else if (unsupportedIsland) {
        ++impl_->totalUnsupportedIslands;
        ++impl_->totalRollbacks;
        writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::UnsupportedIsland;
    } else if (firstAttempt.failureReason != ProximalFailureReason::SolverNotConverged
               && firstAttempt.failureReason != ProximalFailureReason::SpeedLimit) {
        // Retrying with two half-steps can only help failures that depend on
        // integration size or iterative convergence. Invalid/non-finite state,
        // excessive penetration, and state write failures are unchanged by a
        // smaller dt, so publish a held last-good sample without performing two
        // redundant collision and contact solves.
        ++impl_->totalRollbacks;
        ++impl_->totalHeldStates;
        writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
        resetWarmStarts();
        for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
            impl_->commandedServoTargets[i] =
                world.GetServoJointAngle(impl_->wireServoIds[i]);
        }
        impl_->haveCommandedServoTargets = true;
        diagnostics = firstAttempt;
        diagnostics.status = ProximalStepStatus::HeldLastGood;
    } else {
        ++impl_->totalRetries;
        ++impl_->totalRollbacks;
        if (const char* trace = std::getenv("HEXAPOD_PROXIMAL_TRACE_FAILURES");
            trace != nullptr && trace[0] != '\0' && trace[0] != '0') {
            std::cerr << "[proximal-first-failure] reason="
                      << static_cast<unsigned>(firstAttempt.failureReason)
                      << " iterations=" << firstAttempt.iterations
                      << " primal=" << firstAttempt.primalResidual
                      << " dual=" << firstAttempt.dualResidual
                      << " comp=" << firstAttempt.complementarityResidual
                      << " ncp_dual=" << firstAttempt.ncpDualResidual
                      << " ncp_comp=" << firstAttempt.ncpComplementarityResidual
                      << " cone=" << firstAttempt.coneResidual
                      << " contacts=" << firstAttempt.contactConstraintCount
                      << " signature=" << firstAttempt.contactSetSignature
                      << " worst_contact=" << firstAttempt.worstContactId
                      << " rho=" << firstAttempt.admmRho
                      << " pre_v=" << firstAttempt.preIntegrationLinearSpeed
                      << " pre_w=" << firstAttempt.preIntegrationAngularSpeed
                      << '\n';
        }
        const auto preRetryWarmStarts = impl_->contactWarmStarts;
        writeState(world, snapshotQ, snapshotV);
        clearFailedSolverState(firstAttempt.worstContactId);
        ProximalStepDiagnostics retry{};
        // Large primal error or a high spectral penalty indicates that the
        // primary Anderson history is oscillating. A shorter history gives
        // the bounded retry a distinct numerical path. For other convergence
        // modes the primary history is more effective. Neither path changes
        // the contact equations, tolerances, or iteration cap.
        const double largePrimalResidual = std::max(
            5.0 * settings.relativeTolerance,
            10.0 * settings.absoluteTolerance);
        const double nearDualFeasibility = 10.0 * settings.absoluteTolerance;
        const bool oscillatoryFailure = firstAttempt.primalResidual > largePrimalResidual
            || (firstAttempt.admmRho > 3.0
                && firstAttempt.ncpDualResidual <= nearDualFeasibility);
        const std::size_t retryAndersonCapacity =
            impl_->retryAndersonCapacityOverride.value_or(
                oscillatoryFailure
                    ? std::min<std::size_t>(impl_->andersonCapacity, 2U)
                    : impl_->andersonCapacity);
        const std::array<double, 18> halfStepServoTargets =
            advanceCommandedTargets(commandedServoTargetsStart, 0.5 * dt);
        const bool half1 = advanceOnce(
            0.5 * dt, halfStepServoTargets, retryAndersonCapacity, retry);
        const bool half2 = half1
            && advanceOnce(
                0.5 * dt, commandedServoTargets, retryAndersonCapacity, retry);
        retry.dynamicsTimeMs += firstAttempt.dynamicsTimeMs;
        retry.contactSetupTimeMs += firstAttempt.contactSetupTimeMs;
        retry.collisionTimeMs += firstAttempt.collisionTimeMs;
        retry.constraintAssemblyTimeMs += firstAttempt.constraintAssemblyTimeMs;
        retry.delassusTimeMs += firstAttempt.delassusTimeMs;
        retry.admmTimeMs += firstAttempt.admmTimeMs;
        retry.integrationTimeMs += firstAttempt.integrationTimeMs;
        if (half1 && half2) {
            world.CompleteExternalDynamicsStep();
            impl_->commandedServoTargets = commandedServoTargets;
            impl_->haveCommandedServoTargets = true;
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::RecoveredRetry;
            // Retain the reason that made recovery necessary. Consumers can
            // now distinguish a clean healthy step from a usable step that
            // recovered solver non-convergence or a speed-limit rejection.
            diagnostics.failureReason = firstAttempt.failureReason;
            readState(world, impl_->lastGoodQ, impl_->lastGoodV);
        } else {
            ++impl_->totalHeldStates;
            ++impl_->totalRollbacks;
            writeState(world, impl_->lastGoodQ, impl_->lastGoodV);
            impl_->contactWarmStarts = preRetryWarmStarts;
            clearFailedSolverState(
                retry.worstContactId != 0
                    ? retry.worstContactId
                    : firstAttempt.worstContactId);
            for (std::size_t i = 0; i < impl_->wireServoIds.size(); ++i) {
                impl_->commandedServoTargets[i] =
                    world.GetServoJointAngle(impl_->wireServoIds[i]);
            }
            impl_->haveCommandedServoTargets = true;
            diagnostics = retry;
            diagnostics.status = ProximalStepStatus::HeldLastGood;
        }
    }

    diagnostics.warmStartResets = impl_->totalWarmStartResets;
    diagnostics.retries = impl_->totalRetries;
    diagnostics.rollbackCount = impl_->totalRollbacks;
    diagnostics.heldStateCount = impl_->totalHeldStates;
    diagnostics.unsupportedIslandCount = impl_->totalUnsupportedIslands;
    diagnostics.totalStepTimeMs = elapsedMs(stepStart);
    return diagnostics.status == ProximalStepStatus::Healthy
        || diagnostics.status == ProximalStepStatus::RecoveredRetry;
}

} // namespace minphys3d::demo
