#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/multibody/joint.hpp>

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

} // namespace minphys3d::demo
