#include <cassert>
#include <cmath>
#include <cstdint>
#include <sstream>

#define private public
#include "minphys3d/core/world.hpp"
#undef private

namespace {

using namespace minphys3d;

constexpr float kTol = 1e-6f;

bool AlmostEqual(float a, float b, float tol = kTol) {
    return std::abs(a - b) <= tol;
}

bool AlmostEqualVec3(const Vec3& a, const Vec3& b, float tol = kTol) {
    return AlmostEqual(a.x, b.x, tol) && AlmostEqual(a.y, b.y, tol) && AlmostEqual(a.z, b.z, tol);
}

bool AlmostOrthogonal(const Vec3& a, const Vec3& b, float tol = 1e-5f) {
    return std::abs(Dot(a, b)) <= tol;
}

Vec3 ComputeExpectedLocalAnchor(const Body& body, const Vec3& worldAnchor) {
    return Rotate(Conjugate(Normalize(body.orientation)), worldAnchor - body.position);
}

void ConfigureDynamicBody(Body& body, const Vec3& position, const Quat& orientation) {
    body.shape = ShapeType::Sphere;
    body.radius = 0.2f;
    body.mass = 1.0f;
    body.position = position;
    body.orientation = Normalize(orientation);
}

} // namespace

int main() {
    World world({0.0f, 0.0f, 0.0f});

    Body bodyA;
    ConfigureDynamicBody(bodyA, {-1.2f, 0.3f, 0.9f}, {0.88f, 0.12f, 0.33f, -0.27f});
    const std::uint32_t a = world.CreateBody(bodyA);

    Body bodyB;
    ConfigureDynamicBody(bodyB, {1.1f, -0.4f, -0.8f}, {0.77f, -0.20f, 0.40f, 0.11f});
    const std::uint32_t b = world.CreateBody(bodyB);

    const Vec3 sharedAnchor{0.25f, 1.10f, -0.55f};
    const Vec3 distanceAnchorA{-0.85f, 0.45f, 0.12f};
    const Vec3 distanceAnchorB{0.95f, -0.35f, 0.90f};

    const std::uint32_t distanceId = world.CreateDistanceJoint(a, b, distanceAnchorA, distanceAnchorB, 2.5f, 0.4f);
    assert(distanceId == 0);
    assert(AlmostEqualVec3(world.joints_[distanceId].localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), distanceAnchorA)));
    assert(AlmostEqualVec3(world.joints_[distanceId].localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), distanceAnchorB)));

    const std::uint32_t ballId = world.CreateBallSocketJoint(a, b, sharedAnchor);
    assert(ballId == 0);
    assert(AlmostEqualVec3(world.ballSocketJoints_[ballId].localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(world.ballSocketJoints_[ballId].localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));

    const std::uint32_t fixedId = world.CreateFixedJoint(a, b, sharedAnchor);
    assert(fixedId == 0);
    assert(AlmostEqualVec3(world.fixedJoints_[fixedId].localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(world.fixedJoints_[fixedId].localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));

    const std::uint32_t initialHingeCount = static_cast<std::uint32_t>(world.hingeJoints_.size());
    const std::uint32_t initialPrismaticCount = static_cast<std::uint32_t>(world.prismaticJoints_.size());
    const std::uint32_t initialServoCount = static_cast<std::uint32_t>(world.servoJoints_.size());

    const Vec3 invalidAxis{0.0f, 0.0f, 0.0f};
    assert(world.CreateHingeJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.CreatePrismaticJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.CreateServoJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.hingeJoints_.size() == initialHingeCount);
    assert(world.prismaticJoints_.size() == initialPrismaticCount);
    assert(world.servoJoints_.size() == initialServoCount);

    const Vec3 validAxis{0.3f, 1.0f, -0.2f};
    const std::uint32_t hingeId = world.CreateHingeJoint(a, b, sharedAnchor, validAxis);
    assert(hingeId == initialHingeCount);
    const HingeJoint& hinge = world.hingeJoints_[hingeId];
    assert(!hinge.limitsEnabled);
    assert(AlmostEqual(hinge.lowerAngle, 0.0f));
    assert(AlmostEqual(hinge.upperAngle, 0.0f));
    assert(!hinge.motorEnabled);
    assert(AlmostEqual(hinge.motorSpeed, 0.0f));
    assert(AlmostEqual(hinge.maxMotorTorque, 0.0f));
    assert(AlmostEqualVec3(hinge.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(hinge.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));
    assert(AlmostOrthogonal(hinge.localAxisA, hinge.localReferenceA));
    assert(AlmostOrthogonal(hinge.localAxisB, hinge.localReferenceB));

    const std::uint32_t prismaticId = world.CreatePrismaticJoint(a, b, sharedAnchor, validAxis);
    assert(prismaticId == initialPrismaticCount);
    const PrismaticJoint& prismatic = world.prismaticJoints_[prismaticId];
    assert(!prismatic.limitsEnabled);
    assert(AlmostEqual(prismatic.lowerTranslation, 0.0f));
    assert(AlmostEqual(prismatic.upperTranslation, 0.0f));
    assert(!prismatic.motorEnabled);
    assert(AlmostEqual(prismatic.motorSpeed, 0.0f));
    assert(AlmostEqual(prismatic.maxMotorForce, 0.0f));
    assert(AlmostEqualVec3(prismatic.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(prismatic.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));

    const std::uint32_t servoId = world.CreateServoJoint(a, b, sharedAnchor, validAxis);
    assert(servoId == initialServoCount);
    const ServoJoint& servo = world.servoJoints_[servoId];
    assert(AlmostEqual(servo.targetAngle, 0.0f));
    assert(AlmostEqual(servo.maxServoTorque, 1.0f));
    assert(AlmostEqual(servo.positionGain, 40.0f));
    assert(AlmostEqual(servo.dampingGain, 1.0f));
    assert(AlmostEqual(servo.integralGain, 0.0f));
    assert(AlmostEqual(servo.integralClamp, 0.5f));
    assert(AlmostEqual(servo.positionErrorSmoothing, 0.0f));
    assert(AlmostEqual(servo.angleStabilizationScale, 1.0f));
    assert(AlmostEqualVec3(servo.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(servo.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));
    assert(AlmostOrthogonal(servo.localAxisA, servo.localReferenceA));
    assert(AlmostOrthogonal(servo.localAxisB, servo.localReferenceB));

    World collisionWorld({0.0f, 0.0f, 0.0f});
    Body connectedA;
    connectedA.shape = ShapeType::Box;
    connectedA.halfExtents = {0.5f, 0.5f, 0.5f};
    connectedA.mass = 1.0f;
    connectedA.position = {0.0f, 0.0f, 0.0f};
    const std::uint32_t connectedAId = collisionWorld.CreateBody(connectedA);

    Body connectedB = connectedA;
    connectedB.position = {0.4f, 0.0f, 0.0f};
    const std::uint32_t connectedBId = collisionWorld.CreateBody(connectedB);

    collisionWorld.CreateHingeJoint(connectedAId, connectedBId, {0.2f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    collisionWorld.contacts_.clear();
    collisionWorld.GenerateContacts();
    assert(collisionWorld.contacts_.empty());

    return 0;
}
