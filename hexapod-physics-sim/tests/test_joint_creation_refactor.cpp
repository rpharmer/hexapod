#include <cassert>
#include <cmath>
#include <cstdint>
#include <sstream>

#define private public
#include "minphys3d/core/world.hpp"
#undef private

namespace {

using namespace minphys3d;

constexpr Real kTol = 1e-6;

bool AlmostEqual(Real a, Real b, Real tol = kTol) {
    return std::abs(a - b) <= tol;
}

bool AlmostEqualVec3(const Vec3& a, const Vec3& b, Real tol = kTol) {
    return AlmostEqual(a.x, b.x, tol) && AlmostEqual(a.y, b.y, tol) && AlmostEqual(a.z, b.z, tol);
}

bool AlmostOrthogonal(const Vec3& a, const Vec3& b, Real tol = 1e-5) {
    return std::abs(Dot(a, b)) <= tol;
}

Vec3 ComputeExpectedLocalAnchor(const Body& body, const Vec3& worldAnchor) {
    return Rotate(Conjugate(Normalize(body.orientation)), worldAnchor - body.position);
}

void ConfigureDynamicBody(Body& body, const Vec3& position, const Quat& orientation) {
    body.shape = ShapeType::Sphere;
    body.radius = 0.2;
    body.mass = 1.0;
    body.position = position;
    body.orientation = Normalize(orientation);
}

} // namespace

int main() {
    World world({0.0, 0.0, 0.0});

    Body bodyA;
    ConfigureDynamicBody(bodyA, {-1.2, 0.3, 0.9}, {0.88, 0.12, 0.33, -0.27});
    const std::uint32_t a = world.CreateBody(bodyA);

    Body bodyB;
    ConfigureDynamicBody(bodyB, {1.1, -0.4, -0.8}, {0.77, -0.20, 0.40, 0.11});
    const std::uint32_t b = world.CreateBody(bodyB);

    const Vec3 sharedAnchor{0.25, 1.10, -0.55};
    const Vec3 distanceAnchorA{-0.85, 0.45, 0.12};
    const Vec3 distanceAnchorB{0.95, -0.35, 0.90};

    const std::uint32_t distanceId = world.CreateDistanceJoint(a, b, distanceAnchorA, distanceAnchorB, 2.5, 0.4);
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

    const Vec3 invalidAxis{0.0, 0.0, 0.0};
    assert(world.CreateHingeJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.CreatePrismaticJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.CreateServoJoint(a, b, sharedAnchor, invalidAxis) == World::kInvalidJointId);
    assert(world.hingeJoints_.size() == initialHingeCount);
    assert(world.prismaticJoints_.size() == initialPrismaticCount);
    assert(world.servoJoints_.size() == initialServoCount);

    const Vec3 validAxis{0.3, 1.0, -0.2};
    const std::uint32_t hingeId = world.CreateHingeJoint(a, b, sharedAnchor, validAxis);
    assert(hingeId == initialHingeCount);
    const HingeJoint& hinge = world.hingeJoints_[hingeId];
    assert(!hinge.limitsEnabled);
    assert(AlmostEqual(hinge.lowerAngle, 0.0));
    assert(AlmostEqual(hinge.upperAngle, 0.0));
    assert(!hinge.motorEnabled);
    assert(AlmostEqual(hinge.motorSpeed, 0.0));
    assert(AlmostEqual(hinge.maxMotorTorque, 0.0));
    assert(AlmostEqualVec3(hinge.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(hinge.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));
    assert(AlmostOrthogonal(hinge.localAxisA, hinge.localReferenceA));
    assert(AlmostOrthogonal(hinge.localAxisB, hinge.localReferenceB));

    const std::uint32_t prismaticId = world.CreatePrismaticJoint(a, b, sharedAnchor, validAxis);
    assert(prismaticId == initialPrismaticCount);
    const PrismaticJoint& prismatic = world.prismaticJoints_[prismaticId];
    assert(!prismatic.limitsEnabled);
    assert(AlmostEqual(prismatic.lowerTranslation, 0.0));
    assert(AlmostEqual(prismatic.upperTranslation, 0.0));
    assert(!prismatic.motorEnabled);
    assert(AlmostEqual(prismatic.motorSpeed, 0.0));
    assert(AlmostEqual(prismatic.maxMotorForce, 0.0));
    assert(AlmostEqualVec3(prismatic.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(prismatic.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));

    const std::uint32_t servoId = world.CreateServoJoint(a, b, sharedAnchor, validAxis);
    assert(servoId == initialServoCount);
    const ServoJoint& servo = world.servoJoints_[servoId];
    assert(AlmostEqual(servo.targetAngle, 0.0));
    assert(AlmostEqual(servo.maxServoTorque, 1.0));
    assert(AlmostEqual(servo.positionGain, 40.0));
    assert(AlmostEqual(servo.dampingGain, 1.0));
    assert(AlmostEqual(servo.integralGain, 0.0));
    assert(AlmostEqual(servo.integralClamp, 0.5));
    assert(AlmostEqual(servo.positionErrorSmoothing, 0.0));
    assert(AlmostEqual(servo.angleStabilizationScale, 1.0));
    assert(AlmostEqualVec3(servo.localAnchorA, ComputeExpectedLocalAnchor(world.GetBody(a), sharedAnchor)));
    assert(AlmostEqualVec3(servo.localAnchorB, ComputeExpectedLocalAnchor(world.GetBody(b), sharedAnchor)));
    assert(AlmostOrthogonal(servo.localAxisA, servo.localReferenceA));
    assert(AlmostOrthogonal(servo.localAxisB, servo.localReferenceB));

    World collisionWorld({0.0, 0.0, 0.0});
    Body connectedA;
    connectedA.shape = ShapeType::Box;
    connectedA.halfExtents = {0.5, 0.5, 0.5};
    connectedA.mass = 1.0;
    connectedA.position = {0.0, 0.0, 0.0};
    const std::uint32_t connectedAId = collisionWorld.CreateBody(connectedA);

    Body connectedB = connectedA;
    connectedB.position = {0.4, 0.0, 0.0};
    const std::uint32_t connectedBId = collisionWorld.CreateBody(connectedB);

    collisionWorld.CreateHingeJoint(connectedAId, connectedBId, {0.2, 0.0, 0.0}, {0.0, 1.0, 0.0});
    collisionWorld.contacts_.clear();
    collisionWorld.GenerateContacts();
    assert(collisionWorld.contacts_.empty());

    return 0;
}
