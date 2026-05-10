#pragma once

#include <cstdint>
#include <limits>

#include "minphys3d/math/quat.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

struct DistanceJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Real restLength = 1.0;
    Real stiffness = 1.0;
    Real damping = 0.1;
    Real impulseSum = 0.0;
};

struct HingeJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{0.0, 1.0, 0.0};
    Vec3 localAxisB{0.0, 1.0, 0.0};
    Vec3 localReferenceA{1.0, 0.0, 0.0};
    Vec3 localReferenceB{1.0, 0.0, 0.0};
    Real impulseX = 0.0;
    Real impulseY = 0.0;
    Real impulseZ = 0.0;
    Real angularImpulse1 = 0.0;
    Real angularImpulse2 = 0.0;
    bool limitsEnabled = false;
    Real lowerAngle = 0.0;
    Real upperAngle = 0.0;
    bool motorEnabled = false;
    Real motorSpeed = 0.0;
    Real maxMotorTorque = 0.0;
    Real motorImpulseSum = 0.0;
};

struct BallSocketJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Real impulseX = 0.0;
    Real impulseY = 0.0;
    Real impulseZ = 0.0;
};

struct FixedJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Quat referenceRotation{};
    Real impulseX = 0.0;
    Real impulseY = 0.0;
    Real impulseZ = 0.0;
    Real angularImpulseX = 0.0;
    Real angularImpulseY = 0.0;
    Real angularImpulseZ = 0.0;
};

struct PrismaticJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{1.0, 0.0, 0.0};
    Vec3 localAxisB{1.0, 0.0, 0.0};
    Real impulseT1 = 0.0;
    Real impulseT2 = 0.0;
    Real impulseAxis = 0.0;
    bool limitsEnabled = false;
    Real lowerTranslation = 0.0;
    Real upperTranslation = 0.0;
    bool motorEnabled = false;
    Real motorSpeed = 0.0;
    Real maxMotorForce = 0.0;
    Real motorImpulseSum = 0.0;
};

struct ServoJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{0.0, 1.0, 0.0};
    Vec3 localAxisB{0.0, 1.0, 0.0};
    Vec3 localReferenceA{1.0, 0.0, 0.0};
    Vec3 localReferenceB{1.0, 0.0, 0.0};
    Real impulseX = 0.0;
    Real impulseY = 0.0;
    Real impulseZ = 0.0;
    Real angularImpulse1 = 0.0;
    Real angularImpulse2 = 0.0;
    Real servoImpulseSum = 0.0;
    Real targetAngle = 0.0;
    Real maxServoTorque = 0.0;
    /// Max axis speed (rad/s) the servo bias is allowed to request; 0 disables the clamp.
    Real maxServoSpeed = 0.0;
    Real positionGain = 40.0;   // omega_n: natural frequency (rad/s)
    Real dampingGain = 1.0;     // zeta: damping ratio
    Real integralGain = 0.0;
    Real integralClamp = 0.5;
    Real integralAccum = 0.0;
    /// 0 = off; else each Step: smoothed P-error += min(value,1)*(raw-smoothed).
    Real positionErrorSmoothing = 0.0;
    Real smoothedAngleError = 0.0;
    /// Max position error (rad) fed to bias velocity; limits correction rate for large errors.
    Real maxCorrectionAngle = 0.5;
    /// Optional hinge-axis torque (N·m) added to articulated `u` in `PrepareArticulatedInertias`
    /// (parent `localAxisA` / world hinge axis). Default 0 (PD-only path).
    Real articulationDriveTorque = 0.0;
    /// Scales post-integration positional hinge correction for this joint (1 = default).
    Real angleStabilizationScale = 1.0;
    // Solver-side hysteresis flags for staged early-out gates.
    bool anchorEarlyOutActive = false;
    bool angularEarlyOutActive = false;
    bool hingeEarlyOutActive = false;

    // Optional axis-sharing optimisation: when set to a valid joint index (< this joint's
    // own index in the servoJoints_ array), PrepareServoJointSolves copies {axisA, t1, t2}
    // from the named joint's prep instead of recomputing from b.orientation. Exploits the
    // geometric invariant that a slave joint's rotation axis equals that of its master when
    // the intermediate body only rotates around that axis (e.g. femur → tibia in a leg).
    // Set to kNoMasterAxis (default) to disable.
    static constexpr std::uint32_t kNoMasterAxis = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t masterAxisJointIdx = kNoMasterAxis;

    // Set to true by BuildArticulationChains() when this joint belongs to a detected
    // serial kinematic chain. Used by PrepareArticulatedInertias() to include the joint
    // in ABI passes and optional articulated pre-correction; it does not override the
    // hinge-only `invDenomHinge` used by the legacy servo impulse solve.
    bool inArticulationChain = false;
};

} // namespace minphys3d
