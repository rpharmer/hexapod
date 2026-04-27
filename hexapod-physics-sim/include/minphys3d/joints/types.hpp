#pragma once

#include <cstdint>

#include "minphys3d/math/quat.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

struct DistanceJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    float restLength = 1.0f;
    float stiffness = 1.0f;
    float damping = 0.1f;
    float impulseSum = 0.0f;
};

struct HingeJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{0.0f, 1.0f, 0.0f};
    Vec3 localAxisB{0.0f, 1.0f, 0.0f};
    Vec3 localReferenceA{1.0f, 0.0f, 0.0f};
    Vec3 localReferenceB{1.0f, 0.0f, 0.0f};
    float impulseX = 0.0f;
    float impulseY = 0.0f;
    float impulseZ = 0.0f;
    float angularImpulse1 = 0.0f;
    float angularImpulse2 = 0.0f;
    bool limitsEnabled = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    bool motorEnabled = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;
    float motorImpulseSum = 0.0f;
};

struct BallSocketJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    float impulseX = 0.0f;
    float impulseY = 0.0f;
    float impulseZ = 0.0f;
};

struct FixedJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Quat referenceRotation{};
    float impulseX = 0.0f;
    float impulseY = 0.0f;
    float impulseZ = 0.0f;
    float angularImpulseX = 0.0f;
    float angularImpulseY = 0.0f;
    float angularImpulseZ = 0.0f;
};

struct PrismaticJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{1.0f, 0.0f, 0.0f};
    Vec3 localAxisB{1.0f, 0.0f, 0.0f};
    float impulseT1 = 0.0f;
    float impulseT2 = 0.0f;
    float impulseAxis = 0.0f;
    bool limitsEnabled = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;
    bool motorEnabled = false;
    float motorSpeed = 0.0f;
    float maxMotorForce = 0.0f;
    float motorImpulseSum = 0.0f;
};

struct ServoJoint {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{0.0f, 1.0f, 0.0f};
    Vec3 localAxisB{0.0f, 1.0f, 0.0f};
    Vec3 localReferenceA{1.0f, 0.0f, 0.0f};
    Vec3 localReferenceB{1.0f, 0.0f, 0.0f};
    float impulseX = 0.0f;
    float impulseY = 0.0f;
    float impulseZ = 0.0f;
    float angularImpulse1 = 0.0f;
    float angularImpulse2 = 0.0f;
    float servoImpulseSum = 0.0f;
    float targetAngle = 0.0f;
    float maxServoTorque = 0.0f;
    /// Max axis speed (rad/s) the servo bias is allowed to request; 0 disables the clamp.
    float maxServoSpeed = 0.0f;
    float positionGain = 40.0f;   // omega_n: natural frequency (rad/s)
    float dampingGain = 1.0f;     // zeta: damping ratio
    float integralGain = 0.0f;
    float integralClamp = 0.5f;
    float integralAccum = 0.0f;
    /// 0 = off; else each Step: smoothed P-error += min(value,1)*(raw-smoothed).
    float positionErrorSmoothing = 0.0f;
    float smoothedAngleError = 0.0f;
    /// Max position error (rad) fed to bias velocity; limits correction rate for large errors.
    float maxCorrectionAngle = 0.5f;
    /// Scales post-integration positional hinge correction for this joint (1 = default).
    float angleStabilizationScale = 1.0f;
    // Solver-side hysteresis flags for staged early-out gates.
    bool anchorEarlyOutActive = false;
    bool angularEarlyOutActive = false;
    bool hingeEarlyOutActive = false;
};

} // namespace minphys3d
