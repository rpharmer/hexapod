#pragma once

#include <cstdint>

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

} // namespace minphys3d
