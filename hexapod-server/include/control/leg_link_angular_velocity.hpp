#pragma once

#include "types.hpp"

#include <array>
#include <cmath>

// Angular vectors in PhysicsSimBridge's server BODY convention. The bridge
// applies C=(-z,x,y) to angular vectors as well as polar vectors; det(C)=-1.
// Consequently these axes are the negatives of the position-FK axial axes.
// Do not change the bridge or position FK to make their signs look identical.
// The Pinocchio link-velocity oracle tests this convention for every wire.
inline std::array<Vec3, kJointsPerLeg> legRelativeLinkAngularVelocities(
    const LegGeometry& geometry,
    const LegState& measured_servo,
    const std::array<double, kJointsPerLeg>& servo_rates) {
    const LegState mechanical = geometry.servo.toJointAngles(measured_servo);
    const double yaw = legFrameYawRad(geometry) + mechanical.joint_state[COXA].pos_rad.value;
    const auto sign = [](double value) { return value >= 0.0 ? 1.0 : -1.0; };
    const double coxa = sign(geometry.servo.coxaSign) * servo_rates[COXA];
    const double femur = sign(geometry.servo.femurSign) * servo_rates[FEMUR];
    const double tibia = sign(geometry.servo.tibiaSign) * servo_rates[TIBIA];
    const Vec3 pitch_axis{-std::sin(yaw), std::cos(yaw), 0.0};
    return {{{0.0, 0.0, -coxa},
             {pitch_axis.x * femur, pitch_axis.y * femur, -coxa},
             {pitch_axis.x * (femur + tibia), pitch_axis.y * (femur + tibia), -coxa}}};
}
