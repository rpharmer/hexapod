#include <cassert>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <iostream>

#define private public
#include "minphys3d/core/world.hpp"
#undef private

namespace {

using namespace minphys3d;

Real SignedAngleAroundAxis(const Vec3& from, const Vec3& to, const Vec3& axis) {
    Vec3 from_n = from;
    Vec3 to_n = to;
    Vec3 axis_n = axis;
    const bool valid = TryNormalize(from_n, from_n) && TryNormalize(to_n, to_n) && TryNormalize(axis_n, axis_n);
    if (!valid) {
        return 0.0;
    }
    return std::atan2(Dot(Cross(from_n, to_n), axis_n), Dot(from_n, to_n));
}

bool CheckServoSpeedLimit() {
    World world({0.0, 0.0, 0.0});

    Body base;
    base.shape = ShapeType::Box;
    base.halfExtents = {0.2, 0.2, 0.2};
    base.isStatic = true;
    const std::uint32_t base_id = world.CreateBody(base);

    Body link;
    link.shape = ShapeType::Box;
    link.position = {0.6, 0.0, 0.0};
    link.halfExtents = {0.4, 0.05, 0.05};
    link.mass = 1.0;
    const std::uint32_t link_id = world.CreateBody(link);

    constexpr Real kTargetAngle = 1.10;
    constexpr Real kMaxServoSpeed = 0.15;
    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        kTargetAngle,
        6.0,
        25.0,
        2.0,
        0.0,
        0.5,
        0.0,
        1.0,
        kMaxServoSpeed,
        0.5);
    assert(servo_id == 0);

    constexpr Real kDt = 1.0 / 120.0;
    Real peak_rate = 0.0;
    Real prev_angle = world.GetServoJointAngle(servo_id);
    for (int step = 0; step < 240; ++step) {
        world.Step(kDt, 24);
        const Real angle = world.GetServoJointAngle(servo_id);
        const Real rate = std::abs(std::atan2(std::sin(angle - prev_angle), std::cos(angle - prev_angle))) / kDt;
        peak_rate = std::max(peak_rate, rate);
        prev_angle = angle;
    }

    constexpr Real kAllowedPeakRate = 0.50;
    if (peak_rate > kAllowedPeakRate) {
        std::cerr << "peak_rate=" << peak_rate << " cap=" << kAllowedPeakRate << "\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!CheckServoSpeedLimit()) {
        return 1;
    }

    World world({0.0, 0.0, 0.0});

    Body base;
    base.shape = ShapeType::Box;
    base.halfExtents = {0.2, 0.2, 0.2};
    base.isStatic = true;
    const std::uint32_t base_id = world.CreateBody(base);

    Body link;
    link.shape = ShapeType::Box;
    link.position = {0.6, 0.0, 0.0};
    link.halfExtents = {0.4, 0.05, 0.05};
    link.mass = 1.0;
    const std::uint32_t link_id = world.CreateBody(link);

    constexpr Real kTargetAngle = 0.45;
    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        kTargetAngle,
        2.5,
        10.0,
        1.5);
    assert(servo_id == 0);

    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
    }

    const Body& rotated_link = world.GetBody(link_id);
    const Vec3 long_axis = Rotate(rotated_link.orientation, {1.0, 0.0, 0.0});
    const Real measured_angle = SignedAngleAroundAxis({1.0, 0.0, 0.0}, long_axis, {0.0, 1.0, 0.0});
    if (std::abs(measured_angle - kTargetAngle) >= 0.20) {
        std::cerr << "measured_angle=" << measured_angle << " target=" << kTargetAngle << "\n";
        return 1;
    }

    return 0;
}
