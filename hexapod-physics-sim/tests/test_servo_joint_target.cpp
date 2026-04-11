#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#define private public
#include "minphys3d/core/world.hpp"
#undef private

namespace {

using namespace minphys3d;

float SignedAngleAroundAxis(const Vec3& from, const Vec3& to, const Vec3& axis) {
    Vec3 from_n = from;
    Vec3 to_n = to;
    Vec3 axis_n = axis;
    const bool valid = TryNormalize(from_n, from_n) && TryNormalize(to_n, to_n) && TryNormalize(axis_n, axis_n);
    if (!valid) {
        return 0.0f;
    }
    return std::atan2(Dot(Cross(from_n, to_n), axis_n), Dot(from_n, to_n));
}

} // namespace

int main() {
    World world({0.0f, 0.0f, 0.0f});

    Body base;
    base.shape = ShapeType::Box;
    base.halfExtents = {0.2f, 0.2f, 0.2f};
    base.isStatic = true;
    const std::uint32_t base_id = world.CreateBody(base);

    Body link;
    link.shape = ShapeType::Box;
    link.position = {0.6f, 0.0f, 0.0f};
    link.halfExtents = {0.4f, 0.05f, 0.05f};
    link.mass = 1.0f;
    const std::uint32_t link_id = world.CreateBody(link);

    constexpr float kTargetAngle = 0.45f;
    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        kTargetAngle,
        2.5f,
        10.0f,
        1.5f);
    assert(servo_id == 0);

    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
    }

    const Body& rotated_link = world.GetBody(link_id);
    const Vec3 long_axis = Rotate(rotated_link.orientation, {1.0f, 0.0f, 0.0f});
    const float measured_angle = SignedAngleAroundAxis({1.0f, 0.0f, 0.0f}, long_axis, {0.0f, 1.0f, 0.0f});
    if (std::abs(measured_angle - kTargetAngle) >= 0.20f) {
        std::cerr << "measured_angle=" << measured_angle << " target=" << kTargetAngle << "\n";
        return 1;
    }

    return 0;
}
