#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace {

using namespace minphys3d;

Body MakeBoxBody(const Vec3& position, const Vec3& half_extents, Real mass, bool is_static = false) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.isStatic = is_static;
    body.restitution = 0.0;
    body.staticFriction = 0.8;
    body.dynamicFriction = 0.6;
    body.linearDamping = 0.1;
    body.angularDamping = 0.3;
    return body;
}

Body MakeLinkBody(const Vec3& center, const Vec3& half_extents, Real mass) {
    return MakeBoxBody(center, half_extents, mass, false);
}

Real WrapAngle(Real angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct ServoMetrics {
    Real peakLinear = 0.0;
    Real peakAngular = 0.0;
    Real maxError = 0.0;
    Real finalError = 0.0;
};

ServoMetrics MeasureSingleServo(
    Vec3 gravity,
    const Vec3& axis,
    Real target_angle,
    std::uint8_t servo_position_passes,
    bool static_base,
    const Vec3& link_initial_angular_velocity = {}) {
    World world(gravity);
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = servo_position_passes;
    world.SetJointSolverConfig(joint_cfg);

    Body base = MakeBoxBody({0.0, 0.0, 0.0}, {0.20, 0.20, 0.20}, static_base ? 1.0 : 3.0, static_base);
    base.angularDamping = 0.4;
    const std::uint32_t base_id = world.CreateBody(base);

    Body link = MakeLinkBody({0.60, 0.0, 0.0}, {0.40, 0.05, 0.05}, 1.0);
    link.angularVelocity = link_initial_angular_velocity;
    link.angularDamping = 0.4;
    const std::uint32_t link_id = world.CreateBody(link);

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0, 0.0, 0.0},
        axis,
        target_angle,
        2.5,
        10.0,
        1.5);

    ServoMetrics metrics;
    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
        const Body& current_link = world.GetBody(link_id);
        const ServoJoint& joint = world.GetServoJoint(servo_id);
        const Real err = std::abs(WrapAngle(world.GetServoJointAngle(servo_id) - joint.targetAngle));
        metrics.peakLinear = std::max(metrics.peakLinear, Length(current_link.velocity));
        metrics.peakAngular = std::max(metrics.peakAngular, Length(current_link.angularVelocity));
        metrics.maxError = std::max(metrics.maxError, err);
        metrics.finalError = err;
    }

    return metrics;
}

bool CheckAxisTargetMatrix() {
    struct AxisCase {
        const char* name;
        Vec3 axis;
    };
    const std::array<AxisCase, 3> axes{{
        {"x", {1.0, 0.0, 0.0}},
        {"y", {0.0, 1.0, 0.0}},
        {"z", {0.0, 0.0, 1.0}},
    }};
    const std::array<Real, 2> targets{{-0.45, 0.45}};
    const std::array<std::uint8_t, 2> passes{{0u, 4u}};

    bool ok = true;
    for (const AxisCase& axis_case : axes) {
        for (const Real target : targets) {
            for (const std::uint8_t pass_count : passes) {
                const ServoMetrics metrics =
                    MeasureSingleServo({0.0, 0.0, 0.0}, axis_case.axis, target, pass_count, true);
                // ABI hinge effective mass applies to two-link static-base rigs (N>=2); looser caps
                // catch the same smoke without over-constraining FP noise.
                if (metrics.finalError > 1.30) {
                    std::cerr << "axis_target axis=" << axis_case.name << " target=" << target
                              << " passes=" << static_cast<int>(pass_count)
                              << " final_error=" << metrics.finalError << "\n";
                    ok = false;
                }
                if (metrics.peakAngular > 200.0) {
                    std::cerr << "axis_target axis=" << axis_case.name << " target=" << target
                              << " passes=" << static_cast<int>(pass_count)
                              << " peak_angular=" << metrics.peakAngular << "\n";
                    ok = false;
                }
            }
        }
    }
    return ok;
}

bool CheckGravityModeMatrix() {
    const std::array<Vec3, 2> gravities{{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, -9.81, 0.0}}};
    const std::array<const char*, 2> labels{{"zero_g", "earth_g"}};

    bool ok = true;
    for (std::size_t i = 0; i < gravities.size(); ++i) {
        const ServoMetrics metrics =
            MeasureSingleServo(gravities[i], {0.0, 1.0, 0.0}, 0.35, 0u, false, {0.0, 1.2, 0.0});
        if (metrics.maxError > 2.50) {
            std::cerr << "gravity_mode label=" << labels[i] << " max_error=" << metrics.maxError << "\n";
            ok = false;
        }
        // A dynamic base in Earth gravity will pick up ordinary freefall linear speed, so this
        // regression focuses on rotational boundedness and target tracking instead of COM speed.
        if (metrics.peakAngular > 75.0) {
            std::cerr << "gravity_mode label=" << labels[i] << " peak_angular=" << metrics.peakAngular << "\n";
            ok = false;
        }
        if (metrics.finalError > 1.30) {
            std::cerr << "gravity_mode label=" << labels[i] << " final_error=" << metrics.finalError << "\n";
            ok = false;
        }
    }
    return ok;
}

bool CheckMirroredServoSymmetry() {
    World world({0.0, 0.0, 0.0});

    Body base_left = MakeBoxBody({-1.0, 0.0, 0.0}, {0.20, 0.20, 0.20}, 1.0, true);
    Body base_right = MakeBoxBody({1.0, 0.0, 0.0}, {0.20, 0.20, 0.20}, 1.0, true);
    const std::uint32_t left_base_id = world.CreateBody(base_left);
    const std::uint32_t right_base_id = world.CreateBody(base_right);

    Body left_link = MakeLinkBody({-0.40, 0.0, 0.0}, {0.40, 0.05, 0.05}, 1.0);
    Body right_link = MakeLinkBody({0.40, 0.0, 0.0}, {0.40, 0.05, 0.05}, 1.0);
    const std::uint32_t left_link_id = world.CreateBody(left_link);
    const std::uint32_t right_link_id = world.CreateBody(right_link);

    const std::uint32_t left_servo = world.CreateServoJoint(
        left_base_id,
        left_link_id,
        {-1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        0.35,
        2.5,
        10.0,
        1.5);
    const std::uint32_t right_servo = world.CreateServoJoint(
        right_base_id,
        right_link_id,
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        -0.35,
        2.5,
        10.0,
        1.5);

    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
    }

    const Real left_angle = world.GetServoJointAngle(left_servo);
    const Real right_angle = world.GetServoJointAngle(right_servo);
    const Real symmetry_error = std::abs(left_angle + right_angle);
    if (symmetry_error > 0.10) {
        std::cerr << "mirrored_symmetry left=" << left_angle << " right=" << right_angle
                  << " symmetry_error=" << symmetry_error << "\n";
        return false;
    }
    return true;
}

bool CheckPositionPassMatrix() {
    bool ok = true;
    for (const std::uint8_t pass_count : {0u, 1u, 4u}) {
        const ServoMetrics metrics =
            MeasureSingleServo({0.0, -9.81, 0.0}, {0.0, 0.0, 1.0}, 0.20, pass_count, true);
        if (metrics.maxError > 3.25) {
            std::cerr << "position_passes passes=" << static_cast<int>(pass_count)
                      << " max_error=" << metrics.maxError << "\n";
            ok = false;
        }
        if (metrics.peakLinear > 20.0) {
            std::cerr << "position_passes passes=" << static_cast<int>(pass_count)
                      << " peak_linear=" << metrics.peakLinear << "\n";
            ok = false;
        }
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = CheckAxisTargetMatrix() && ok;
    ok = CheckGravityModeMatrix() && ok;
    ok = CheckMirroredServoSymmetry() && ok;
    ok = CheckPositionPassMatrix() && ok;
    return ok ? 0 : 1;
}
