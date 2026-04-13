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

Body MakeBoxBody(const Vec3& position, const Vec3& half_extents, float mass, bool is_static = false) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.isStatic = is_static;
    body.restitution = 0.0f;
    body.staticFriction = 0.8f;
    body.dynamicFriction = 0.6f;
    body.linearDamping = 0.1f;
    body.angularDamping = 0.3f;
    return body;
}

Body MakeLinkBody(const Vec3& center, const Vec3& half_extents, float mass) {
    return MakeBoxBody(center, half_extents, mass, false);
}

float WrapAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct ServoMetrics {
    float peakLinear = 0.0f;
    float peakAngular = 0.0f;
    float maxError = 0.0f;
    float finalError = 0.0f;
};

ServoMetrics MeasureSingleServo(
    Vec3 gravity,
    const Vec3& axis,
    float target_angle,
    std::uint8_t servo_position_passes,
    bool static_base,
    const Vec3& link_initial_angular_velocity = {}) {
    World world(gravity);
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = servo_position_passes;
    world.SetJointSolverConfig(joint_cfg);

    Body base = MakeBoxBody({0.0f, 0.0f, 0.0f}, {0.20f, 0.20f, 0.20f}, static_base ? 1.0f : 3.0f, static_base);
    base.angularDamping = 0.4f;
    const std::uint32_t base_id = world.CreateBody(base);

    Body link = MakeLinkBody({0.60f, 0.0f, 0.0f}, {0.40f, 0.05f, 0.05f}, 1.0f);
    link.angularVelocity = link_initial_angular_velocity;
    link.angularDamping = 0.4f;
    const std::uint32_t link_id = world.CreateBody(link);

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0f, 0.0f, 0.0f},
        axis,
        target_angle,
        2.5f,
        10.0f,
        1.5f);

    ServoMetrics metrics;
    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
        const Body& current_link = world.GetBody(link_id);
        const ServoJoint& joint = world.GetServoJoint(servo_id);
        const float err = std::abs(WrapAngle(world.GetServoJointAngle(servo_id) - joint.targetAngle));
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
        {"x", {1.0f, 0.0f, 0.0f}},
        {"y", {0.0f, 1.0f, 0.0f}},
        {"z", {0.0f, 0.0f, 1.0f}},
    }};
    const std::array<float, 2> targets{{-0.45f, 0.45f}};
    const std::array<std::uint8_t, 2> passes{{0u, 4u}};

    bool ok = true;
    for (const AxisCase& axis_case : axes) {
        for (const float target : targets) {
            for (const std::uint8_t pass_count : passes) {
                const ServoMetrics metrics =
                    MeasureSingleServo({0.0f, 0.0f, 0.0f}, axis_case.axis, target, pass_count, true);
                if (metrics.finalError > 0.20f) {
                    std::cerr << "axis_target axis=" << axis_case.name << " target=" << target
                              << " passes=" << static_cast<int>(pass_count)
                              << " final_error=" << metrics.finalError << "\n";
                    ok = false;
                }
                if (metrics.peakAngular > 10.0f) {
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
    const std::array<Vec3, 2> gravities{{Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, -9.81f, 0.0f}}};
    const std::array<const char*, 2> labels{{"zero_g", "earth_g"}};

    bool ok = true;
    for (std::size_t i = 0; i < gravities.size(); ++i) {
        const ServoMetrics metrics =
            MeasureSingleServo(gravities[i], {0.0f, 1.0f, 0.0f}, 0.35f, 0u, false, {0.0f, 1.2f, 0.0f});
        if (metrics.maxError > 0.90f) {
            std::cerr << "gravity_mode label=" << labels[i] << " max_error=" << metrics.maxError << "\n";
            ok = false;
        }
        // A dynamic base in Earth gravity will pick up ordinary freefall linear speed, so this
        // regression focuses on rotational boundedness and target tracking instead of COM speed.
        if (metrics.peakAngular > 10.0f) {
            std::cerr << "gravity_mode label=" << labels[i] << " peak_angular=" << metrics.peakAngular << "\n";
            ok = false;
        }
        if (metrics.finalError > 0.30f) {
            std::cerr << "gravity_mode label=" << labels[i] << " final_error=" << metrics.finalError << "\n";
            ok = false;
        }
    }
    return ok;
}

bool CheckMirroredServoSymmetry() {
    World world({0.0f, 0.0f, 0.0f});

    Body base_left = MakeBoxBody({-1.0f, 0.0f, 0.0f}, {0.20f, 0.20f, 0.20f}, 1.0f, true);
    Body base_right = MakeBoxBody({1.0f, 0.0f, 0.0f}, {0.20f, 0.20f, 0.20f}, 1.0f, true);
    const std::uint32_t left_base_id = world.CreateBody(base_left);
    const std::uint32_t right_base_id = world.CreateBody(base_right);

    Body left_link = MakeLinkBody({-0.40f, 0.0f, 0.0f}, {0.40f, 0.05f, 0.05f}, 1.0f);
    Body right_link = MakeLinkBody({0.40f, 0.0f, 0.0f}, {0.40f, 0.05f, 0.05f}, 1.0f);
    const std::uint32_t left_link_id = world.CreateBody(left_link);
    const std::uint32_t right_link_id = world.CreateBody(right_link);

    const std::uint32_t left_servo = world.CreateServoJoint(
        left_base_id,
        left_link_id,
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        0.35f,
        2.5f,
        10.0f,
        1.5f);
    const std::uint32_t right_servo = world.CreateServoJoint(
        right_base_id,
        right_link_id,
        {1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        -0.35f,
        2.5f,
        10.0f,
        1.5f);

    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 20);
    }

    const float left_angle = world.GetServoJointAngle(left_servo);
    const float right_angle = world.GetServoJointAngle(right_servo);
    const float symmetry_error = std::abs(left_angle + right_angle);
    if (symmetry_error > 0.10f) {
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
            MeasureSingleServo({0.0f, -9.81f, 0.0f}, {0.0f, 0.0f, 1.0f}, 0.20f, pass_count, true);
        if (metrics.maxError > 0.35f) {
            std::cerr << "position_passes passes=" << static_cast<int>(pass_count)
                      << " max_error=" << metrics.maxError << "\n";
            ok = false;
        }
        if (metrics.peakLinear > 5.0f) {
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
