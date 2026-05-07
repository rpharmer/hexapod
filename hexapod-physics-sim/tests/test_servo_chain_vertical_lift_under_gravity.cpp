#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct VerticalLiftMetrics {
    float peak_payload_height = 0.0f;
    float initial_payload_height = 0.0f;
    float final_angle = 0.0f;
    int first_step_reaching_target = -1;
    bool finite = true;
};

VerticalLiftMetrics runVerticalLift(float max_servo_torque) {
    World world({0.0f, -9.81f, 0.0f});
    const std::uint32_t base_id = world.CreateBody(MakeStaticBase());

    constexpr float kArmLength = 1.0f;
    const std::uint32_t arm_id = world.CreateBody(MakeArmLink({0.5f * kArmLength, 0.0f, 0.0f}, kArmLength, 0.7f));

    Body payload;
    payload.shape = ShapeType::Sphere;
    payload.radius = 0.07f;
    payload.mass = 8.0f;
    payload.position = {kArmLength, 0.0f, 0.0f};
    payload.restitution = 0.0f;
    payload.linearDamping = 0.02f;
    payload.angularDamping = 0.02f;
    const std::uint32_t payload_id = world.CreateBody(payload);
    world.CreateFixedJoint(arm_id, payload_id, {kArmLength, 0.0f, 0.0f});

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        arm_id,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        1.15f,
        max_servo_torque,
        28.0f,
        2.8f);

    VerticalLiftMetrics metrics{};
    metrics.initial_payload_height = world.GetBody(payload_id).position.y;
    constexpr float kDt = 1.0f / 240.0f;
    for (int step = 0; step < 720; ++step) {
        world.Step(kDt, 32);
        const Body& arm = world.GetBody(arm_id);
        const Body& payload_body = world.GetBody(payload_id);
        if (!IsFiniteVec3(arm.position) || !IsFiniteVec3(arm.velocity) || !IsFiniteVec3(payload_body.position)
            || !IsFiniteVec3(payload_body.velocity) || !IsFiniteQuat(arm.orientation)) {
            metrics.finite = false;
            break;
        }
        metrics.peak_payload_height = std::max(metrics.peak_payload_height, payload_body.position.y);
        if (metrics.first_step_reaching_target < 0 && world.GetServoJointAngle(servo_id) >= 1.0f) {
            metrics.first_step_reaching_target = step;
        }
    }
    metrics.final_angle = world.GetServoJointAngle(servo_id);
    return metrics;
}

int runCase() {
    const VerticalLiftMetrics high_torque = runVerticalLift(8.0f);

    if (!high_torque.finite) {
        std::cerr << "vertical_lift encountered non-finite state\n";
        return 1;
    }
    const float high_lift = high_torque.peak_payload_height - high_torque.initial_payload_height;
    if (high_lift < 0.20f) {
        std::cerr << "vertical_lift high_lift=" << high_lift << " floor=0.20\n";
        return 1;
    }
    if (high_lift > 10.0f) {
        std::cerr << "vertical_lift high_lift runaway=" << high_lift << " cap=10\n";
        return 1;
    }
    if (high_torque.first_step_reaching_target < 0) {
        std::cerr << "vertical_lift high torque never reached target window\n";
        return 1;
    }
    if (high_torque.final_angle < 0.50f) {
        std::cerr << "vertical_lift final_angle=" << high_torque.final_angle << " floor=0.50\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
