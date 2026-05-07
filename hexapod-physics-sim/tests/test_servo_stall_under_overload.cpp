#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct LiftResult {
    float final_angle = 0.0f;
    float mean_abs_error_first_window = 0.0f;
    bool finite = true;
};

LiftResult runLiftCase(float max_servo_speed_radps) {
    World world({0.0f, 0.0f, 0.0f});
    const std::uint32_t base_id = world.CreateBody(MakeStaticBase());

    constexpr float kLinkLength = 0.9f;
    constexpr float kLinkMass = 0.6f;
    // Start clear of the static base so this remains a servo-rate overload test,
    // not a self-contact fixture.
    const std::uint32_t link_id =
        world.CreateBody(MakeArmLink({0.60f, 0.0f, 0.0f}, kLinkLength, kLinkMass));

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        2.4f,
        3.0f,
        25.0f,
        2.5f,
        0.0f,
        0.5f,
        0.0f,
        1.0f,
        max_servo_speed_radps,
        0.5f);

    LiftResult result{};
    constexpr float kDt = 1.0f / 240.0f;
    constexpr int kWindowSteps = 120;
    float sum_abs_error = 0.0f;
    for (int step = 0; step < 720; ++step) {
        world.Step(kDt, 32);
        const Body& link = world.GetBody(link_id);
        if (!IsFiniteVec3(link.position) || !IsFiniteVec3(link.velocity) || !IsFiniteQuat(link.orientation)
            || !IsFiniteVec3(link.angularVelocity)) {
            result.finite = false;
            break;
        }
        const ServoJoint& servo = world.GetServoJoint(servo_id);
        const float err = std::abs(WrapAngle(world.GetServoJointAngle(servo_id) - servo.targetAngle));
        if (step < kWindowSteps) {
            sum_abs_error += err;
        }
    }
    result.mean_abs_error_first_window = sum_abs_error / static_cast<float>(kWindowSteps);
    result.final_angle = world.GetServoJointAngle(servo_id);
    return result;
}

int runCase() {
    const LiftResult fast = runLiftCase(8.0f);
    const LiftResult slow = runLiftCase(0.10f);

    if (!fast.finite || !slow.finite) {
        std::cerr << "stall_overload non-finite state encountered\n";
        return 1;
    }
    if (fast.mean_abs_error_first_window > 1.3f) {
        std::cerr << "stall_overload fast mean error unexpectedly high=" << fast.mean_abs_error_first_window << "\n";
        return 1;
    }
    if (slow.mean_abs_error_first_window < fast.mean_abs_error_first_window + 0.20f) {
        std::cerr << "stall_overload slow-case error contrast too small fast="
                  << fast.mean_abs_error_first_window << " slow=" << slow.mean_abs_error_first_window << "\n";
        return 1;
    }
    if (!std::isfinite(fast.final_angle) || !std::isfinite(slow.final_angle)) {
        std::cerr << "stall_overload final angles non-finite\n";
        return 1;
    }
    if (slow.final_angle > fast.final_angle - 0.20f) {
        std::cerr << "stall_overload slow final angle too close fast=" << fast.final_angle
                  << " slow=" << slow.final_angle << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
