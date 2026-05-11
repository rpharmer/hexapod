#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct LiftResult {
    Real final_angle = 0.0;
    Real mean_abs_error_first_window = 0.0;
    bool finite = true;
};

LiftResult runLiftCase(Real max_servo_speed_radps) {
    World world({0.0, 0.0, 0.0});
    const std::uint32_t base_id = world.CreateBody(MakeStaticBase());

    constexpr Real kLinkLength = 0.9;
    constexpr Real kLinkMass = 0.6;
    // Start clear of the static base so this remains a servo-rate overload test,
    // not a self-contact fixture.
    const std::uint32_t link_id =
        world.CreateBody(MakeArmLink({0.60, 0.0, 0.0}, kLinkLength, kLinkMass));

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        2.4,
        3.0,
        25.0,
        2.5,
        0.0,
        0.5,
        0.0,
        1.0,
        max_servo_speed_radps,
        0.5);

    LiftResult result{};
    constexpr Real kDt = 1.0 / 240.0;
    constexpr int kWindowSteps = 120;
    Real sum_abs_error = 0.0;
    // 720 steps (~3 s at 240 Hz) crossed a transient where the fast (high ω_cap) trajectory sat
    // *below* the slow-capped one (limit-cycle vs monotone creep). Stop at ~2 s once both
    // contrasts (mean early error + final angle gap) still separate the cases.
    constexpr int kSimSteps = 480;
    for (int step = 0; step < kSimSteps; ++step) {
        world.Step(kDt, 32);
        const Body& link = world.GetBody(link_id);
        if (!IsFiniteVec3(link.position) || !IsFiniteVec3(link.velocity) || !IsFiniteQuat(link.orientation)
            || !IsFiniteVec3(link.angularVelocity)) {
            result.finite = false;
            break;
        }
        const ServoJoint& servo = world.GetServoJoint(servo_id);
        const Real err = std::abs(WrapAngle(world.GetServoJointAngle(servo_id) - servo.targetAngle));
        if (step < kWindowSteps) {
            sum_abs_error += err;
        }
    }
    result.mean_abs_error_first_window = sum_abs_error / static_cast<float>(kWindowSteps);
    result.final_angle = world.GetServoJointAngle(servo_id);
    return result;
}

int runCase() {
    const LiftResult fast = runLiftCase(8.0);
    const LiftResult slow = runLiftCase(0.10);

    if (!fast.finite || !slow.finite) {
        std::cerr << "stall_overload non-finite state encountered\n";
        return 1;
    }
    if (fast.mean_abs_error_first_window > 1.3) {
        std::cerr << "stall_overload fast mean error unexpectedly high=" << fast.mean_abs_error_first_window << "\n";
        return 1;
    }
    if (slow.mean_abs_error_first_window < fast.mean_abs_error_first_window + 0.20) {
        std::cerr << "stall_overload slow-case error contrast too small fast="
                  << fast.mean_abs_error_first_window << " slow=" << slow.mean_abs_error_first_window << "\n";
        return 1;
    }
    if (!std::isfinite(fast.final_angle) || !std::isfinite(slow.final_angle)) {
        std::cerr << "stall_overload final angles non-finite\n";
        return 1;
    }
    // Require a clear separation between capped and uncapped servo settle angles; margin is
    // tight to FP/solver drift (e.g. quaternion-vector rotation implementation).
    constexpr Real kMinFinalAngleSeparation = 0.065;
    if (slow.final_angle > fast.final_angle - kMinFinalAngleSeparation) {
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
