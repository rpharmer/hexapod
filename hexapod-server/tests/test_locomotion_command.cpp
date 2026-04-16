#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.cmd_vx_mps = LinearRateMps{10.0};
    intent.cmd_vy_mps = LinearRateMps{0.0};
    intent.cmd_yaw_radps = AngularRateRadPerSec{0.0};
    const PlanarMotionCommand planar = planarMotionCommand(intent);
    const BodyTwist raw = rawLocomotionTwistFromIntent(intent, planar);

    control_config::LocomotionCommandConfig cfg{};
    cfg.max_abs_linear_x_mps = 0.4;
    const BodyTwist c = clampLocomotionTwist(raw, cfg);
    if (!nearlyEq(c.linear_mps.x, 0.4)) {
        std::cerr << "FAIL: clamp linear x\n";
        return EXIT_FAILURE;
    }

    LocomotionCommandProcessor proc(cfg);
    intent.timestamp_us = TimePointUs{1000};
    const BodyTwist a0 = proc.update(intent, planar, intent.timestamp_us);
    if (!nearlyEq(a0.linear_mps.x, 0.4)) {
        std::cerr << "FAIL: first walk frame should snap to clamped target\n";
        return EXIT_FAILURE;
    }

    intent.timestamp_us = TimePointUs{5000};
    intent.cmd_vx_mps = LinearRateMps{0.2};
    const PlanarMotionCommand planar2 = planarMotionCommand(intent);
    const BodyTwist a1 = proc.update(intent, planar2, intent.timestamp_us);
    if (a1.linear_mps.x >= a0.linear_mps.x) {
        std::cerr << "FAIL: filtered command should move toward lower vx\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
