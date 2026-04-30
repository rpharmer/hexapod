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

    {
        control_config::LocomotionCommandConfig legacy_cfg = cfg;
        legacy_cfg.enable_chassis_accel_limit = false;
        legacy_cfg.enable_first_order_filter = false;
        LocomotionCommandProcessor proc(legacy_cfg);

        intent.timestamp_us = TimePointUs{1000};
        const BodyTwist a0 = proc.update(intent, planar, intent.timestamp_us);
        if (!nearlyEq(a0.linear_mps.x, 0.4)) {
            std::cerr << "FAIL: legacy path should snap first walk frame to clamped target\n";
            return EXIT_FAILURE;
        }
    }

    {
        control_config::LocomotionCommandConfig slew_cfg = cfg;
        slew_cfg.enable_chassis_accel_limit = true;
        slew_cfg.enable_first_order_filter = false;
        slew_cfg.max_linear_accel_xy_mps2 = 2.0;
        slew_cfg.nominal_dt_s = 0.004;

        LocomotionCommandProcessor proc(slew_cfg);
        intent.cmd_vx_mps = LinearRateMps{10.0};
        intent.timestamp_us = TimePointUs{1000};
        const PlanarMotionCommand p0 = planarMotionCommand(intent);
        const BodyTwist s0 = proc.update(intent, p0, intent.timestamp_us);
        const double max_step0 = slew_cfg.max_linear_accel_xy_mps2 * slew_cfg.nominal_dt_s;
        if (!(s0.linear_mps.x > 0.0) || !(s0.linear_mps.x <= max_step0 * 1.0001)) {
            std::cerr << "FAIL: first chassis-slew frame should move at most max_linear_accel_xy * dt\n";
            return EXIT_FAILURE;
        }

        TimePointUs t{1000};
        MotionIntent ramp = intent;
        ramp.cmd_vx_mps = LinearRateMps{0.4};
        for (int i = 0; i < 120; ++i) {
            t = TimePointUs{t.value + 4000};
            const PlanarMotionCommand pr = planarMotionCommand(ramp);
            (void)proc.update(ramp, pr, t);
        }
        const PlanarMotionCommand p_high = planarMotionCommand(ramp);
        const BodyTwist at_high = proc.update(ramp, p_high, TimePointUs{t.value + 4000});
        if (!(at_high.linear_mps.x > 0.35)) {
            std::cerr << "FAIL: sled command should approach high vx after many steps\n";
            return EXIT_FAILURE;
        }

        MotionIntent low = ramp;
        low.cmd_vx_mps = LinearRateMps{0.05};
        const BodyTwist down =
            proc.update(low, planarMotionCommand(low), TimePointUs{t.value + 8000});
        if (!(down.linear_mps.x < at_high.linear_mps.x)) {
            std::cerr << "FAIL: chassis slew should decrease vx when command drops\n";
            return EXIT_FAILURE;
        }
    }

    {
        control_config::LocomotionCommandConfig slew_cfg = cfg;
        slew_cfg.enable_chassis_accel_limit = true;
        slew_cfg.enable_first_order_filter = false;

        LocomotionCommandProcessor proc(slew_cfg);
        MotionIntent walk{};
        walk.requested_mode = RobotMode::WALK;
        walk.cmd_vx_mps = LinearRateMps{0.35};
        walk.cmd_vy_mps = LinearRateMps{0.0};
        walk.cmd_yaw_radps = AngularRateRadPerSec{0.0};
        MotionIntent stand = walk;
        stand.requested_mode = RobotMode::STAND;
        stand.cmd_vx_mps = LinearRateMps{0.0};
        stand.cmd_vy_mps = LinearRateMps{0.0};
        stand.cmd_yaw_radps = AngularRateRadPerSec{0.0};
        TimePointUs t{5000};
        BodyTwist before_stop{};
        for (int i = 0; i < 90; ++i) {
            t = TimePointUs{t.value + 4000};
            before_stop = proc.update(walk, planarMotionCommand(walk), t);
        }
        t = TimePointUs{t.value + 4000};
        const BodyTwist after_stop = proc.update(stand, planarMotionCommand(stand), t);
        if (!(after_stop.linear_mps.x <= before_stop.linear_mps.x)) {
            std::cerr << "FAIL: stand intent should slew planar vx down toward zero\n";
            return EXIT_FAILURE;
        }
    }

    ScenarioMotionIntent scenario{};
    scenario.enabled = true;
    scenario.mode = RobotMode::WALK;
    scenario.gait = GaitType::TRIPOD;
    scenario.speed_mps = 0.25;
    scenario.heading_rad = 0.0;
    const MotionIntent scenario_intent = makeMotionIntent(scenario);
    if (!nearlyEq(scenario_intent.cmd_vx_mps.value, 0.25) ||
        !nearlyEq(scenario_intent.cmd_vy_mps.value, 0.0)) {
        std::cerr << "FAIL: scenario motion should preserve the MotionIntent forward convention\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
