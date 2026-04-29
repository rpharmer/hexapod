#include "command_governor.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

MotionIntent makeWalkIntent(const double vx_mps, const double yaw_rate_radps, const uint64_t timestamp_us) {
    MotionIntent intent = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.14);
    intent.cmd_vx_mps = LinearRateMps{vx_mps};
    intent.cmd_vy_mps = LinearRateMps{0.0};
    intent.cmd_yaw_radps = AngularRateRadPerSec{yaw_rate_radps};
    intent.speed_mps = LinearRateMps{vx_mps};
    intent.heading_rad = AngleRad{0.0};
    intent.twist.body_trans_m.z = 0.14;
    intent.timestamp_us = TimePointUs{timestamp_us};
    intent.sample_id = timestamp_us;
    return intent;
}

RobotState makeState(double tilt_rad,
                     double body_rate_radps,
                     double trust,
                     double mismatch_ratio) {
    RobotState est{};
    est.valid = true;
    est.timestamp_us = TimePointUs{1'000'000};
    est.has_body_twist_state = true;
    est.body_twist_state.twist_pos_rad.x = tilt_rad * 0.7;
    est.body_twist_state.twist_pos_rad.y = tilt_rad * 0.7;
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.gyro_radps.x = body_rate_radps * 0.7;
    est.imu.gyro_radps.y = body_rate_radps * 0.7;
    est.has_fusion_diagnostics = true;
    est.fusion.model_trust = trust;
    est.fusion.residuals.contact_mismatch_ratio = mismatch_ratio;
    return est;
}

GaitState makePreviousGait(double static_margin_m, double clearance_m) {
    GaitState gait{};
    gait.timestamp_us = TimePointUs{1'000'000};
    gait.static_stability_margin_m = static_margin_m;
    gait.support_liftoff_clearance_m.fill(clearance_m);
    gait.support_liftoff_safe_to_lift.fill(clearance_m > 0.0);
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    return gait;
}

} // namespace

int main() {
    CommandGovernor governor{};
    SafetyState healthy_safety{};
    healthy_safety.inhibit_motion = false;
    const GaitState steady_gait = makePreviousGait(0.024, 0.018);
    const RobotState healthy_state = makeState(0.03, 0.12, 0.92, 0.04);

    MotionIntent low_speed = makeWalkIntent(0.06, 0.0, 1'000'000);
    const CommandGovernorState low_out =
        governor.apply(healthy_state, low_speed, healthy_safety, steady_gait);
    if (!expect(low_out.command_scale > 0.95, "low-speed command should pass through almost unchanged") ||
        !expect(low_out.cadence_scale > 0.92, "low-speed command should not be cadence-throttled") ||
        !expect(low_out.swing_height_floor_m >= gaitPresetSwingHeightFloor(GaitType::TRIPOD),
                "low-speed command should preserve the preset swing floor")) {
        return EXIT_FAILURE;
    }

    MotionIntent aggressive = makeWalkIntent(0.34, 0.68, 1'004'000);
    const RobotState stressed_state = makeState(0.24, 1.25, 0.52, 0.34);
    const GaitState stressed_gait = makePreviousGait(-0.006, -0.010);
    const CommandGovernorState stressed_out =
        governor.apply(stressed_state, aggressive, healthy_safety, stressed_gait);

    if (!expect(stressed_out.command_scale < low_out.command_scale,
                "worse stability should attenuate the commanded speed") ||
        !expect(stressed_out.cadence_scale <= low_out.cadence_scale,
                "worse stability should not increase cadence") ||
        !expect(stressed_out.swing_height_floor_m >= low_out.swing_height_floor_m,
                "worse stability should not reduce swing floor") ||
        !expect(stressed_out.body_height_delta_m <= low_out.body_height_delta_m,
                "worse stability should squat the body more, not less") ||
        !expect(stressed_out.command_scale > 0.25,
                "aggressive motion should be reduced progressively, not collapsed to zero")) {
        return EXIT_FAILURE;
    }

    MotionIntent boundary_low = makeWalkIntent(0.119, 0.24, 2'000'000);
    MotionIntent boundary_high = makeWalkIntent(0.121, 0.26, 2'004'000);
    CommandGovernor boundary_low_governor{};
    CommandGovernor boundary_high_governor{};
    const CommandGovernorState boundary_low_out =
        boundary_low_governor.apply(healthy_state, boundary_low, healthy_safety, steady_gait);
    const CommandGovernorState boundary_high_out =
        boundary_high_governor.apply(healthy_state, boundary_high, healthy_safety, steady_gait);

    if (!expect(std::abs(boundary_low_out.command_scale - boundary_high_out.command_scale) < 0.05,
                "command governor should remain continuous near the low-speed boundary") ||
        !expect(std::abs(boundary_low_out.cadence_scale - boundary_high_out.cadence_scale) < 0.05,
                "cadence shaping should remain continuous near the low-speed boundary")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
