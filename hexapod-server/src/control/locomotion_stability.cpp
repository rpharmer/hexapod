#include "locomotion_stability.hpp"

#include "config/geometry_config.hpp"
#include "gait_params.hpp"
#include "motion_intent_utils.hpp"
#include "support_assessment.hpp"

#include <algorithm>
#include <cmath>

namespace {

double planarBodyRateRadps(const RobotState& est) {
    if (!est.has_imu || !est.imu.valid) {
        return 0.0;
    }
    return std::hypot(est.imu.gyro_radps.x, est.imu.gyro_radps.y);
}

} // namespace

LocomotionStability::LocomotionStability(LocomotionStabilityConfig config)
    : config_(config) {}

void LocomotionStability::reset() {
}

void LocomotionStability::apply(const RobotState& est, const MotionIntent& intent, GaitState& gait) {
    gait.stability_hold_stance.fill(false);
    gait.support_liftoff_clearance_m.fill(0.0);
    gait.support_liftoff_safe_to_lift.fill(false);
    gait.static_stability_margin_m = 0.0;

    const bool walking = (intent.requested_mode == RobotMode::WALK);
    if (!walking) {
        return;
    }

    const HexapodGeometry geo = geometry_config::activeHexapodGeometry();
    const SupportAssessment support = assessSupportState(est, intent, gait, geo);
    gait.static_stability_margin_m = support.static_margin_m;

    double margin_need = config_.min_margin_required_m;
    bool emergency_tilt_hold = false;
    double tilt_scale = 1.0;
    if (est.has_body_twist_state) {
        const double roll = est.body_twist_state.twist_pos_rad.x;
        const double pitch = est.body_twist_state.twist_pos_rad.y;
        if (std::isfinite(roll) && std::isfinite(pitch)) {
            const double tilt_mag = std::hypot(roll, pitch);
            // When the chassis is already leaning, require extra support margin before allowing
            // another leg to leave the ground. This keeps the gait from chasing a growing tilt
            // with the same lift pattern that caused it.
            margin_need += std::clamp(0.04 * tilt_mag, 0.0, 0.05);
            const double tilt_over = std::max(0.0, tilt_mag - 0.12);
            tilt_scale = std::clamp(1.0 - 0.75 * tilt_over, 0.45, 1.0);
            emergency_tilt_hold = tilt_mag > 0.30;
        }
    }
    if (gait.stride_phase_rate_hz.value < config_.slow_stride_hz_threshold) {
        margin_need *= config_.slow_gait_margin_multiplier;
    }

    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const double commanded_planar_speed_mps = std::hypot(cmd.vx_mps, cmd.vy_mps);
    const double commanded_yaw_rate_radps = std::abs(cmd.yaw_rate_radps);
    const bool high_activity_command = commanded_planar_speed_mps >= 0.18 || commanded_yaw_rate_radps >= 0.35;

    double body_rate_scale = 1.0;
    if (high_activity_command && est.has_imu && est.imu.valid) {
        const double body_rate = planarBodyRateRadps(est);
        if (std::isfinite(body_rate)) {
            constexpr double kBodyRateSoftStartRadps = 0.60;
            constexpr double kBodyRateScaleSlope = 0.45;
            const double body_rate_over = std::max(0.0, body_rate - kBodyRateSoftStartRadps);
            body_rate_scale = std::clamp(1.0 - kBodyRateScaleSlope * body_rate_over, 0.45, 1.0);
        }
    }

    const double gait_activity_scale = std::min(tilt_scale, body_rate_scale);
    gait.step_length_m *= gait_activity_scale;
    gait.stride_phase_rate_hz.value *= std::clamp(0.75 + 0.25 * gait_activity_scale, 0.55, 1.0);

    const double commanded_body_height_m = intent.twist.body_trans_m.z;
    const double measured_body_height_m =
        (est.has_body_twist_state && std::isfinite(est.body_twist_state.body_trans_m.z))
            ? est.body_twist_state.body_trans_m.z
            : commanded_body_height_m;
    const double body_height_deficit_m = std::max(0.0, commanded_body_height_m - measured_body_height_m);
    const double support_deficit_m = std::max(0.0, config_.min_margin_required_m - gait.static_stability_margin_m);
    const double swing_height_boost_m = std::clamp(body_height_deficit_m + 0.5 * support_deficit_m, 0.0, 0.06);
    const double swing_height_floor_m = gaitPresetSwingHeightFloor(intent.gait);
    gait.swing_height_m =
        std::clamp(gait.swing_height_m + swing_height_boost_m, swing_height_floor_m, 0.06);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double lift_clearance_m =
            supportPolygonClearanceExcludingLeg(support, leg, margin_need, config_.support_inset_m);
        const bool can_lift = lift_clearance_m >= 0.0 && !emergency_tilt_hold;
        const bool supported_swing_leg =
            support.effective_support[static_cast<std::size_t>(leg)] &&
            !gait.in_stance[static_cast<std::size_t>(leg)];
        gait.support_liftoff_clearance_m[static_cast<std::size_t>(leg)] = lift_clearance_m;
        gait.support_liftoff_safe_to_lift[static_cast<std::size_t>(leg)] = can_lift;
        gait.stability_hold_stance[static_cast<std::size_t>(leg)] = !can_lift || supported_swing_leg;
    }

    const bool no_leg_safe_to_lift = std::none_of(
        gait.support_liftoff_safe_to_lift.begin(),
        gait.support_liftoff_safe_to_lift.end(),
        [](const bool safe) { return safe; });
    if (high_activity_command && gait.static_stability_margin_m <= 0.0 && no_leg_safe_to_lift) {
        // Once the current support polygon is already invalid, continuing to alternate tripod
        // memberships can kick the body into a tip before the outer safety loop reacts. Treat the
        // gait as a pure recovery stance until the support margin recovers instead of preserving
        // the nominal walk phase split.
        gait.in_stance.fill(true);
        gait.stability_hold_stance.fill(true);
        gait.step_length_m = 0.0;
        gait.stride_phase_rate_hz.value = 0.0;
    }
}
