#include "gait_scheduler.hpp"

#include "gait_params.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

GaitScheduler::GaitScheduler(control_config::GaitConfig config)
    : config_(config) {}

double GaitScheduler::wrap01(const double x) const {
    return gaitWrap01(x);
}

void GaitScheduler::reset() {
    phase_accum_ = 0.0;
    last_update_us_ = TimePointUs{};
    committed_gait_ = GaitType::TRIPOD;
    committed_initialized_ = false;
    transition_from_snap_ = UnifiedGaitDescription{};
    last_blended_ = UnifiedGaitDescription{};
    transition_start_us_ = TimePointUs{};
    have_last_blended_ = false;
    last_cmd_vx_mps_ = 0.0;
    last_cmd_vy_mps_ = 0.0;
}

GaitState GaitScheduler::update(const RobotState&,
                                const MotionIntent& intent,
                                const SafetyState& safety,
                                const BodyTwist& cmd_twist,
                                const CommandGovernorState& governor) {
    GaitState out{};
    out.timestamp_us = now_us();

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    if (!walking) {
        last_cmd_vx_mps_ = cmd_twist.linear_mps.x;
        last_cmd_vy_mps_ = cmd_twist.linear_mps.y;
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
            out.phase_offset[static_cast<std::size_t>(i)] = 0.0;
            out.stability_hold_stance[static_cast<std::size_t>(i)] = false;
            out.support_liftoff_clearance_m[static_cast<std::size_t>(i)] = 0.0;
            out.support_liftoff_safe_to_lift[static_cast<std::size_t>(i)] = false;
        }
        out.duty_factor = 0.5;
        out.step_length_m = 0.06;
        out.swing_height_m = 0.03;
        out.stance_duration_s = 0.5;
        out.swing_duration_s = 0.5;
        out.stride_phase_rate_hz = FrequencyHz{1.0};
        out.swing_time_ease_01 = 1.0;
        out.static_stability_margin_m = 0.0;
        out.cmd_accel_body_x_mps2 = 0.0;
        out.cmd_accel_body_y_mps2 = 0.0;
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    const PlanarMotionCommand cmd = planarMotionFromCommandTwist(cmd_twist);
    double cmd_ax = 0.0;
    double cmd_ay = 0.0;
    if (dt.value > 1e-9) {
        const double inv_dt = 1.0 / dt.value;
        cmd_ax = std::clamp((cmd.vx_mps - last_cmd_vx_mps_) * inv_dt, -8.0, 8.0);
        cmd_ay = std::clamp((cmd.vy_mps - last_cmd_vy_mps_) * inv_dt, -8.0, 8.0);
    }
    last_cmd_vx_mps_ = cmd.vx_mps;
    last_cmd_vy_mps_ = cmd.vy_mps;

    UnifiedGaitDescription target{};
    if (intent.gait == GaitType::TRIPOD) {
        target = buildAdaptiveTripodCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else if (intent.gait == GaitType::RIPPLE) {
        target = buildAdaptiveRippleCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else if (intent.gait == GaitType::WAVE) {
        target = buildAdaptiveWaveCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else {
        target = buildTargetUnifiedGait(
            intent.gait, cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, config_, cmd_ax, cmd_ay);
    }

    if (committed_initialized_ && intent.gait != committed_gait_) {
        transition_from_snap_ = have_last_blended_ ? last_blended_ : target;
        transition_start_us_ = now;
    }
    if (!committed_initialized_) {
        transition_from_snap_ = target;
    }
    committed_gait_ = intent.gait;
    committed_initialized_ = true;

    double alpha = 1.0;
    if (!transition_start_us_.isZero()) {
        const double elapsed_s = static_cast<double>((now - transition_start_us_).value) * 1e-6;
        alpha = std::clamp(elapsed_s / std::max(config_.transition_blend_s, 1e-4), 0.0, 1.0);
    }

    const UnifiedGaitDescription blended =
        (alpha >= 1.0 - 1e-9) ? target : blendUnifiedGait(transition_from_snap_, target, alpha);
    if (alpha >= 1.0 - 1e-9) {
        transition_start_us_ = {};
    }

    last_blended_ = blended;
    have_last_blended_ = true;

    const double governor_cadence_scale = std::clamp(governor.cadence_scale, 0.25, 1.0);
    const double governor_swing_floor_m = std::max(0.0, governor.swing_height_floor_m);
    const double step_hz = std::max(blended.step_frequency_hz * governor_cadence_scale, 1e-6);
    out.stride_phase_rate_hz = FrequencyHz{step_hz};
    out.duty_factor = blended.duty_factor;
    out.step_length_m = blended.step_length_m;
    out.swing_height_m = std::max(blended.swing_height_m, governor_swing_floor_m);
    out.swing_time_ease_01 = blended.swing_time_ease;
    out.stance_duration_s = blended.stance_duration_s;
    out.swing_duration_s = blended.swing_duration_s;
    out.phase_offset = blended.phase_offset;

    phase_accum_ = wrap01(phase_accum_ + dt.value * step_hz);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double off = blended.phase_offset[static_cast<std::size_t>(leg)];
        const double p = wrap01(phase_accum_ + off);
        out.phase[leg] = p;
        out.in_stance[leg] = (p < blended.duty_factor);
        out.stability_hold_stance[static_cast<std::size_t>(leg)] = false;
        out.support_liftoff_clearance_m[static_cast<std::size_t>(leg)] = 0.0;
        out.support_liftoff_safe_to_lift[static_cast<std::size_t>(leg)] = false;
    }
    out.static_stability_margin_m = 0.0;
    out.cmd_accel_body_x_mps2 = cmd_ax;
    out.cmd_accel_body_y_mps2 = cmd_ay;

    return out;
}
