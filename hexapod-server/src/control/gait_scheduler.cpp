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

GaitState GaitScheduler::update(const RobotState&,
                                const MotionIntent& intent,
                                const SafetyState& safety) {
    GaitState out{};
    out.timestamp_us = now_us();

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    if (!walking) {
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
            out.phase_offset[static_cast<std::size_t>(i)] = 0.0;
        }
        out.duty_factor = 0.5;
        out.step_length_m = 0.06;
        out.swing_height_m = 0.03;
        out.stance_duration_s = 0.5;
        out.swing_duration_s = 0.5;
        out.stride_phase_rate_hz = FrequencyHz{1.0};
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const UnifiedGaitDescription target =
        buildTargetUnifiedGait(intent.gait, cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, config_);

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

    const double step_hz = std::max(blended.step_frequency_hz, 1e-6);
    out.stride_phase_rate_hz = FrequencyHz{step_hz};
    out.duty_factor = blended.duty_factor;
    out.step_length_m = blended.step_length_m;
    out.swing_height_m = blended.swing_height_m;
    out.stance_duration_s = blended.stance_duration_s;
    out.swing_duration_s = blended.swing_duration_s;
    out.phase_offset = blended.phase_offset;

    phase_accum_ = wrap01(phase_accum_ + dt.value * step_hz);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double off = blended.phase_offset[static_cast<std::size_t>(leg)];
        const double p = wrap01(phase_accum_ + off);
        out.phase[leg] = p;
        out.in_stance[leg] = (p < blended.duty_factor);
    }

    return out;
}
