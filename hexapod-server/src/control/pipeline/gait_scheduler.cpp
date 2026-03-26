#include "gait_scheduler.hpp"

#include "control_config.hpp"
#include "stability_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>

GaitScheduler::GaitScheduler(control_config::GaitConfig config)
    : config_(config) {}

double GaitScheduler::wrap01(double x) const {
    while (x >= 1.0) x -= 1.0;
    while (x < 0.0) x += 1.0;
    return x;
}

GaitState GaitScheduler::update(const RobotState& est,
                                const MotionIntent& intent,
                                const SafetyState& safety) {
    GaitPolicyPlanner planner{config_};
    const RuntimeGaitPolicy policy = planner.plan(est, intent, safety);
    return update(est, intent, safety, policy);
}

GaitState GaitScheduler::update(const RobotState& est,
                                const MotionIntent& intent,
                                const SafetyState& safety,
                                const RuntimeGaitPolicy& policy) {
    GaitState out{};
    out.timestamp_us = now_us();

    const StabilityAssessment stability = assessStability(est);
    out.stable = stability.com_inside_support_polygon;
    out.support_contact_count = stability.support_contact_count;
    out.stability_margin_m = stability.stability_margin_m;

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut &&
        out.stable &&
        !policy.suppression.suppress_stride_progression;

    if (!walking) {
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
        }
        out.stride_phase_rate_hz = FrequencyHz{0.0};
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    const double commanded_speed = std::abs(intent.speed_mps.value);
    const double normalized_command =
        std::clamp(commanded_speed / config_.frequency.nominal_max_speed_mps.value, 0.0, 1.0);
    const double speed_mag = std::max(normalized_command, config_.fallback_speed_mag.value);
    const double envelope_speed_scale = std::clamp(
        (1.0 - policy.reach_utilization) / config_.frequency.reach_envelope_soft_limit,
        config_.frequency.reach_envelope_min_scale,
        1.0);

    const double step_hz = std::clamp(
        config_.frequency.min_hz.value +
            (config_.frequency.max_hz.value - config_.frequency.min_hz.value) * speed_mag * envelope_speed_scale,
        config_.frequency.min_hz.value,
        config_.frequency.max_hz.value);
    const FrequencyHz step_rate_hz{step_hz};
    out.stride_phase_rate_hz = step_rate_hz;

    phase_accum_ = wrap01(phase_accum_ + dt.value * step_rate_hz.value);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double p = wrap01(phase_accum_ + policy.per_leg[leg].phase_offset);
        out.phase[leg] = p;
        out.in_stance[leg] = (p < std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95));
    }

    return out;
}
