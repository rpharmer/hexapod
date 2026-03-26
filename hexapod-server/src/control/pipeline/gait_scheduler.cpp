#include "gait_scheduler.hpp"

#include "control_config.hpp"
#include "stability_tracker.hpp"

#include <algorithm>
#include <array>

namespace {

constexpr double kCadenceSlewUpHzPerSec = 150.0;
constexpr double kCadenceSlewDownHzPerSec = 200.0;

}

GaitScheduler::GaitScheduler(control_config::GaitConfig config)
    : config_(config) {}

FrequencyHz GaitScheduler::applyCadenceSlew(const FrequencyHz& target_rate_hz, const DurationSec& dt) {
    const double up_limit = kCadenceSlewUpHzPerSec * std::max(dt.value, 0.0);
    const double down_limit = kCadenceSlewDownHzPerSec * std::max(dt.value, 0.0);
    const double delta = target_rate_hz.value - cadence_hz_.value;
    const double clamped_delta = std::clamp(delta, -down_limit, up_limit);
    cadence_hz_ = FrequencyHz{cadence_hz_.value + clamped_delta};
    return cadence_hz_;
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
            out.phase[i] = wrap01(policy.per_leg[i].phase_offset);
            out.in_stance[i] = true;
        }
        out.stride_phase_rate_hz = FrequencyHz{0.0};
        cadence_hz_ = FrequencyHz{0.0};
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    FrequencyHz commanded_cadence = policy.cadence_hz;
    if (policy.suppression.suppress_stride_progression) {
        commanded_cadence = FrequencyHz{0.0};
    }
    const FrequencyHz step_rate_hz = applyCadenceSlew(commanded_cadence, dt);
    out.stride_phase_rate_hz = step_rate_hz;

    phase_accum_ = wrap01(phase_accum_ + dt.value * step_rate_hz.value);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double p = wrap01(phase_accum_ + policy.per_leg[leg].phase_offset);
        out.phase[leg] = p;
        out.in_stance[leg] = (p < std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95));
    }

    return out;
}
