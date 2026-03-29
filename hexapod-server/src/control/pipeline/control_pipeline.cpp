#include "control_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string_view>

namespace {

bool contactManagerBypassed() {
    const char* raw = std::getenv("HEXAPOD_BYPASS_CONTACT_MANAGER");
    if (raw == nullptr) {
        return false;
    }
    const std::string_view value{raw};
    return value == "1" || value == "true" || value == "TRUE" || value == "yes" || value == "YES";
}

} // namespace

ControlPipeline::ControlPipeline(control_config::ControlConfig config)
    : config_(config),
      planner_(config_.gait),
      motion_limiter_(config.motion_limiter),
      gait_(config_.gait),
      body_(config_.motion_limiter) {
    timing_window_size_ = std::clamp<std::size_t>(
        config_.pipeline_stage_timing.rolling_window_samples,
        1U,
        kMaxTimingWindow);
}

PipelineStepResult ControlPipeline::runStep(const RobotState& estimated,
                                            const MotionIntent& intent,
                                            const SafetyState& safety_state,
                                            const DurationSec& loop_dt,
                                            bool bus_ok,
                                            uint64_t loop_counter) {
    // Freeze/hold semantics used during operator-guided bring-up:
    //  - "All-off" (inhibit_motion or torque_cut) holds the last known-good joint targets.
    //  - On a stage re-enable edge, that stage executes once for internal reseed/reset, but
    //    downstream stages consume the previous stable cached output for that cycle.
    //  - The next cycle after reseed uses the newly produced outputs, avoiding uninitialized
    //    defaults and reducing transition transients.
    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }
    const bool all_off_mode = safety_state.inhibit_motion || safety_state.torque_cut;
    const bool gait_stage_enabled = !all_off_mode;
    const bool contact_stage_enabled = gait_stage_enabled;
    const bool body_stage_enabled = !all_off_mode;
    const bool ik_stage_enabled = !all_off_mode;

    const RuntimeGaitPolicy gait_policy = planner_.plan(estimated, intent, safety_state);
    const auto limiter_start = std::chrono::steady_clock::now();
    const MotionLimiterOutput limiter_output =
        motion_limiter_.update(estimated, intent, gait_policy, safety_state, loop_dt);
    const auto limiter_finish = std::chrono::steady_clock::now();
    recordStageDuration(PipelineStage::Limiter,
                        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  limiter_finish - limiter_start)
                                                  .count()));

    RuntimeGaitPolicy stable_managed_policy = gait_policy;
    if (has_cached_managed_policy_) {
        stable_managed_policy = cached_managed_policy_;
    }

    GaitState stable_gait_state{};
    if (has_cached_gait_state_) {
        stable_gait_state = cached_gait_state_;
    }

    const auto gait_start = std::chrono::steady_clock::now();
    const GaitState gait_state =
        gait_.update(estimated, limiter_output.limited_intent, safety_state, limiter_output.adapted_gait_policy);
    const auto gait_finish = std::chrono::steady_clock::now();
    recordStageDuration(PipelineStage::Gait,
                        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  gait_finish - gait_start)
                                                  .count()));

    if (gait_stage_enabled && gait_stage_enabled_last_cycle_) {
        stable_gait_state = gait_state;
        cached_gait_state_ = gait_state;
        has_cached_gait_state_ = true;
    } else if (gait_stage_enabled && !gait_stage_enabled_last_cycle_) {
        cached_gait_state_ = gait_state;
        has_cached_gait_state_ = true;
    }

    ContactManagerOutput contact_adjusted{};
    if (contactManagerBypassed()) {
        contact_adjusted.managed_gait = gait_state;
        contact_adjusted.managed_policy = limiter_output.adapted_gait_policy;
    } else {
        contact_adjusted = contact_manager_.update(estimated, gait_state, limiter_output.adapted_gait_policy);
    }
    if (contact_stage_enabled && contact_stage_enabled_last_cycle_) {
        stable_managed_policy = contact_adjusted.managed_policy;
        cached_managed_policy_ = contact_adjusted.managed_policy;
        has_cached_managed_policy_ = true;
    } else if (contact_stage_enabled && !contact_stage_enabled_last_cycle_) {
        cached_managed_policy_ = contact_adjusted.managed_policy;
        has_cached_managed_policy_ = true;
    }

    if (has_cached_gait_state_) {
        contact_adjusted.managed_gait = stable_gait_state;
    } else {
        contact_adjusted.managed_gait = gait_state;
    }
    if (has_cached_managed_policy_) {
        contact_adjusted.managed_policy = stable_managed_policy;
    } else {
        contact_adjusted.managed_policy = limiter_output.adapted_gait_policy;
    }

    const auto body_start = std::chrono::steady_clock::now();
    const LegTargets computed_leg_targets = body_.update(
        estimated,
        limiter_output.limited_intent,
        contact_adjusted.managed_gait,
        contact_adjusted.managed_policy,
        safety_state);
    const auto body_finish = std::chrono::steady_clock::now();
    recordStageDuration(PipelineStage::Body,
                        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  body_finish - body_start)
                                                  .count()));
    LegTargets leg_targets = computed_leg_targets;
    if (body_stage_enabled && body_stage_enabled_last_cycle_) {
        cached_leg_targets_ = computed_leg_targets;
        has_cached_leg_targets_ = true;
    } else if (body_stage_enabled && !body_stage_enabled_last_cycle_) {
        if (has_cached_leg_targets_) {
            leg_targets = cached_leg_targets_;
        }
        cached_leg_targets_ = computed_leg_targets;
        has_cached_leg_targets_ = true;
    } else if (has_cached_leg_targets_) {
        leg_targets = cached_leg_targets_;
    }

    const auto ik_start = std::chrono::steady_clock::now();
    const JointTargets computed_joint_targets = ik_.solve(estimated, leg_targets, safety_state);
    const auto ik_finish = std::chrono::steady_clock::now();
    recordStageDuration(PipelineStage::Ik,
                        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  ik_finish - ik_start)
                                                  .count()));
    JointTargets joint_targets = computed_joint_targets;
    if (ik_stage_enabled && ik_stage_enabled_last_cycle_) {
        cached_joint_targets_ = computed_joint_targets;
        has_cached_joint_targets_ = true;
    } else if (ik_stage_enabled && !ik_stage_enabled_last_cycle_) {
        if (has_cached_joint_targets_) {
            joint_targets = cached_joint_targets_;
        }
        cached_joint_targets_ = computed_joint_targets;
        has_cached_joint_targets_ = true;
    } else if (has_cached_joint_targets_) {
        joint_targets = cached_joint_targets_;
    } else {
        cached_joint_targets_ = computed_joint_targets;
        has_cached_joint_targets_ = true;
    }
    if (all_off_mode && has_cached_joint_targets_) {
        joint_targets = cached_joint_targets_;
    }
    gait_stage_enabled_last_cycle_ = gait_stage_enabled;
    contact_stage_enabled_last_cycle_ = contact_stage_enabled;
    body_stage_enabled_last_cycle_ = body_stage_enabled;
    ik_stage_enabled_last_cycle_ = ik_stage_enabled;

    const BodyController::MotionLimiterTelemetry body_limiter = body_.lastMotionLimiterTelemetry();

    ControlStatus status{};
    status.active_mode = active_mode;
    status.estimator_valid = !estimated.timestamp_us.isZero();
    status.bus_ok = bus_ok;
    status.active_fault = safety_state.active_fault;
    status.loop_counter = loop_counter;
    status.dynamic_gait.valid = contact_adjusted.managed_policy.dynamic_enabled;
    status.dynamic_gait.gait_family = contact_adjusted.managed_policy.gait_family;
    status.dynamic_gait.region = static_cast<uint8_t>(contact_adjusted.managed_policy.region);
    status.dynamic_gait.turn_mode = static_cast<uint8_t>(contact_adjusted.managed_policy.turn_mode);
    status.dynamic_gait.fallback_stage = static_cast<uint8_t>(contact_adjusted.managed_policy.fallback_stage);
    status.dynamic_gait.cadence_hz = contact_adjusted.managed_policy.cadence_hz.value;
    status.dynamic_gait.reach_utilization = contact_adjusted.managed_policy.reach_utilization;
    status.dynamic_gait.envelope_max_speed_normalized =
        contact_adjusted.managed_policy.envelope.max_speed_normalized;
    status.dynamic_gait.envelope_max_yaw_normalized =
        contact_adjusted.managed_policy.envelope.max_yaw_normalized;
    status.dynamic_gait.envelope_max_roll_pitch_rad =
        contact_adjusted.managed_policy.envelope.max_roll_pitch_rad;
    status.dynamic_gait.envelope_allow_tripod =
        contact_adjusted.managed_policy.envelope.allow_tripod;
    status.dynamic_gait.suppress_stride_progression =
        contact_adjusted.managed_policy.suppression.suppress_stride_progression;
    status.dynamic_gait.suppress_turning =
        contact_adjusted.managed_policy.suppression.suppress_turning;
    status.dynamic_gait.prioritize_stability =
        contact_adjusted.managed_policy.suppression.prioritize_stability;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        status.dynamic_gait.leg_phase[leg] = contact_adjusted.managed_gait.phase[leg];
        status.dynamic_gait.leg_duty_cycle[leg] = contact_adjusted.managed_policy.per_leg[leg].duty_cycle;
        status.dynamic_gait.leg_in_stance[leg] = contact_adjusted.managed_gait.in_stance[leg];
    }
    status.dynamic_gait.limiter_enabled = limiter_output.diagnostics.enabled;
    status.dynamic_gait.limiter_phase = limiter_output.diagnostics.phase;
    status.dynamic_gait.active_constraint_reason = limiter_output.diagnostics.constraint_reason;
    status.dynamic_gait.adaptation_scale_linear = limiter_output.diagnostics.adaptation_scale_linear;
    status.dynamic_gait.adaptation_scale_yaw = limiter_output.diagnostics.adaptation_scale_yaw;
    status.dynamic_gait.adaptation_scale_cadence = contact_adjusted.managed_policy.adaptation_scale_cadence;
    status.dynamic_gait.adaptation_scale_step = contact_adjusted.managed_policy.adaptation_scale_step;
    status.dynamic_gait.hard_clamp_linear = limiter_output.diagnostics.hard_clamp_linear;
    status.dynamic_gait.hard_clamp_yaw = limiter_output.diagnostics.hard_clamp_yaw;
    status.dynamic_gait.hard_clamp_reach =
        limiter_output.diagnostics.hard_clamp_reach || body_limiter.hard_clamp_reach;
    status.dynamic_gait.hard_clamp_cadence = contact_adjusted.managed_policy.hard_clamp_cadence;
    status.dynamic_gait.saturated =
        status.dynamic_gait.hard_clamp_linear ||
        status.dynamic_gait.hard_clamp_yaw ||
        status.dynamic_gait.hard_clamp_reach ||
        status.dynamic_gait.hard_clamp_cadence;

    PipelineStepResult result{};
    result.leg_targets = leg_targets;
    result.joint_targets = joint_targets;
    result.status = status;
    return result;
}

void ControlPipeline::recordStageDuration(PipelineStage stage, uint64_t duration_us) {
    const std::size_t stage_index = static_cast<std::size_t>(stage);
    if (stage_index >= stage_timings_.size()) {
        return;
    }

    std::scoped_lock<std::mutex> lock(timing_mutex_);
    StageTimingWindow& window = stage_timings_[stage_index];
    window.samples_us[window.write_index] = duration_us;
    window.write_index = (window.write_index + 1) % timing_window_size_;
    if (window.sample_count < timing_window_size_) {
        ++window.sample_count;
    }
}

PipelineTimingSnapshot ControlPipeline::timingSnapshot() const {
    std::scoped_lock<std::mutex> lock(timing_mutex_);
    return PipelineTimingSnapshot{
        .estimator = stageEnvelopeLocked(PipelineStage::Estimator),
        .limiter = stageEnvelopeLocked(PipelineStage::Limiter),
        .gait = stageEnvelopeLocked(PipelineStage::Gait),
        .body = stageEnvelopeLocked(PipelineStage::Body),
        .ik = stageEnvelopeLocked(PipelineStage::Ik),
    };
}

uint64_t ControlPipeline::stageBudgetP95Us(PipelineStage stage) const {
    switch (stage) {
        case PipelineStage::Estimator:
            return config_.pipeline_stage_timing.estimator.p95_max_us;
        case PipelineStage::Limiter:
            return config_.pipeline_stage_timing.limiter.p95_max_us;
        case PipelineStage::Gait:
            return config_.pipeline_stage_timing.gait.p95_max_us;
        case PipelineStage::Body:
            return config_.pipeline_stage_timing.body.p95_max_us;
        case PipelineStage::Ik:
            return config_.pipeline_stage_timing.ik.p95_max_us;
        case PipelineStage::Count:
            break;
    }
    return 0;
}

uint64_t ControlPipeline::stageBudgetP99Us(PipelineStage stage) const {
    switch (stage) {
        case PipelineStage::Estimator:
            return config_.pipeline_stage_timing.estimator.p99_max_us;
        case PipelineStage::Limiter:
            return config_.pipeline_stage_timing.limiter.p99_max_us;
        case PipelineStage::Gait:
            return config_.pipeline_stage_timing.gait.p99_max_us;
        case PipelineStage::Body:
            return config_.pipeline_stage_timing.body.p99_max_us;
        case PipelineStage::Ik:
            return config_.pipeline_stage_timing.ik.p99_max_us;
        case PipelineStage::Count:
            break;
    }
    return 0;
}

StageTimingEnvelope ControlPipeline::stageEnvelopeLocked(PipelineStage stage) const {
    const StageTimingWindow& window = stage_timings_[static_cast<std::size_t>(stage)];
    const uint64_t p95_us = percentileLocked(window, 0.95);
    const uint64_t p99_us = percentileLocked(window, 0.99);
    const uint64_t budget_p95_us = stageBudgetP95Us(stage);
    const uint64_t budget_p99_us = stageBudgetP99Us(stage);
    return StageTimingEnvelope{
        .sample_count = static_cast<uint64_t>(window.sample_count),
        .p95_us = p95_us,
        .p99_us = p99_us,
        .budget_p95_us = budget_p95_us,
        .budget_p99_us = budget_p99_us,
        .p95_within_budget = (budget_p95_us == 0) || (p95_us <= budget_p95_us),
        .p99_within_budget = (budget_p99_us == 0) || (p99_us <= budget_p99_us),
    };
}

uint64_t ControlPipeline::percentileLocked(const StageTimingWindow& window, double quantile) const {
    if (window.sample_count == 0) {
        return 0;
    }

    std::vector<uint64_t> ordered(window.sample_count);
    for (std::size_t idx = 0; idx < window.sample_count; ++idx) {
        const std::size_t ring_idx =
            (window.write_index + timing_window_size_ - window.sample_count + idx) % timing_window_size_;
        ordered[idx] = window.samples_us[ring_idx];
    }
    std::sort(ordered.begin(), ordered.end());
    const double scaled_rank = std::ceil(quantile * static_cast<double>(ordered.size()));
    const std::size_t rank = std::clamp<std::size_t>(
        scaled_rank > 1.0 ? static_cast<std::size_t>(scaled_rank) : 1U,
        1U,
        ordered.size());
    return ordered[rank - 1U];
}
