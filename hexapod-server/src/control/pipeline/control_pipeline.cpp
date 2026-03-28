#include "control_pipeline.hpp"

#include <cstdlib>
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
    : planner_(config.gait),
      motion_limiter_(config.motion_limiter),
      gait_(config.gait),
      body_(config.motion_limiter) {}

PipelineStepResult ControlPipeline::runStep(const RobotState& estimated,
                                            const MotionIntent& intent,
                                            const SafetyState& safety_state,
                                            const DurationSec& loop_dt,
                                            bool bus_ok,
                                            uint64_t loop_counter) {
    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }

    const RuntimeGaitPolicy gait_policy = planner_.plan(estimated, intent, safety_state);
    const MotionLimiterOutput limiter_output =
        motion_limiter_.update(estimated, intent, gait_policy, safety_state, loop_dt);
    const GaitState gait_state =
        gait_.update(estimated, limiter_output.limited_intent, safety_state, limiter_output.adapted_gait_policy);
    ContactManagerOutput contact_adjusted{};
    if (contactManagerBypassed()) {
        contact_adjusted.managed_gait = gait_state;
        contact_adjusted.managed_policy = limiter_output.adapted_gait_policy;
    } else {
        contact_adjusted = contact_manager_.update(estimated, gait_state, limiter_output.adapted_gait_policy);
    }
    const LegTargets leg_targets = body_.update(estimated, limiter_output.limited_intent, contact_adjusted.managed_gait, contact_adjusted.managed_policy, safety_state);
    const JointTargets joint_targets = ik_.solve(estimated, leg_targets, safety_state);
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
