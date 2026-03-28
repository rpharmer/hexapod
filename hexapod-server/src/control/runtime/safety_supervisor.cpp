#include "safety_supervisor.hpp"

#include <cmath>

SafetySupervisor::SafetySupervisor(control_config::SafetyConfig config)
    : config_(config) {}

SafetyState SafetySupervisor::evaluate(const RobotState& raw,
                                       const RobotState& est,
                                       const MotionIntent& intent) {
    return evaluate(raw, est, intent, FreshnessInputs{true, true});
}

int SafetySupervisor::faultPriority(FaultCode code) {
    switch (code) {
        case FaultCode::NONE: return 0;
        case FaultCode::COMMAND_TIMEOUT: return 10;
        case FaultCode::ESTIMATOR_INVALID: return 20;
        case FaultCode::BUS_TIMEOUT: return 80;
        case FaultCode::MOTOR_FAULT: return 90;
        case FaultCode::TIP_OVER: return 100;
        default: return 50;
    }
}

bool SafetySupervisor::shouldReplaceFault(FaultCode current, FaultCode candidate) {
    return faultPriority(candidate) > faultPriority(current);
}

std::size_t SafetySupervisor::faultIndex(FaultCode code) {
    return static_cast<std::size_t>(code);
}

bool SafetySupervisor::canAttemptClear(const MotionIntent& intent,
                                       const FreshnessInputs& freshness) const {
    return intent.requested_mode == RobotMode::SAFE_IDLE && freshness.intent_valid;
}

bool SafetySupervisor::hasLargeJointPositionDiscontinuity(const RobotState& est) const {
    if (!has_previous_estimate_) {
        previous_estimate_ = est;
        has_previous_estimate_ = true;
        return false;
    }
    if (previous_estimate_.sample_id == 0 ||
        est.sample_id == 0 ||
        est.sample_id <= previous_estimate_.sample_id ||
        (est.sample_id - previous_estimate_.sample_id) > 1) {
        previous_estimate_ = est;
        return false;
    }

    bool has_discontinuity = false;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double previous = previous_estimate_.leg_states[leg].joint_state[joint].pos_rad.value;
            const double current = est.leg_states[leg].joint_state[joint].pos_rad.value;
            if (std::abs(current - previous) > config_.max_joint_position_step_rad.value) {
                has_discontinuity = true;
                break;
            }
        }
        if (has_discontinuity) {
            break;
        }
    }

    previous_estimate_ = est;
    return has_discontinuity;
}

SafetySupervisor::FaultDecision SafetySupervisor::evaluateFaultRules(
    const RobotState& raw,
    const RobotState& est,
    const MotionIntent& intent,
    const FreshnessInputs& freshness,
    const StabilityAssessment& stability,
    int contact_count) const {
    const std::array<FaultRule, 7> rules{{
        FaultRule{
            .is_triggered = [&](void) {
                return !freshness.intent_valid;
            },
            .fault = FaultCode::COMMAND_TIMEOUT,
            .torque_cut = false,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return contact_count < config_.min_foot_contacts ||
                       contact_count > config_.max_foot_contacts;
            },
            .fault = FaultCode::ESTIMATOR_INVALID,
            .torque_cut = false,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return !freshness.estimator_valid;
            },
            .fault = FaultCode::ESTIMATOR_INVALID,
            .torque_cut = false,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return !raw.bus_ok;
            },
            .fault = FaultCode::BUS_TIMEOUT,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return raw.voltage < config_.min_bus_voltage_v ||
                       raw.current > config_.max_bus_current_a;
            },
            .fault = FaultCode::MOTOR_FAULT,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return freshness.estimator_valid &&
                       freshness.intent_valid &&
                       intent.requested_mode == RobotMode::WALK &&
                       hasLargeJointPositionDiscontinuity(est);
            },
            .fault = FaultCode::JOINT_LIMIT,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                const bool hard_fault_context =
                    !raw.bus_ok || raw.voltage < config_.min_bus_voltage_v || raw.current > config_.max_bus_current_a;
                return ((est.has_measured_body_pose_state || hard_fault_context) &&
                        (std::abs(est.body_pose_state.orientation_rad.x) > config_.max_tilt_rad.value ||
                         std::abs(est.body_pose_state.orientation_rad.y) > config_.max_tilt_rad.value)) ||
                       (freshness.estimator_valid && !stability.com_inside_support_polygon);
            },
            .fault = FaultCode::TIP_OVER,
            .torque_cut = true,
        },
    }};

    FaultDecision decision{};
    for (const FaultRule& rule : rules) {
        if (!rule.is_triggered()) {
            continue;
        }
        if (shouldReplaceFault(decision.code, rule.fault)) {
            decision.code = rule.fault;
            decision.torque_cut = rule.torque_cut;
        }
    }

    return decision;
}

SafetySupervisor::FaultDecision SafetySupervisor::evaluateCurrentFault(const RobotState& raw,
                                                                       const RobotState& est,
                                                                       const MotionIntent& intent,
                                                                       const FreshnessInputs& freshness,
                                                                       StabilityAssessment& stability) const {
    RobotState stability_state = est;
    stability_state.foot_contacts = raw.foot_contacts;
    stability = assessStability(stability_state);

    const int contact_count = stability.support_contact_count;
    return evaluateFaultRules(raw, est, intent, freshness, stability, contact_count);
}

void SafetySupervisor::trip(FaultCode code, bool torque_cut, TimePointUs timestamp_us) {
    if (!shouldReplaceFault(state_.active_fault, code)) {
        return;
    }

    state_.active_fault = code;
    state_.fault_lifecycle = FaultLifecycle::LATCHED;
    state_.inhibit_motion = true;
    state_.torque_cut = state_.torque_cut || torque_cut;

    const std::size_t idx = faultIndex(code);
    if (idx < trip_counts_.size()) {
        ++trip_counts_[idx];
        last_trip_timestamps_[idx] = timestamp_us;
        state_.active_fault_trip_count = trip_counts_[idx];
        state_.active_fault_last_trip_us = timestamp_us;
    }

    recovery_started_at_us_ = TimePointUs{};
}

void SafetySupervisor::clearActiveFault() {
    state_.active_fault = FaultCode::NONE;
    state_.fault_lifecycle = FaultLifecycle::ACTIVE;
    state_.active_fault_trip_count = 0;
    state_.active_fault_last_trip_us = TimePointUs{};
    state_.torque_cut = false;
    recovery_started_at_us_ = TimePointUs{};
}

SafetyState SafetySupervisor::evaluate(const RobotState& raw,
                                       const RobotState& est,
                                       const MotionIntent& intent,
                                       FreshnessInputs freshness) {
    const TimePointUs now = now_us();
    StabilityAssessment stability{};
    FaultDecision fault = evaluateCurrentFault(raw, est, intent, freshness, stability);

    const bool hard_fault_context =
        !raw.bus_ok || raw.voltage < config_.min_bus_voltage_v || raw.current > config_.max_bus_current_a;
    const bool tip_over_from_tilt =
        est.has_measured_body_pose_state &&
        (std::abs(est.body_pose_state.orientation_rad.x) > config_.max_tilt_rad.value ||
         std::abs(est.body_pose_state.orientation_rad.y) > config_.max_tilt_rad.value);
    const bool tip_over_from_support = freshness.estimator_valid && !stability.com_inside_support_polygon;

    if (fault.code == FaultCode::TIP_OVER &&
        tip_over_from_tilt &&
        !tip_over_from_support &&
        !hard_fault_context) {
        ++tip_over_candidate_samples_;
        if (tip_over_candidate_samples_ < kTipOverConfirmSamples) {
            fault.code = FaultCode::NONE;
            fault.torque_cut = false;
        }
    } else {
        tip_over_candidate_samples_ = 0;
    }

    state_.stable = stability.com_inside_support_polygon;
    state_.support_contact_count = stability.support_contact_count;
    state_.stability_margin_m = stability.stability_margin_m;

    if (fault.code != FaultCode::NONE) {
        trip(fault.code, fault.torque_cut, now);
    } else if (state_.active_fault != FaultCode::NONE) {
        if (!canAttemptClear(intent, freshness)) {
            state_.fault_lifecycle = FaultLifecycle::LATCHED;
            recovery_started_at_us_ = TimePointUs{};
        } else if (state_.fault_lifecycle != FaultLifecycle::RECOVERING ||
                   recovery_started_at_us_.isZero()) {
            state_.fault_lifecycle = FaultLifecycle::RECOVERING;
            recovery_started_at_us_ = now;
        } else if ((now - recovery_started_at_us_) > kRecoveryHoldTimeUs) {
            clearActiveFault();
        }
    } else {
        state_.fault_lifecycle = FaultLifecycle::ACTIVE;
    }

    state_.inhibit_motion =
        (state_.active_fault != FaultCode::NONE) || (intent.requested_mode == RobotMode::SAFE_IDLE);

    if (state_.active_fault == FaultCode::NONE) {
        state_.torque_cut = false;
    }

    return state_;
}
