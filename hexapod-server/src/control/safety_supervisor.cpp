#include "safety_supervisor.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>

namespace {

double planarBodyRateRadps(const RobotState& est) {
    if (!est.has_imu || !est.imu.valid) {
        return 0.0;
    }
    return std::hypot(est.imu.gyro_radps.x, est.imu.gyro_radps.y);
}

double measuredPlanarSpeedMps(const RobotState& est) {
    if (!est.has_body_twist_state) {
        return 0.0;
    }
    return std::hypot(est.body_twist_state.body_trans_mps.x, est.body_twist_state.body_trans_mps.y);
}

double commandedYawRateRadps(const MotionIntent& intent) {
    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    return std::abs(cmd.yaw_rate_radps);
}

} // namespace

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
        case FaultCode::BODY_COLLAPSE: return 70;
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

SafetySupervisor::FaultDecision SafetySupervisor::evaluateFaultRules(
    const RobotState& raw,
    const RobotState& est,
    const MotionIntent& intent,
    const FreshnessInputs& freshness,
    int contact_count) const {
    const std::array<FaultRule, 8> rules{{
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
                return raw.has_power_state &&
                       (raw.voltage < config_.min_bus_voltage_v ||
                        raw.current > config_.max_bus_current_a);
            },
            .fault = FaultCode::MOTOR_FAULT,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                return std::abs(est.body_twist_state.twist_pos_rad.x) > config_.max_tilt_rad.value ||
                       std::abs(est.body_twist_state.twist_pos_rad.y) > config_.max_tilt_rad.value;
            },
            .fault = FaultCode::TIP_OVER,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                if (config_.rapid_body_rate_radps <= 0.0) {
                    return false;
                }
                if (intent.requested_mode != RobotMode::WALK) {
                    return false;
                }
                const double planar_meas_mps = measuredPlanarSpeedMps(est);
                const double yaw_cmd_radps = commandedYawRateRadps(intent);
                if (planar_meas_mps < 0.18 && yaw_cmd_radps < 0.35) {
                    return false;
                }
                if (!est.has_body_twist_state || !est.has_imu || !est.imu.valid) {
                    return false;
                }
                if (contact_count > config_.rapid_body_rate_max_contacts) {
                    return false;
                }
                const double body_rate_radps = planarBodyRateRadps(est);
                return std::isfinite(body_rate_radps) && body_rate_radps > config_.rapid_body_rate_radps;
            },
            .fault = FaultCode::TIP_OVER,
            .torque_cut = true,
        },
        FaultRule{
            .is_triggered = [&](void) {
                if (intent.requested_mode != RobotMode::WALK) {
                    return false;
                }
                if (!est.has_body_twist_state) {
                    return false;
                }
                const double measured_body_height_m = est.body_twist_state.body_trans_m.z;
                if (!std::isfinite(measured_body_height_m)) {
                    return false;
                }
                if (contact_count > config_.body_height_collapse_max_contacts) {
                    return false;
                }
                if (config_.body_height_collapse_min_safe_m > 0.0) {
                    if (measured_body_height_m <= 0.0) {
                        return false;
                    }
                    return measured_body_height_m < config_.body_height_collapse_min_safe_m;
                }
                if (config_.body_height_collapse_margin_m <= 0.0) {
                    return false;
                }
                const double commanded_body_height_m = intent.twist.body_trans_m.z;
                if (!std::isfinite(commanded_body_height_m)) {
                    return false;
                }
                if (commanded_body_height_m <= 0.0 || measured_body_height_m <= 0.0) {
                    return false;
                }
                return (commanded_body_height_m - measured_body_height_m) > config_.body_height_collapse_margin_m;
            },
            .fault = FaultCode::BODY_COLLAPSE,
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
                                                                       const FreshnessInputs& freshness) const {
    int contact_count = 0;
    for (bool foot_contact : raw.foot_contacts) {
        if (foot_contact) {
            ++contact_count;
        }
    }

    return evaluateFaultRules(raw, est, intent, freshness, contact_count);
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
    const FaultDecision fault = evaluateCurrentFault(raw, est, intent, freshness);

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
