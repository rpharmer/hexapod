#include "safety_supervisor.hpp"

#include "control_config.hpp"

#include <cmath>

namespace {

bool isIntentStale(const MotionIntent& intent) {
    if (intent.timestamp_us.isZero()) {
        return true;
    }

    return (now_us() - intent.timestamp_us) > control_config::kCommandTimeoutUs;
}

} // namespace

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

void SafetySupervisor::trip(SafetyState& s, FaultCode code, bool torque_cut) {
    if (!shouldReplaceFault(s.active_fault, code)) {
        return;
    }

    s.active_fault = code;
    s.inhibit_motion = true;
    s.torque_cut = s.torque_cut || torque_cut;
}

SafetyState SafetySupervisor::evaluate(const RawHardwareState& raw,
                                       const EstimatedState& est,
                                       const MotionIntent& intent) {
    SafetyState s{};
    int contact_count = 0;
    for (bool foot_contact : raw.foot_contacts) {
        if (foot_contact) {
            ++contact_count;
        }
    }

    if (!raw.bus_ok) {
        trip(s, FaultCode::BUS_TIMEOUT, true);
    }

    if (raw.voltage < control_config::kMinBusVoltageV ||
        raw.current > control_config::kMaxBusCurrentA) {
        trip(s, FaultCode::MOTOR_FAULT, true);
    }

    if (contact_count < control_config::kMinFootContacts ||
        contact_count > control_config::kMaxFootContacts) {
        trip(s, FaultCode::ESTIMATOR_INVALID, false);
    }

    if (std::abs(est.body_twist_state.twist_pos_rad.x) > control_config::kMaxTiltRad.value ||
        std::abs(est.body_twist_state.twist_pos_rad.y) > control_config::kMaxTiltRad.value) {
        trip(s, FaultCode::TIP_OVER, true);
    }

    if (isIntentStale(intent)) {
        trip(s, FaultCode::COMMAND_TIMEOUT, false);
    }

    if (intent.requested_mode == RobotMode::SAFE_IDLE) {
        s.inhibit_motion = true;
    }

    return s;
}
