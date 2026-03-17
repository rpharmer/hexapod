#include "safety_supervisor.hpp"

#include <cmath>

namespace {
constexpr double kMaxTiltRad = 0.70;
constexpr uint64_t kCommandTimeoutUs = 300000; // 300 ms

bool isIntentStale(const MotionIntent& intent) {
    if (intent.timestamp_us == 0) {
        return true;
    }

    return (now_us() - intent.timestamp_us) > kCommandTimeoutUs;
}
} // namespace

void SafetySupervisor::trip(SafetyState& s, FaultCode code, bool torque_cut) {
    s.active_fault = code;
    s.inhibit_motion = true;
    s.torque_cut = torque_cut;
}

SafetyState SafetySupervisor::evaluate(const RawHardwareState& raw,
                                       const EstimatedState& est,
                                       const MotionIntent& intent) {
    (void)raw; // currently unused in safety checks

    SafetyState s{};

    if (std::abs(est.body_twist_state.twist_pos_rad.x) > kMaxTiltRad ||
        std::abs(est.body_twist_state.twist_pos_rad.y) > kMaxTiltRad) {
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
