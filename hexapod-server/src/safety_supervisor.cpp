#include "safety_supervisor.hpp"

#include <cmath>

void SafetySupervisor::trip(SafetyState& s, FaultCode code, bool torque_cut) {
    s.active_fault = code;
    s.inhibit_motion = true;
    s.torque_cut = torque_cut;
}

SafetyState SafetySupervisor::evaluate(const RawHardwareState& raw,
                                       const EstimatedState& est,
                                       const MotionIntent& intent) {
    SafetyState s{};
    s.inhibit_motion = false;
    s.torque_cut = false;
    s.active_fault = FaultCode::NONE;


    constexpr double kMaxTilt = 0.70;
    if (std::abs(est.body_twist_state.twist_pos_rad.x) > kMaxTilt ||
        std::abs(est.body_twist_state.twist_pos_rad.y) > kMaxTilt) {
        trip(s, FaultCode::TIP_OVER, true);
    }

    // Command watchdog: if control intent is stale, inhibit motion.
    constexpr uint64_t kCmdTimeoutUs = 300000; // 300 ms
    const uint64_t age = now_us() - intent.timestamp_us;
    if (intent.timestamp_us == 0 || age > kCmdTimeoutUs) {
        trip(s, FaultCode::COMMAND_TIMEOUT, false);
    }

    if (intent.requested_mode == RobotMode::SAFE_IDLE) {
        s.inhibit_motion = true;
    }

  (void)raw; // suppress unused variable warning

    return s;
}