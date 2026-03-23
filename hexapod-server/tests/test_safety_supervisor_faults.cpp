#include "safety_supervisor.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RawHardwareState nominalRaw() {
    RawHardwareState raw{};
    raw.timestamp_us = now_us();
    raw.bus_ok = true;
    raw.voltage = 12.0f;
    raw.current = 1.0f;
    raw.foot_contacts = {true, true, true, true, true, true};
    return raw;
}

EstimatedState nominalEstimated() {
    EstimatedState est{};
    est.timestamp_us = now_us();
    return est;
}

MotionIntent intentNow(RobotMode mode) {
    MotionIntent intent{};
    intent.requested_mode = mode;
    intent.timestamp_us = now_us();
    return intent;
}

MotionIntent staleIntent(RobotMode mode) {
    MotionIntent intent{};
    intent.requested_mode = mode;
    intent.timestamp_us = TimePointUs{};
    return intent;
}

bool testHigherPriorityFaultWins() {
    SafetySupervisor supervisor;
    RawHardwareState raw = nominalRaw();
    EstimatedState est = nominalEstimated();

    raw.bus_ok = false;
    est.body_twist_state.twist_pos_rad.x = 2.0; // clearly above default tilt limit

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "simultaneous bus + tip-over should keep higher-priority TIP_OVER") &&
           expect(state.torque_cut, "tip-over should request torque cut");
}

bool testLatchedRemainsWhenIntentNotSafeIdle() {
    SafetySupervisor supervisor;
    RawHardwareState raw = nominalRaw();
    EstimatedState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch BUS_TIMEOUT")) {
        return false;
    }

    raw.bus_ok = true;
    const SafetyState held = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(held.active_fault == FaultCode::BUS_TIMEOUT,
                  "fault should remain BUS_TIMEOUT after hardware clears") &&
           expect(held.fault_lifecycle == FaultLifecycle::LATCHED,
                  "lifecycle should stay LATCHED when intent mode is not SAFE_IDLE");
}

bool testLatchedRemainsWhenIntentStale() {
    SafetySupervisor supervisor;
    RawHardwareState raw = nominalRaw();
    EstimatedState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch before stale-intent check")) {
        return false;
    }

    raw.bus_ok = true;
    const SafetyState held = supervisor.evaluate(
        raw, est, staleIntent(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, false});
    return expect(held.active_fault == FaultCode::BUS_TIMEOUT,
                  "fault should remain BUS_TIMEOUT with stale SAFE_IDLE intent") &&
           expect(held.fault_lifecycle == FaultLifecycle::LATCHED,
                  "lifecycle should stay LATCHED when intent timestamp is stale");
}

bool testRecoveryRequiresBothConditionsAndHoldTime() {
    SafetySupervisor supervisor;
    RawHardwareState raw = nominalRaw();
    EstimatedState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch for recovery test")) {
        return false;
    }

    raw.bus_ok = true;

    const SafetyState not_safe_idle = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(not_safe_idle.fault_lifecycle == FaultLifecycle::LATCHED,
                "recovery must not start until mode is SAFE_IDLE")) {
        return false;
    }

    const SafetyState stale_safe_idle = supervisor.evaluate(
        raw, est, staleIntent(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, false});
    if (!expect(stale_safe_idle.fault_lifecycle == FaultLifecycle::LATCHED,
                "recovery must not start with stale SAFE_IDLE intent")) {
        return false;
    }

    const SafetyState recovering = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(recovering.fault_lifecycle == FaultLifecycle::RECOVERING,
                "recovery should start only with SAFE_IDLE + fresh intent")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    const SafetyState cleared = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    return expect(cleared.active_fault == FaultCode::NONE,
                  "fault should clear after recovery hold time") &&
           expect(cleared.fault_lifecycle == FaultLifecycle::ACTIVE,
                  "lifecycle should return ACTIVE after clear");
}

} // namespace

int main() {
    if (!testHigherPriorityFaultWins() ||
        !testLatchedRemainsWhenIntentNotSafeIdle() ||
        !testLatchedRemainsWhenIntentStale() ||
        !testRecoveryRequiresBothConditionsAndHoldTime()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
