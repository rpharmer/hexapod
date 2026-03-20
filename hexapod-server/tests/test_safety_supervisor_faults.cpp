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

MotionIntent safeIdleIntentNow() {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::SAFE_IDLE;
    intent.timestamp_us = now_us();
    return intent;
}

} // namespace

int main() {
    SafetySupervisor supervisor;

    RawHardwareState raw{};
    raw.timestamp_us = now_us();
    raw.bus_ok = true;
    raw.voltage = 12.0f;
    raw.current = 1.0f;
    raw.foot_contacts = {true, true, true, true, true, true};

    EstimatedState est{};
    est.timestamp_us = now_us();

    MotionIntent intent = safeIdleIntentNow();

    const SafetyState nominal = supervisor.evaluate(raw, est, intent);
    if (!expect(nominal.active_fault == FaultCode::NONE, "nominal inputs should keep safety fault-free") ||
        !expect(nominal.fault_lifecycle == FaultLifecycle::ACTIVE, "nominal lifecycle should remain ACTIVE")) {
        return EXIT_FAILURE;
    }

    raw.bus_ok = false;
    const SafetyState tripped = supervisor.evaluate(raw, est, intent);
    if (!expect(tripped.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should trip BUS_TIMEOUT") ||
        !expect(tripped.fault_lifecycle == FaultLifecycle::LATCHED, "fault should latch immediately")) {
        return EXIT_FAILURE;
    }

    raw.bus_ok = true;
    intent = safeIdleIntentNow();
    const SafetyState recovering = supervisor.evaluate(raw, est, intent);
    if (!expect(recovering.fault_lifecycle == FaultLifecycle::RECOVERING, "SAFE_IDLE with cleared conditions should enter RECOVERING")) {
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    intent = safeIdleIntentNow();
    const SafetyState cleared = supervisor.evaluate(raw, est, intent);
    if (!expect(cleared.active_fault == FaultCode::NONE, "fault should clear after recovery hold time") ||
        !expect(cleared.fault_lifecycle == FaultLifecycle::ACTIVE, "lifecycle should return ACTIVE after clearing") ||
        !expect(cleared.torque_cut == false, "torque cut should reset when no active fault remains")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
