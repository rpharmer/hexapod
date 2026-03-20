#include "estimator.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<SimpleEstimator>();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr);

    if (!expect(runtime.init(), "runtime should initialize with simulator bridge")) {
        return EXIT_FAILURE;
    }

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.timestamp_us = now_us();
    runtime.setMotionIntent(walk_intent);

    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    const ControlStatus status_after_first = runtime.getStatus();
    if (!expect(status_after_first.loop_counter == 1, "loop counter should increment on first control step") ||
        !expect(status_after_first.bus_ok, "simulator bus should be healthy by default") ||
        !expect(status_after_first.active_mode == RobotMode::WALK, "active mode should follow WALK intent")) {
        return EXIT_FAILURE;
    }

    SimHardwareFaultToggles drop_bus_fault{};
    drop_bus_fault.drop_bus = true;
    if (!expect(runtime.setSimFaultToggles(drop_bus_fault), "sim fault toggles should be configurable")) {
        return EXIT_FAILURE;
    }

    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    const ControlStatus status_after_fault = runtime.getStatus();
    if (!expect(status_after_fault.loop_counter == 2, "loop counter should increment on each control step") ||
        !expect(!status_after_fault.bus_ok, "bus status should reflect injected simulator fault") ||
        !expect(status_after_fault.active_fault == FaultCode::BUS_TIMEOUT, "bus drop should surface as BUS_TIMEOUT fault")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
