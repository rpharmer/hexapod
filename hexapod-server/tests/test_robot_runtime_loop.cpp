#include "estimator.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

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
    control_config::ControlConfig strict_freshness{};
    strict_freshness.freshness.estimator.max_allowed_age_us = DurationUs{20000};
    strict_freshness.freshness.intent.max_allowed_age_us = DurationUs{20000};

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<SimpleEstimator>();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, strict_freshness);

    if (!expect(runtime.init(), "runtime should initialize with simulator bridge")) {
        return EXIT_FAILURE;
    }

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.timestamp_us = now_us();
    walk_intent.sample_id = 101;
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

    SimHardwareFaultToggles clear_faults{};
    if (!expect(runtime.setSimFaultToggles(clear_faults), "sim fault toggles should clear")) {
        return EXIT_FAILURE;
    }

    MotionIntent stale_intent = walk_intent;
    stale_intent.sample_id = 102;
    stale_intent.timestamp_us = TimePointUs{
        now_us().value - (strict_freshness.freshness.intent.max_allowed_age_us.value + 1000)};
    runtime.setMotionIntent(stale_intent);

    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
    const ControlStatus status_after_stale_intent = runtime.getStatus();
    if (!expect(status_after_stale_intent.active_mode == RobotMode::SAFE_IDLE,
                "stale intent should gate control output to SAFE_IDLE") ||
        !expect(status_after_stale_intent.active_fault == FaultCode::COMMAND_TIMEOUT,
                "stale intent should report COMMAND_TIMEOUT fault")) {
        return EXIT_FAILURE;
    }

    MotionIntent fresh_walk = walk_intent;
    fresh_walk.sample_id = 103;
    fresh_walk.timestamp_us = now_us();
    runtime.setMotionIntent(fresh_walk);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    runtime.controlStep();
    const ControlStatus status_after_stale_estimator = runtime.getStatus();
    if (!expect(status_after_stale_estimator.active_mode == RobotMode::SAFE_IDLE,
                "stale estimator should gate control output to SAFE_IDLE") ||
        !expect(status_after_stale_estimator.active_fault == FaultCode::ESTIMATOR_INVALID,
                "stale estimator should report ESTIMATOR_INVALID fault")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
