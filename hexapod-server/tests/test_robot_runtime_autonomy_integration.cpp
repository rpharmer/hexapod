#include "robot_runtime.hpp"

#include "estimator.hpp"
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

class PassThroughEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override {
        return raw;
    }
};

void runRuntimeStep(RobotRuntime& runtime) {
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

MotionIntent makeFreshWalkIntent() {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.timestamp_us = now_us();
    return intent;
}

autonomy::WaypointMission makeMission() {
    autonomy::WaypointMission mission{};
    mission.mission_id = "runtime-autonomy";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 0.2, .yaw_rad = 0.1},
    };
    return mission;
}

bool testRuntimeRoutesAutonomyOutputIntoControlPath() {
    control_config::ControlConfig cfg{};
    cfg.autonomy.enabled = true;
    cfg.autonomy.no_progress_timeout_ms = 100;
    cfg.autonomy.recovery_retry_budget = 1;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassThroughEstimator>();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

    if (!expect(runtime.init(), "runtime init should succeed with autonomy enabled")) {
        return false;
    }
    if (!expect(runtime.loadAutonomyMissionForTest(makeMission()), "mission load should succeed")) {
        return false;
    }
    if (!expect(runtime.startAutonomyMissionForTest(), "mission start should succeed")) {
        return false;
    }

    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto first_output = runtime.lastAutonomyStepOutputForTest();
    if (!expect(first_output.has_value(), "autonomy output should be captured each control step")) {
        return false;
    }
    if (!expect(first_output->motion_decision.allow_motion, "nominal motion decision should allow motion")) {
        return false;
    }
    if (!expect(first_output->locomotion_command.sent, "nominal locomotion command should be dispatched")) {
        return false;
    }
    if (!expect(runtime.getStatus().active_mode == RobotMode::WALK,
                "runtime control status should reflect autonomy WALK decision")) {
        return false;
    }

    runtime.setAutonomyBlockedForTest(true);
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto blocked_output = runtime.lastAutonomyStepOutputForTest();
    if (!expect(blocked_output.has_value(), "autonomy output should exist in blocked step")) {
        return false;
    }
    if (!expect(!blocked_output->motion_decision.allow_motion,
                "blocked autonomy decision should suppress motion")) {
        return false;
    }
    if (!expect(runtime.getStatus().active_mode == RobotMode::SAFE_IDLE,
                "blocked autonomy decision should map to SAFE_IDLE control mode")) {
        return false;
    }

    runtime.setAutonomyBlockedForTest(false);
    runtime.signalAutonomyWaypointReachedForTest();
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto reached_output = runtime.lastAutonomyStepOutputForTest();
    if (!expect(reached_output.has_value(), "autonomy output should exist for waypoint event")) {
        return false;
    }
    if (!expect(reached_output->mission_event.progress.completed_waypoints >= 1,
                "waypoint reached signal should advance mission progress")) {
        return false;
    }

    runtime.stop();
    return true;
}

} // namespace

int main() {
    if (!testRuntimeRoutesAutonomyOutputIntoControlPath()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
