#include "robot_runtime.hpp"

#include "estimator.hpp"
#include "sim_hardware_bridge.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>

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

class CapturingTelemetryPublisher final : public telemetry::ITelemetryPublisher {
public:
    void publishGeometry(const HexapodGeometry&) override {}

    void publishControlStep(const telemetry::ControlStepTelemetry& telemetry) override {
        last_sample = telemetry;
        ++publish_count;
    }

    telemetry::TelemetryPublishCounters counters() const override {
        telemetry::TelemetryPublishCounters counters{};
        counters.packets_sent = publish_count;
        return counters;
    }

    std::optional<telemetry::ControlStepTelemetry> last_sample{};
    uint64_t publish_count{0};
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

double autonomyCommandSpeedMps(const autonomy::AutonomyStepOutput& output,
                               const control_config::ControlConfig& cfg) {
    const double target_x = output.locomotion_command.target.x_m;
    const double target_y = output.locomotion_command.target.y_m;
    const double distance = std::sqrt((target_x * target_x) + (target_y * target_y));
    return std::min(distance, cfg.gait.fallback_speed_mag.value);
}

double autonomyCommandHeadingRad(const autonomy::AutonomyStepOutput& output) {
    return std::atan2(output.locomotion_command.target.y_m, output.locomotion_command.target.x_m);
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
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{1000};

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassThroughEstimator>();
    auto telemetry_publisher = std::make_unique<CapturingTelemetryPublisher>();
    auto* telemetry_raw = telemetry_publisher.get();
    RobotRuntime runtime(
        std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry_publisher));

    if (!expect(runtime.init(), "runtime init should succeed with autonomy enabled")) {
        return false;
    }
    runtime.startTelemetry();
    if (!expect(runtime.loadAutonomyMission(makeMission()), "mission load should succeed")) {
        return false;
    }
    if (!expect(runtime.startAutonomyMission(), "mission start should succeed")) {
        return false;
    }

    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto first_output = runtime.lastAutonomyStepOutput();
    if (!expect(first_output.has_value(), "autonomy output should be captured each control step")) {
        return false;
    }
    if (!expect(first_output->motion_decision.allow_motion, "nominal motion decision should allow motion")) {
        return false;
    }
    if (!expect(first_output->locomotion_command.sent, "nominal locomotion command should be dispatched")) {
        return false;
    }
    const double first_speed_mps = autonomyCommandSpeedMps(*first_output, cfg);
    if (!expect(std::isfinite(first_speed_mps), "autonomy handoff speed should remain finite")) {
        return false;
    }
    if (!expect(first_speed_mps >= 0.0, "autonomy handoff speed should remain non-negative")) {
        return false;
    }
    if (!expect(first_speed_mps <= (cfg.gait.fallback_speed_mag.value + 1e-9),
                "autonomy handoff speed should be capped by fallback speed bound")) {
        return false;
    }
    const double first_heading_rad = autonomyCommandHeadingRad(*first_output);
    if (!expect(std::isfinite(first_heading_rad), "autonomy handoff heading should remain finite")) {
        return false;
    }
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto second_output = runtime.lastAutonomyStepOutput();
    if (!expect(second_output.has_value(), "autonomy output should remain available on nominal follow-up step")) {
        return false;
    }
    if (!expect(second_output->locomotion_command.sent,
                "autonomy locomotion command should continue dispatching after handoff")) {
        return false;
    }
    const double second_speed_mps = autonomyCommandSpeedMps(*second_output, cfg);
    if (!expect(std::isfinite(second_speed_mps), "post-handoff autonomy speed should remain finite")) {
        return false;
    }
    if (!expect(second_speed_mps >= 0.0, "post-handoff autonomy speed should remain non-negative")) {
        return false;
    }
    if (!expect(second_speed_mps <= (cfg.gait.fallback_speed_mag.value + 1e-9),
                "post-handoff autonomy speed should remain bounded by fallback speed")) {
        return false;
    }
    if (!expect(std::abs(second_speed_mps - first_speed_mps) <= (cfg.gait.fallback_speed_mag.value + 1e-9),
                "autonomy handoff should avoid discontinuous speed jumps")) {
        return false;
    }
    const double second_heading_rad = autonomyCommandHeadingRad(*second_output);
    if (!expect(std::isfinite(second_heading_rad), "post-handoff autonomy heading should remain finite")) {
        return false;
    }
    if (!expect(std::abs(second_heading_rad - first_heading_rad) <= 3.14159265358979323846,
                "autonomy handoff should avoid discontinuous heading jumps")) {
        return false;
    }
    if (!expect(runtime.getStatus().active_mode == RobotMode::WALK,
                "runtime control status should reflect autonomy WALK decision")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.mission_running, "mission should report running after start")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.mission_state == static_cast<uint8_t>(autonomy::MissionState::Exec),
                "runtime status should report EXEC mission state")) {
        return false;
    }
    if (!expect(telemetry_raw->last_sample.has_value(), "runtime should publish telemetry samples")) {
        return false;
    }
    if (!expect(telemetry_raw->last_sample->status.autonomy.mission_running,
                "operator telemetry should include running mission status")) {
        return false;
    }

    runtime.setAutonomyBlocked(true);
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto blocked_output = runtime.lastAutonomyStepOutput();
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
    if (!expect(runtime.getStatus().autonomy.blocked, "runtime status should report blocked signal")) {
        return false;
    }
    if (!expect(telemetry_raw->last_sample->status.autonomy.blocked,
                "operator telemetry should include blocked mission signal")) {
        return false;
    }

    runtime.setAutonomyBlocked(false);
    runtime.setAutonomyNoProgress(true);
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto recovery_output = runtime.lastAutonomyStepOutput();
    if (!expect(recovery_output.has_value(), "autonomy output should exist in recovery step")) {
        return false;
    }
    if (!expect(runtime.getStatus().active_mode == RobotMode::SAFE_IDLE,
                "recovery pause should gate motion to SAFE_IDLE")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.recovery_active,
                "runtime status should report active recovery during no-progress event")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.mission_state == static_cast<uint8_t>(autonomy::MissionState::Paused),
                "runtime status should report mission PAUSED during recovery")) {
        return false;
    }
    if (!expect(telemetry_raw->last_sample->status.autonomy.recovery_active,
                "operator telemetry should include active recovery state")) {
        return false;
    }

    runtime.setAutonomyNoProgress(false);
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    auto resumed_output = runtime.lastAutonomyStepOutput();
    if (!expect(resumed_output.has_value(), "autonomy output should exist after recovery clears")) {
        return false;
    }
    const double resumed_speed_mps = autonomyCommandSpeedMps(*resumed_output, cfg);
    if (!expect(std::isfinite(resumed_speed_mps), "post-recovery autonomy speed should remain finite")) {
        return false;
    }
    if (!expect(resumed_speed_mps >= 0.0, "post-recovery autonomy speed should remain non-negative")) {
        return false;
    }
    if (!expect(resumed_speed_mps <= (cfg.gait.fallback_speed_mag.value + 1e-9),
                "post-recovery autonomy speed should respect fallback speed cap")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.mission_state == static_cast<uint8_t>(autonomy::MissionState::Exec),
                "runtime status should return mission to EXEC when recovery clears")) {
        return false;
    }

    runtime.signalAutonomyWaypointReached();
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    const auto reached_output = runtime.lastAutonomyStepOutput();
    if (!expect(reached_output.has_value(), "autonomy output should exist for waypoint event")) {
        return false;
    }
    if (!expect(reached_output->mission_event.progress.completed_waypoints >= 1,
                "waypoint reached signal should advance mission progress")) {
        return false;
    }
    if (!expect(runtime.getStatus().autonomy.mission_completed_waypoints >= 1,
                "runtime status should report waypoint completion progress")) {
        return false;
    }

    runtime.signalAutonomyWaypointReached();
    runtime.setMotionIntent(makeFreshWalkIntent());
    runRuntimeStep(runtime);
    if (!expect(runtime.getStatus().autonomy.mission_state == static_cast<uint8_t>(autonomy::MissionState::Complete),
                "runtime status should report mission COMPLETE when final waypoint is reached")) {
        return false;
    }
    if (!expect(!runtime.getStatus().autonomy.mission_running,
                "runtime status should report mission not running after completion")) {
        return false;
    }
    if (!expect(telemetry_raw->last_sample->status.autonomy.mission_state ==
                    static_cast<uint8_t>(autonomy::MissionState::Complete),
                "operator telemetry should include mission completion state")) {
        return false;
    }

    runtime.stop();
    return true;
}

bool testInterleavedManualAndAutonomyIntentsPreserveFreshnessAndSampleMonotonicity() {
    control_config::ControlConfig cfg{};
    cfg.autonomy.enabled = true;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{1000};

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassThroughEstimator>();
    auto telemetry_publisher = std::make_unique<CapturingTelemetryPublisher>();
    auto* telemetry_raw = telemetry_publisher.get();
    RobotRuntime runtime(
        std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry_publisher));

    if (!expect(runtime.init(), "runtime init should succeed for source-switching monotonicity test")) {
        return false;
    }
    runtime.startTelemetry();
    if (!expect(runtime.loadAutonomyMission(makeMission()), "mission load should succeed")) {
        return false;
    }
    if (!expect(runtime.startAutonomyMission(), "mission start should succeed")) {
        return false;
    }

    uint64_t last_sample_id = 0;
    const auto run_interleaved_step = [&](MotionIntent intent, const char* step_label) {
        runtime.setMotionIntent(intent);
        runRuntimeStep(runtime);

        if (!expect(telemetry_raw->last_sample.has_value(), "telemetry sample should be available after step")) {
            return false;
        }
        const auto& telemetry = *telemetry_raw->last_sample;
        if (!expect(telemetry.motion_intent.sample_id != 0,
                    "intent sample ids should remain non-zero across source switches")) {
            return false;
        }
        if (!expect(telemetry.motion_intent.sample_id > last_sample_id,
                    "intent sample ids should remain strictly monotonic across source switches")) {
            return false;
        }
        last_sample_id = telemetry.motion_intent.sample_id;

        if (!expect(telemetry.status.active_fault != FaultCode::COMMAND_TIMEOUT,
                    "freshness gate should not falsely raise COMMAND_TIMEOUT when intent source switches")) {
            return false;
        }
        if (!expect(telemetry.runtime_freshness.total_rejects == 0,
                    "freshness gate should not reject valid interleaved manual/autonomy intent stream")) {
            return false;
        }
        if (!expect(telemetry.runtime_freshness.non_monotonic_id_rejects == 0,
                    "freshness gate should not record non-monotonic rejects for source switching alone")) {
            return false;
        }

        const auto step_output = runtime.lastAutonomyStepOutput();
        if (!expect(step_output.has_value(), "autonomy output should be produced for interleaved step")) {
            return false;
        }
        if (!expect(step_output->motion_decision.allow_motion || runtime.getStatus().active_mode == RobotMode::SAFE_IDLE,
                    step_label)) {
            return false;
        }
        return true;
    };

    MotionIntent manual_with_explicit_sample{};
    manual_with_explicit_sample.requested_mode = RobotMode::WALK;
    manual_with_explicit_sample.timestamp_us = now_us();
    manual_with_explicit_sample.sample_id = 100;
    if (!run_interleaved_step(manual_with_explicit_sample,
                              "interleaved step 1 should keep runtime in a valid autonomy control state")) {
        return false;
    }

    MotionIntent manual_with_runtime_sample = makeFreshWalkIntent();
    manual_with_runtime_sample.sample_id = 0;
    if (!run_interleaved_step(manual_with_runtime_sample,
                              "interleaved step 2 should keep runtime in a valid autonomy control state")) {
        return false;
    }

    MotionIntent manual_with_higher_explicit_sample = makeFreshWalkIntent();
    manual_with_higher_explicit_sample.sample_id = 200;
    if (!run_interleaved_step(manual_with_higher_explicit_sample,
                              "interleaved step 3 should keep runtime in a valid autonomy control state")) {
        return false;
    }

    MotionIntent manual_with_runtime_sample_again = makeFreshWalkIntent();
    manual_with_runtime_sample_again.sample_id = 0;
    if (!run_interleaved_step(manual_with_runtime_sample_again,
                              "interleaved step 4 should keep runtime in a valid autonomy control state")) {
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
    if (!testInterleavedManualAndAutonomyIntentsPreserveFreshnessAndSampleMonotonicity()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
