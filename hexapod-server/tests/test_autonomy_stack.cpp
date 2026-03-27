#include "autonomy/modules/autonomy_stack.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RobotState makeEstimatorState(uint64_t timestamp_us,
                              double x_m,
                              double y_m,
                              double yaw_rad) {
    RobotState est{};
    est.valid = true;
    est.has_body_pose_state = true;
    est.timestamp_us = TimePointUs{timestamp_us};
    est.body_pose_state.body_trans_m.x = x_m;
    est.body_pose_state.body_trans_m.y = y_m;
    est.body_pose_state.orientation_rad.z = yaw_rad;
    return est;
}

autonomy::WaypointMission makeMission() {
    autonomy::WaypointMission mission{};
    mission.mission_id = "autonomy-stack-mission";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.5, .y_m = 0.5, .yaw_rad = 0.2},
    };
    return mission;
}

bool testHappyPathWiresMissionNavAndMotion() {
    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 100,
        .recovery_retry_budget = 2,
    });
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }
    if (!expect(stack.loadMission(makeMission()).accepted, "load mission should succeed")) {
        return false;
    }
    if (!expect(stack.startMission().accepted, "start mission should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput first{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .blocked = false,
                               .waypoint_reached = false,
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.1, 0.2, 0.05),
                               .localization_frame_id = "map",
                           },
                           &first),
                "first step should succeed")) {
        return false;
    }
    if (!expect(first.navigation_update.has_intent, "navigation intent should be produced")) {
        return false;
    }
    if (!expect(first.localization_estimate.valid, "localization estimate should be produced")) {
        return false;
    }
    if (!expect(first.world_model_snapshot.has_map, "world model should be updated")) {
        return false;
    }
    if (!expect(first.traversability_report.traversable, "traversability should report traversable on nominal path")) {
        return false;
    }
    if (!expect(first.global_plan.has_plan, "global planner should produce plan")) {
        return false;
    }
    if (!expect(first.local_plan.has_command, "local planner should produce local command")) {
        return false;
    }
    if (!expect(first.motion_decision.allow_motion, "motion should be allowed on active nav intent")) {
        return false;
    }
    if (!expect(first.locomotion_command.sent, "locomotion interface should dispatch allowed motion")) {
        return false;
    }

    autonomy::AutonomyStepOutput second{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .blocked = false,
                               .waypoint_reached = true,
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(19'000, 1.4, 0.45, 0.19),
                               .localization_frame_id = "map",
                           },
                           &second),
                "second step should succeed")) {
        return false;
    }
    if (!expect(second.mission_event.progress.completed_waypoints == 1,
                "waypoint completion should advance mission progress")) {
        return false;
    }

    stack.stop();
    return true;
}

bool testBlockedFlowEscalatesRecoveryToAbort() {
    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
        .recovery_retry_budget = 1,
    });
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }
    if (!expect(stack.loadMission(makeMission()).accepted, "load mission should succeed")) {
        return false;
    }
    if (!expect(stack.startMission().accepted, "start mission should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput step1{};
    autonomy::AutonomyStepOutput step2{};
    autonomy::AutonomyStepOutput step3{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 10, .blocked = true, .has_estimator_state = true, .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0)}, &step1),
                "first blocked step should succeed")) {
        return false;
    }
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 20, .blocked = true, .has_estimator_state = true, .estimator_state = makeEstimatorState(19'000, 0.0, 0.0, 0.0)}, &step2),
                "second blocked step should succeed")) {
        return false;
    }
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 30, .blocked = true, .has_estimator_state = true, .estimator_state = makeEstimatorState(29'000, 0.0, 0.0, 0.0)}, &step3),
                "third blocked step should succeed")) {
        return false;
    }

    if (!expect(step1.recovery_decision.action == autonomy::RecoveryAction::Hold,
                "first blocked step should hold")) {
        return false;
    }
    if (!expect(step2.recovery_decision.action == autonomy::RecoveryAction::Replan,
                "second blocked step should replan when retry budget is exhausted")) {
        return false;
    }
    if (!expect(step3.recovery_decision.action == autonomy::RecoveryAction::Abort,
                "third blocked step should abort mission")) {
        return false;
    }
    if (!expect(step3.mission_event.state == autonomy::MissionState::Aborted,
                "abort recovery should push mission to aborted")) {
        return false;
    }
    if (!expect(!step3.motion_decision.allow_motion,
                "aborted recovery path should not allow motion")) {
        return false;
    }
    if (!expect(!step3.locomotion_command.sent,
                "aborted recovery path should suppress locomotion dispatch")) {
        return false;
    }

    stack.stop();
    return true;
}

bool testScriptAndContractLoadPath() {
    autonomy::AutonomyStack stack;
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }

    const std::string script =
        "mission_id=script-mission\n"
        "waypoint,map,0.0,0.0,0.0\n";

    autonomy::ContractEnvelope valid_envelope{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-1",
        .stream_id = "mission_script",
        .sample_id = 1,
        .timestamp_ms = 50,
    };

    const auto valid_load = stack.loadMissionScript(script, valid_envelope, 100);
    if (!expect(valid_load.accepted, "valid script+contract should load mission")) {
        return false;
    }

    autonomy::ContractEnvelope invalid_envelope = valid_envelope;
    invalid_envelope.contract_version = "v2.0";
    invalid_envelope.sample_id = 2;

    const auto invalid_load = stack.loadMissionScript(script, invalid_envelope, 120);
    return expect(!invalid_load.accepted, "incompatible contract version should be rejected") &&
           expect(invalid_load.contract_validation.invalid_version,
                  "invalid version flag should be set");
}

} // namespace

int main() {
    if (!testHappyPathWiresMissionNavAndMotion() ||
        !testBlockedFlowEscalatesRecoveryToAbort() ||
        !testScriptAndContractLoadPath()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
