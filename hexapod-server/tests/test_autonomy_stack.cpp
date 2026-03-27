#include "autonomy/modules/autonomy_stack.hpp"

#include <cstdlib>
#include <iostream>

namespace {
constexpr const char* kMissionScriptIngressStreamId = "autonomy.mission_script.ingress";
constexpr const char* kAutonomyStepIngressStreamId = "autonomy.step.ingress";

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
                               .map_slice_input = autonomy::MapSliceInput{
                                   .has_occupancy = true,
                                   .has_elevation = true,
                                   .has_risk_confidence = true,
                                   .occupancy = 0.1,
                                   .elevation_m = 0.01,
                                   .risk_confidence = 0.95,
                               },
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
    if (!expect(first.global_plan.status == autonomy::PlannerStatus::Ready,
                "nominal global plan should be marked ready")) {
        return false;
    }
    if (!expect(first.local_plan.has_command, "local planner should produce local command")) {
        return false;
    }
    if (!expect(first.local_plan.status == autonomy::PlannerStatus::Ready,
                "nominal local plan should be marked ready")) {
        return false;
    }
    if (!expect(first.motion_decision.allow_motion, "motion should be allowed on active nav intent")) {
        return false;
    }
    if (!expect(first.mission_event.progress.mission_id == "autonomy-stack-mission",
                "mission telemetry should expose mission id")) {
        return false;
    }
    if (!expect(first.mission_event.progress.total_waypoints == 2,
                "mission telemetry should expose total waypoint count")) {
        return false;
    }
    if (!expect(first.locomotion_command.sent, "locomotion interface should dispatch allowed motion")) {
        return false;
    }
    if (!expect(first.locomotion_command.status == autonomy::LocomotionCommand::DispatchStatus::Dispatched,
                "locomotion command should report dispatch success")) {
        return false;
    }

    autonomy::AutonomyStepOutput second{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .blocked = false,
                               .map_slice_input = autonomy::MapSliceInput{
                                   .has_occupancy = true,
                                   .has_elevation = true,
                                   .has_risk_confidence = true,
                                   .occupancy = 0.2,
                                   .elevation_m = 0.03,
                                   .risk_confidence = 0.9,
                               },
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
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .blocked = true,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 1.0},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0),
                           },
                           &step1),
                "first blocked step should succeed")) {
        return false;
    }
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .blocked = true,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 1.0},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(19'000, 0.0, 0.0, 0.0),
                           },
                           &step2),
                "second blocked step should succeed")) {
        return false;
    }
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 30,
                               .blocked = true,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 1.0},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(29'000, 0.0, 0.0, 0.0),
                           },
                           &step3),
                "third blocked step should succeed")) {
        return false;
    }

    if (!expect(step1.recovery_decision.action == autonomy::RecoveryAction::Hold,
                "first blocked step should hold")) {
        return false;
    }
    if (!expect(step1.mission_event.state == autonomy::MissionState::Paused,
                "active recovery should pause mission while blocked")) {
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
    if (!expect(step3.mission_event.reason == "retry budget exhausted",
                "abort recovery should expose explicit telemetry abort reason")) {
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
    if (!expect(step1.global_plan.status == autonomy::PlannerStatus::NoPlan,
                "blocked navigation should surface no-plan global status")) {
        return false;
    }
    if (!expect(step1.local_plan.status == autonomy::PlannerStatus::NoPlan,
                "blocked navigation should surface no-plan local status")) {
        return false;
    }
    if (!expect(step1.local_plan.reason == "no-navigation-intent",
                "local plan reason should preserve no-plan-vs-unsafe distinction")) {
        return false;
    }

    stack.stop();
    return true;
}

bool testDispatchFailureTriggersRecoveryAndMotionGate() {
    bool sink_invoked = false;
    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
        .recovery_retry_budget = 2,
        .locomotion_command_sink =
            [&](const autonomy::Waypoint&, std::string* failure_reason) {
                sink_invoked = true;
                if (failure_reason) {
                    *failure_reason = "hardware bridge write failed";
                }
                return false;
            },
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

    autonomy::AutonomyStepOutput step{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .blocked = false,
                               .map_slice_input = autonomy::MapSliceInput{
                                   .has_occupancy = true,
                                   .occupancy = 0.1,
                               },
                               .waypoint_reached = false,
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0),
                           },
                           &step),
                "step should succeed")) {
        return false;
    }

    if (!expect(sink_invoked, "locomotion dispatch should invoke configured command sink")) {
        return false;
    }
    if (!expect(step.locomotion_command.status == autonomy::LocomotionCommand::DispatchStatus::DispatchFailed,
                "dispatch failure should be explicitly reported")) {
        return false;
    }
    if (!expect(!step.locomotion_command.write_ok, "failed dispatch should report write failure")) {
        return false;
    }
    if (!expect(step.recovery_decision.recovery_active,
                "dispatch failure should trigger recovery handling")) {
        return false;
    }
    if (!expect(step.recovery_decision.action == autonomy::RecoveryAction::Hold,
                "first dispatch failure should request hold recovery action")) {
        return false;
    }
    if (!expect(!step.motion_decision.allow_motion,
                "dispatch failure recovery should gate subsequent motion commands")) {
        return false;
    }

    stack.stop();
    return true;
}

bool testRecoverySuccessResumesMissionExecution() {
    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
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

    autonomy::AutonomyStepOutput blocked{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .blocked = true,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0),
                           },
                           &blocked),
                "blocked step should succeed")) {
        return false;
    }
    if (!expect(blocked.mission_event.state == autonomy::MissionState::Paused,
                "blocked recovery should pause mission")) {
        return false;
    }
    if (!expect(blocked.recovery_decision.recovery_active,
                "blocked recovery decision should be active")) {
        return false;
    }

    autonomy::AutonomyStepOutput resumed{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .blocked = false,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .waypoint_reached = true,
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(19'000, 0.1, 0.0, 0.0),
                           },
                           &resumed),
                "recovery-clear step should succeed")) {
        return false;
    }
    if (!expect(resumed.recovery_decision.action == autonomy::RecoveryAction::None,
                "recovery-clear step should reset to no recovery action")) {
        return false;
    }
    if (!expect(resumed.mission_event.state == autonomy::MissionState::Exec,
                "recovery-clear step should resume mission to EXEC")) {
        return false;
    }
    stack.stop();
    return true;
}

bool testRecoveryDoesNotBypassEStopOrHoldPriority() {
    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
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

    autonomy::AutonomyStepOutput estop_step{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 10, .estop = true, .blocked = true}, &estop_step),
                "estop + recovery step should succeed")) {
        return false;
    }
    if (!expect(estop_step.recovery_decision.recovery_active,
                "estop case should still detect active recovery")) {
        return false;
    }
    if (!expect(estop_step.motion_decision.source == autonomy::MotionSource::EStop,
                "estop should keep highest arbitration priority during recovery")) {
        return false;
    }
    if (!expect(!estop_step.locomotion_command.sent,
                "estop should prevent locomotion dispatch during recovery")) {
        return false;
    }

    autonomy::AutonomyStepOutput hold_step{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 20, .hold = true, .blocked = true}, &hold_step),
                "hold + recovery step should succeed")) {
        return false;
    }
    if (!expect(hold_step.recovery_decision.recovery_active,
                "hold case should still detect active recovery")) {
        return false;
    }
    if (!expect(hold_step.motion_decision.source == autonomy::MotionSource::Hold,
                "hold should outrank recovery in arbitration")) {
        return false;
    }
    if (!expect(!hold_step.locomotion_command.sent,
                "hold should prevent locomotion dispatch during recovery")) {
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
        .stream_id = kMissionScriptIngressStreamId,
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

bool testTerrainAndRiskChangesUpdatePlannerBehavior() {
    autonomy::AutonomyStack stack;
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

    autonomy::AutonomyStepOutput nominal{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .blocked = false,
                               .map_slice_input = autonomy::MapSliceInput{
                                   .has_occupancy = true,
                                   .has_elevation = true,
                                   .has_risk_confidence = true,
                                   .occupancy = 0.1,
                                   .elevation_m = 0.02,
                                   .risk_confidence = 0.95,
                               },
                           },
                           &nominal),
                "nominal terrain step should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput hazardous{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .blocked = false,
                               .map_slice_input = autonomy::MapSliceInput{
                                   .has_occupancy = true,
                                   .has_elevation = true,
                                   .has_risk_confidence = true,
                                   .occupancy = 0.95,
                                   .elevation_m = 0.9,
                                   .risk_confidence = 0.1,
                               },
                           },
                           &hazardous),
                "hazardous terrain step should succeed")) {
        return false;
    }

    stack.stop();

    return expect(nominal.local_plan.has_command, "nominal terrain should allow local planning") &&
           expect(!hazardous.traversability_report.traversable, "hazardous terrain should be non-traversable") &&
           expect(!hazardous.global_plan.has_plan, "hazardous terrain should suppress global plan") &&
           expect(!hazardous.local_plan.has_command, "hazardous terrain should suppress local command");
}

bool testStepRejectsStaleEnvelope() {
    autonomy::AutonomyStack stack;
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

    autonomy::AutonomyStepOutput output{};
    const autonomy::ContractEnvelope stale_envelope{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-stale",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 1,
        .timestamp_ms = 100,
    };
    autonomy::ContractValidationConfig config{};
    config.max_age_ms = 5;

    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 200}, stale_envelope, &output, config),
                  "stale step envelope should be rejected");
}

bool testStepRejectsNonMonotonicSamplePerStream() {
    autonomy::AutonomyStack stack;
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
    const autonomy::ContractEnvelope envelope_a{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-monotonic-a",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 7,
        .timestamp_ms = 10,
    };
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 10}, envelope_a, &first),
                "first step with explicit envelope should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput second{};
    const autonomy::ContractEnvelope envelope_b{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-monotonic-b",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 7,
        .timestamp_ms = 20,
    };
    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 20}, envelope_b, &second),
                  "equal sample id on same stream should be rejected");
}

bool testStepRejectsInvalidContractVersion() {
    autonomy::AutonomyStack stack;
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

    autonomy::AutonomyStepOutput output{};
    const autonomy::ContractEnvelope envelope{
        .contract_version = "v2.0",
        .frame_id = "map",
        .correlation_id = "corr-version",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 1,
        .timestamp_ms = 10,
    };
    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 10}, envelope, &output),
                  "invalid major contract version should be rejected for step payload ingress");
}

bool testStepRejectsMissingFrameAndCorrelationMetadata() {
    autonomy::AutonomyStack stack;
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

    autonomy::AutonomyStepOutput output{};
    const autonomy::ContractEnvelope envelope{
        .contract_version = "v1.0",
        .frame_id = "",
        .correlation_id = "",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 1,
        .timestamp_ms = 10,
    };
    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 10}, envelope, &output),
                  "missing frame/correlation metadata should be rejected on step ingress");
}

bool testStepRejectsOutOfOrderSampleId() {
    autonomy::AutonomyStack stack;
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
    const autonomy::ContractEnvelope envelope_a{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-order-a",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 10,
        .timestamp_ms = 10,
    };
    if (!expect(stack.step(autonomy::AutonomyStepInput{.now_ms = 10}, envelope_a, &first),
                "first step should pass")) {
        return false;
    }

    autonomy::AutonomyStepOutput second{};
    const autonomy::ContractEnvelope envelope_b{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-order-b",
        .stream_id = kAutonomyStepIngressStreamId,
        .sample_id = 9,
        .timestamp_ms = 20,
    };
    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 20}, envelope_b, &second),
                  "out-of-order sample id should be rejected");
}

bool testStepRejectsStreamIdDrift() {
    autonomy::AutonomyStack stack;
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

    autonomy::AutonomyStepOutput output{};
    const autonomy::ContractEnvelope envelope{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-drift",
        .stream_id = "autonomy_step_input",
        .sample_id = 1,
        .timestamp_ms = 10,
    };
    return expect(!stack.step(autonomy::AutonomyStepInput{.now_ms = 10}, envelope, &output),
                  "non-canonical stream ids should be rejected to avoid drift");
}

bool testMissionScriptRejectsDuplicateSampleId() {
    autonomy::AutonomyStack stack;
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }

    const std::string script =
        "mission_id=script-dup\n"
        "waypoint,map,0.0,0.0,0.0\n";

    autonomy::ContractEnvelope envelope{
        .contract_version = "v1.0",
        .frame_id = "map",
        .correlation_id = "corr-script-a",
        .stream_id = kMissionScriptIngressStreamId,
        .sample_id = 1,
        .timestamp_ms = 10,
    };
    const auto first_load = stack.loadMissionScript(script, envelope, 20);
    if (!expect(first_load.accepted, "first mission script load should succeed")) {
        return false;
    }

    envelope.correlation_id = "corr-script-b";
    envelope.timestamp_ms = 30;
    const auto duplicate_load = stack.loadMissionScript(script, envelope, 30);
    return expect(!duplicate_load.accepted, "duplicate mission script sample id should be rejected") &&
           expect(duplicate_load.contract_validation.non_monotonic_sample_id,
                  "duplicate mission script sample id should flag monotonic violation");
}

} // namespace

int main() {
    if (!testHappyPathWiresMissionNavAndMotion() ||
        !testBlockedFlowEscalatesRecoveryToAbort() ||
        !testDispatchFailureTriggersRecoveryAndMotionGate() ||
        !testRecoverySuccessResumesMissionExecution() ||
        !testRecoveryDoesNotBypassEStopOrHoldPriority() ||
        !testScriptAndContractLoadPath() ||
        !testTerrainAndRiskChangesUpdatePlannerBehavior() ||
        !testStepRejectsStaleEnvelope() ||
        !testStepRejectsNonMonotonicSamplePerStream() ||
        !testStepRejectsInvalidContractVersion() ||
        !testStepRejectsMissingFrameAndCorrelationMetadata() ||
        !testStepRejectsOutOfOrderSampleId() ||
        !testStepRejectsStreamIdDrift() ||
        !testMissionScriptRejectsDuplicateSampleId()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
