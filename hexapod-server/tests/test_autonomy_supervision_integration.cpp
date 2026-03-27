#include "autonomy/modules/autonomy_stack.hpp"

#include <algorithm>
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

RobotState makeEstimatorState(uint64_t timestamp_us, double x_m, double y_m, double yaw_rad) {
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
    mission.mission_id = "autonomy-supervision-mission";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 2.0, .y_m = 0.5, .yaw_rad = 0.1},
    };
    return mission;
}

bool testContractsDefineBoundariesAndPolicies() {
    autonomy::AutonomyStack stack;
    const auto contracts = stack.processContracts();

    const auto critical_iter = std::find_if(contracts.cbegin(), contracts.cend(), [](const auto& contract) {
        return contract.module_name == "motion_arbiter";
    });
    if (!expect(critical_iter != contracts.cend(), "motion arbiter process contract should exist")) {
        return false;
    }
    if (!expect(critical_iter->criticality == autonomy::ProcessCriticality::Critical,
                "motion arbiter should be marked critical")) {
        return false;
    }

    const auto soft_rt_iter = std::find_if(contracts.cbegin(), contracts.cend(), [](const auto& contract) {
        return contract.module_name == "localization";
    });
    if (!expect(soft_rt_iter != contracts.cend(), "localization process contract should exist")) {
        return false;
    }
    if (!expect(soft_rt_iter->criticality == autonomy::ProcessCriticality::SoftRealtime,
                "localization should be marked soft-RT")) {
        return false;
    }

    const auto noncritical_iter = std::find_if(contracts.cbegin(), contracts.cend(), [](const auto& contract) {
        return contract.module_name == "mission_scripting";
    });
    if (!expect(noncritical_iter != contracts.cend(), "mission scripting process contract should exist")) {
        return false;
    }
    return expect(noncritical_iter->criticality == autonomy::ProcessCriticality::NonCritical,
                  "mission scripting should be marked noncritical");
}

bool testCrashIsolationRestartAndDegradedFallback() {
    autonomy::AutonomyStack stack;
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }
    if (!expect(stack.loadMission(makeMission()).accepted, "mission load should succeed")) {
        return false;
    }
    if (!expect(stack.startMission().accepted, "mission start should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput nominal{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0),
                           },
                           &nominal),
                "nominal step should succeed")) {
        return false;
    }
    if (!expect(!nominal.degraded_mode, "nominal step should not be degraded")) {
        return false;
    }

    autonomy::AutonomyStepOutput crash_step{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 20,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(19'000, 0.1, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "global_planner", .crash = true},
                               },
                           },
                           &crash_step),
                "step with soft-RT crash should succeed via isolation/restart")) {
        return false;
    }
    if (!expect(crash_step.degraded_mode, "crash step should transition to degraded mode")) {
        return false;
    }
    if (!expect(crash_step.degraded_reason == "planner unavailable", "planner crash should trigger planner fallback")) {
        return false;
    }
    if (!expect(!crash_step.locomotion_command.sent, "planner fallback should suppress locomotion")) {
        return false;
    }

    const auto planner_status = stack.supervisorStatus("global_planner");
    if (!expect(planner_status.has_value(), "planner status should be available")) {
        return false;
    }
    if (!expect(planner_status->restart_count == 1, "planner crash should increment restart count")) {
        return false;
    }
    return expect(planner_status->isolated_fault, "planner fault should be isolated from the rest of stack");
}

bool testTimeoutStaleLocalizationAndDispatchFailureFallback() {
    autonomy::AutonomyStack stack;
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }
    if (!expect(stack.loadMission(makeMission()).accepted, "mission load should succeed")) {
        return false;
    }
    if (!expect(stack.startMission().accepted, "mission start should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput stale_localization{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 30,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(29'000, 0.2, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "localization", .timeout = true},
                               },
                           },
                           &stale_localization),
                "localization timeout step should succeed")) {
        return false;
    }
    if (!expect(stale_localization.degraded_mode, "stale localization should trigger degraded mode")) {
        return false;
    }
    if (!expect(stale_localization.degraded_reason == "stale localization",
                "localization timeout should trigger stale localization fallback")) {
        return false;
    }

    autonomy::AutonomyStepOutput locomotion_failure{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 40,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(39'000, 0.3, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "locomotion_interface", .crash = true},
                               },
                           },
                           &locomotion_failure),
                "locomotion crash step should succeed due to restart policy")) {
        return false;
    }
    if (!expect(locomotion_failure.degraded_mode, "locomotion failure should trigger degraded mode")) {
        return false;
    }
    if (!expect(locomotion_failure.degraded_reason == "locomotion dispatch failure",
                "locomotion failure should trigger safe dispatch fallback")) {
        return false;
    }

    const auto locomotion_status = stack.supervisorStatus("locomotion_interface");
    if (!expect(locomotion_status.has_value(), "locomotion status should be available")) {
        return false;
    }
    return expect(locomotion_status->restart_count == 1,
                  "locomotion crash should be restarted once by supervisor");
}

} // namespace

int main() {
    if (!testContractsDefineBoundariesAndPolicies() ||
        !testCrashIsolationRestartAndDegradedFallback() ||
        !testTimeoutStaleLocalizationAndDispatchFailureFallback()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
