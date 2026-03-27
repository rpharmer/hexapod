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
    const auto ipc_boundaries = stack.ipcBoundaryContracts();

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
    if (!expect(critical_iter->process_group == autonomy::ProcessGroup::Critical,
                "motion arbiter should be in critical process group")) {
        return false;
    }
    if (!expect(critical_iter->safe_stop_on_exhausted_restart,
                "critical modules should safe-stop when restart budget is exhausted")) {
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
    if (!expect(soft_rt_iter->process_group == autonomy::ProcessGroup::SoftRealtime,
                "localization should be in soft-RT process group")) {
        return false;
    }

    const auto noncritical_iter = std::find_if(contracts.cbegin(), contracts.cend(), [](const auto& contract) {
        return contract.module_name == "mission_scripting";
    });
    if (!expect(noncritical_iter != contracts.cend(), "mission scripting process contract should exist")) {
        return false;
    }
    if (!expect(noncritical_iter->criticality == autonomy::ProcessCriticality::NonCritical,
                "mission scripting should be marked noncritical")) {
        return false;
    }
    if (!expect(noncritical_iter->process_group == autonomy::ProcessGroup::NonCritical,
                "mission scripting should be in noncritical process group")) {
        return false;
    }
    if (!expect(!noncritical_iter->safe_stop_on_exhausted_restart,
                "noncritical modules should isolate on restart exhaustion")) {
        return false;
    }

    const auto critical_to_soft_rt = std::find_if(ipc_boundaries.cbegin(), ipc_boundaries.cend(), [](const auto& edge) {
        return edge.producer_module == "mission_executive" &&
               edge.consumer_module == "navigation_manager" &&
               edge.message_type == "NavigationUpdate";
    });
    if (!expect(critical_to_soft_rt != ipc_boundaries.cend(),
                "critical->soft-RT IPC boundary should be explicit")) {
        return false;
    }

    const auto soft_rt_to_critical = std::find_if(ipc_boundaries.cbegin(), ipc_boundaries.cend(), [](const auto& edge) {
        return edge.producer_module == "local_planner" &&
               edge.consumer_module == "locomotion_interface" &&
               edge.message_type == "LocalPlan";
    });
    if (!expect(soft_rt_to_critical != ipc_boundaries.cend(),
                "soft-RT->critical IPC boundary should be explicit")) {
        return false;
    }

    const auto noncritical_to_critical = std::find_if(ipc_boundaries.cbegin(), ipc_boundaries.cend(), [](const auto& edge) {
        return edge.producer_module == "recovery_manager" &&
               edge.consumer_module == "motion_arbiter" &&
               edge.message_type == "RecoveryDecision";
    });
    return expect(noncritical_to_critical != ipc_boundaries.cend(),
                  "noncritical->critical IPC boundary should be explicit");
}

bool testCrashIsolationPerProcessGroup() {
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
    if (!expect(planner_status->isolated_fault, "planner fault should be isolated from the rest of stack")) {
        return false;
    }

    autonomy::AutonomyStepOutput noncritical_crash{};
    if (!expect(stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 30,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(29'000, 0.2, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "mission_scripting", .crash = true},
                               },
                           },
                           &noncritical_crash),
                "noncritical crash should be isolated without taking down stack")) {
        return false;
    }

    const auto scripting_status = stack.supervisorStatus("mission_scripting");
    if (!expect(scripting_status.has_value(), "mission scripting status should be available")) {
        return false;
    }
    if (!expect(scripting_status->restart_count == 1, "noncritical crash should be restarted once")) {
        return false;
    }
    return expect(scripting_status->isolated_fault, "noncritical crash should remain isolated");
}

bool testTimeoutAndSafeStopPolicies() {
    autonomy::AutonomyStack timeout_stack;
    if (!expect(timeout_stack.init(), "timeout stack init should succeed")) {
        return false;
    }
    if (!expect(timeout_stack.start(), "timeout stack start should succeed")) {
        return false;
    }
    if (!expect(timeout_stack.loadMission(makeMission()).accepted, "timeout stack mission load should succeed")) {
        return false;
    }
    if (!expect(timeout_stack.startMission().accepted, "timeout stack mission start should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput stale_localization{};
    if (!expect(timeout_stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.3, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "localization", .timeout = true},
                               },
                           },
                           &stale_localization),
                "soft-RT timeout should be isolated")) {
        return false;
    }
    if (!expect(stale_localization.degraded_mode, "soft-RT timeout should force degraded mode")) {
        return false;
    }
    if (!expect(stale_localization.degraded_reason == "stale localization",
                "localization timeout should trigger stale localization reason")) {
        return false;
    }

    autonomy::AutonomyStack critical_stack;
    if (!expect(critical_stack.init(), "critical stack init should succeed")) {
        return false;
    }
    if (!expect(critical_stack.start(), "critical stack start should succeed")) {
        return false;
    }
    if (!expect(critical_stack.loadMission(makeMission()).accepted, "critical stack mission load should succeed")) {
        return false;
    }
    if (!expect(critical_stack.startMission().accepted, "critical stack mission start should succeed")) {
        return false;
    }

    autonomy::AutonomyStepOutput ignored{};
    if (!expect(critical_stack.step(autonomy::AutonomyStepInput{
                               .now_ms = 10,
                               .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                               .has_estimator_state = true,
                               .estimator_state = makeEstimatorState(9'000, 0.0, 0.0, 0.0),
                               .fault_injections = {
                                   autonomy::ModuleFaultInjection{.module_name = "motion_arbiter", .crash = true},
                               },
                           },
                           &ignored),
                "critical module first crash should be restarted")) {
        return false;
    }

    return expect(!critical_stack.step(autonomy::AutonomyStepInput{
                                           .now_ms = 20,
                                           .map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = 0.1},
                                           .has_estimator_state = true,
                                           .estimator_state = makeEstimatorState(19'000, 0.0, 0.0, 0.0),
                                           .fault_injections = {
                                               autonomy::ModuleFaultInjection{.module_name = "motion_arbiter", .crash = true},
                                           },
                                       },
                                       &ignored),
                  "critical module should safe-stop once restart budget is exhausted");
}

} // namespace

int main() {
    if (!testContractsDefineBoundariesAndPolicies() ||
        !testCrashIsolationPerProcessGroup() ||
        !testTimeoutAndSafeStopPolicies()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
