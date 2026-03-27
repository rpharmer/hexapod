#include "autonomy/modules/autonomy_stack.hpp"

namespace autonomy {

AutonomyStack::AutonomyStack(const AutonomyStackConfig& config)
    : progress_monitor_module_(config.no_progress_timeout_ms),
      recovery_manager_module_(config.recovery_retry_budget) {}

bool AutonomyStack::init() {
    for (auto* module : modules()) {
        if (!module->init()) {
            return false;
        }
    }
    return true;
}

bool AutonomyStack::start() {
    for (auto* module : modules()) {
        if (!module->start()) {
            return false;
        }
    }
    return true;
}

bool AutonomyStack::step(const AutonomyStepInput& input, AutonomyStepOutput* output) {
    if (!output) {
        return false;
    }

    for (auto* module : modules()) {
        if (!module->step(input.now_ms)) {
            return false;
        }
    }

    output->mission_event = MissionEvent{
        .accepted = true,
        .state = mission_executive_.state(),
        .reason = {},
        .progress = mission_executive_.currentProgress(),
    };

    if (input.waypoint_reached) {
        output->mission_event = mission_executive_.markWaypointReached();
        if (output->mission_event.accepted) {
            progress_monitor_module_.reset();
            recovery_manager_module_.reset();
        }
    }

    const auto* mission = mission_executive_.activeMission();
    if (mission) {
        output->navigation_update = navigation_manager_module_.computeIntent(
            *mission,
            output->mission_event.progress.completed_waypoints,
            input.blocked);
    } else {
        output->navigation_update = NavigationUpdate{};
    }

    output->localization_estimate = localization_module_.update(output->navigation_update, input.now_ms);
    output->world_model_snapshot = world_model_module_.update(
        output->localization_estimate,
        input.map_slice_input,
        input.blocked,
        input.now_ms);
    output->traversability_report = traversability_analyzer_module_.analyze(
        output->world_model_snapshot,
        input.now_ms);
    output->global_plan = global_planner_module_.plan(
        output->navigation_update,
        output->traversability_report);
    output->local_plan = local_planner_module_.plan(output->global_plan);

    output->progress_evaluation = progress_monitor_module_.evaluate(ProgressSample{
        .timestamp_ms = input.now_ms,
        .completed_waypoints = output->mission_event.progress.completed_waypoints,
    });

    const bool recovery_trigger = input.blocked || output->progress_evaluation.no_progress;
    output->recovery_decision = recovery_manager_module_.onNoProgress(recovery_trigger);
    if (recovery_trigger) {
        const auto recovery_event = mission_executive_.onRecoveryDecision(output->recovery_decision);
        if (recovery_event.accepted) {
            output->mission_event = recovery_event;
        }
    }

    output->motion_decision = motion_arbiter_module_.arbitrate(
        input.estop,
        input.hold,
        output->recovery_decision.recovery_active,
        output->navigation_update);
    output->locomotion_command = locomotion_interface_module_.dispatch(
        output->motion_decision,
        output->local_plan);

    return true;
}

void AutonomyStack::stop() {
    for (auto* module : modules()) {
        module->stop();
    }
}

MissionEvent AutonomyStack::loadMission(const WaypointMission& mission) {
    progress_monitor_module_.reset();
    recovery_manager_module_.reset();
    last_sample_id_.reset();
    return mission_executive_.loadMission(mission);
}

MissionScriptLoadResult AutonomyStack::loadMissionScript(const std::string& script,
                                                         const ContractEnvelope& envelope,
                                                         uint64_t now_ms,
                                                         const ContractValidationConfig& config) {
    MissionScriptLoadResult result{};
    result.contract_validation = contract_enforcer_.validate(envelope, now_ms, config, last_sample_id_);
    if (!result.contract_validation.valid) {
        result.error = result.contract_validation.message;
        return result;
    }

    result.parse_result = mission_scripting_.parseWaypointMission(script);
    if (!result.parse_result.ok) {
        result.error = result.parse_result.error;
        return result;
    }

    result.mission_event = loadMission(result.parse_result.mission);
    result.accepted = result.mission_event.accepted;
    if (!result.accepted && result.error.empty()) {
        result.error = result.mission_event.reason;
    }

    if (result.accepted) {
        last_sample_id_ = envelope.sample_id;
    }
    return result;
}

MissionEvent AutonomyStack::startMission() {
    progress_monitor_module_.reset();
    recovery_manager_module_.reset();
    return mission_executive_.start();
}

AutonomyStack::ModuleArray AutonomyStack::modules() {
    return {
        &mission_executive_module_,
        &mission_scripting_module_,
        &navigation_manager_module_,
        &recovery_manager_module_,
        &motion_arbiter_module_,
        &localization_module_,
        &world_model_module_,
        &traversability_analyzer_module_,
        &global_planner_module_,
        &local_planner_module_,
        &locomotion_interface_module_,
        &progress_monitor_module_,
    };
}

} // namespace autonomy
