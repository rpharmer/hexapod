#include "autonomy/modules/autonomy_stack.hpp"

#include <string>

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
    const std::string correlation_id = "autonomy-step-" + std::to_string(++generated_correlation_id_);
    ContractEnvelope envelope = makeInternalEnvelope("autonomy_step_input", "map", correlation_id, input.now_ms);
    return step(input, envelope, output);
}

bool AutonomyStack::step(const AutonomyStepInput& input,
                         const ContractEnvelope& envelope,
                         AutonomyStepOutput* output,
                         const ContractValidationConfig& config) {
    if (!output) {
        return false;
    }

    if (!validateEnvelopeForStream(envelope, input.now_ms, config, "autonomy_step_input")) {
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
    const std::string correlation_id = envelope.correlation_id.empty()
                                           ? "autonomy-step-" + std::to_string(++generated_correlation_id_)
                                           : envelope.correlation_id;

    if (mission) {
        const auto nav_envelope = makeInternalEnvelope("navigation_manager.intent", "map", correlation_id, input.now_ms);
        if (!validateEnvelopeForStream(nav_envelope, input.now_ms, config, "navigation_manager.intent")) {
            return false;
        }
        output->navigation_update = navigation_manager_module_.computeIntent(
            *mission,
            output->mission_event.progress.completed_waypoints,
            input.blocked,
            nav_envelope);
    } else {
        output->navigation_update = NavigationUpdate{};
    }

    const auto localization_envelope = makeInternalEnvelope("localization.estimate", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(localization_envelope, input.now_ms, config, "localization.estimate")) {
        return false;
    }
    output->localization_estimate = localization_module_.update(output->navigation_update,
                                                                input.now_ms,
                                                                localization_envelope);

    const auto world_model_envelope = makeInternalEnvelope("world_model.snapshot", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(world_model_envelope, input.now_ms, config, "world_model.snapshot")) {
        return false;
    }
    output->world_model_snapshot = world_model_module_.update(
        output->localization_estimate,
        input.blocked,
        input.now_ms,
        world_model_envelope);

    const auto traversability_envelope = makeInternalEnvelope("traversability.report", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(traversability_envelope, input.now_ms, config, "traversability.report")) {
        return false;
    }
    output->traversability_report = traversability_analyzer_module_.analyze(
        output->world_model_snapshot,
        input.now_ms,
        traversability_envelope);

    const auto global_plan_envelope = makeInternalEnvelope("global_planner.plan", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(global_plan_envelope, input.now_ms, config, "global_planner.plan")) {
        return false;
    }
    output->global_plan = global_planner_module_.plan(
        output->navigation_update,
        output->traversability_report,
        global_plan_envelope);

    const auto local_plan_envelope = makeInternalEnvelope("local_planner.plan", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(local_plan_envelope, input.now_ms, config, "local_planner.plan")) {
        return false;
    }
    output->local_plan = local_planner_module_.plan(output->global_plan,
                                                    input.blocked,
                                                    local_plan_envelope);

    const auto progress_envelope = makeInternalEnvelope("progress_monitor.evaluation", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(progress_envelope, input.now_ms, config, "progress_monitor.evaluation")) {
        return false;
    }
    output->progress_evaluation = progress_monitor_module_.evaluate(ProgressSample{
        .timestamp_ms = input.now_ms,
        .completed_waypoints = output->mission_event.progress.completed_waypoints,
    }, progress_envelope);

    const bool recovery_trigger = input.blocked || output->progress_evaluation.no_progress;
    const auto recovery_envelope = makeInternalEnvelope("recovery_manager.decision", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(recovery_envelope, input.now_ms, config, "recovery_manager.decision")) {
        return false;
    }
    output->recovery_decision = recovery_manager_module_.onNoProgress(recovery_trigger,
                                                                      recovery_envelope);
    if (recovery_trigger) {
        const auto recovery_event = mission_executive_.onRecoveryDecision(output->recovery_decision);
        if (recovery_event.accepted) {
            output->mission_event = recovery_event;
        }
    }

    const auto motion_envelope = makeInternalEnvelope("motion_arbiter.decision", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(motion_envelope, input.now_ms, config, "motion_arbiter.decision")) {
        return false;
    }
    output->motion_decision = motion_arbiter_module_.arbitrate(
        input.estop,
        input.hold,
        output->recovery_decision.recovery_active,
        output->navigation_update,
        motion_envelope);

    const auto locomotion_envelope = makeInternalEnvelope("locomotion_interface.command", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(locomotion_envelope, input.now_ms, config, "locomotion_interface.command")) {
        return false;
    }
    output->locomotion_command = locomotion_interface_module_.dispatch(
        output->motion_decision,
        output->local_plan,
        locomotion_envelope);

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
    last_sample_id_by_stream_.clear();
    generated_sample_id_ = 0;
    generated_correlation_id_ = 0;
    return mission_executive_.loadMission(mission);
}

MissionScriptLoadResult AutonomyStack::loadMissionScript(const std::string& script,
                                                         const ContractEnvelope& envelope,
                                                         uint64_t now_ms,
                                                         const ContractValidationConfig& config) {
    MissionScriptLoadResult result{};
    const std::string stream_id = envelope.stream_id.empty() ? "mission_script" : envelope.stream_id;
    std::optional<uint64_t> last_sample_id = std::nullopt;
    const auto sample_it = last_sample_id_by_stream_.find(stream_id);
    if (sample_it != last_sample_id_by_stream_.end()) {
        last_sample_id = sample_it->second;
    }

    result.contract_validation = contract_enforcer_.validate(envelope, now_ms, config, last_sample_id);
    if (!result.contract_validation.valid) {
        result.error = result.contract_validation.message;
        return result;
    }

    if (!validateEnvelopeForStream(envelope, now_ms, config, "mission_script")) {
        result.contract_validation = contract_enforcer_.validate(envelope,
                                                                 now_ms,
                                                                 config,
                                                                 last_sample_id);
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

ContractEnvelope AutonomyStack::makeInternalEnvelope(const std::string& stream_id,
                                                     const std::string& frame_id,
                                                     const std::string& correlation_id,
                                                     uint64_t timestamp_ms) {
    return ContractEnvelope{
        .contract_version = "v1.0",
        .frame_id = frame_id,
        .correlation_id = correlation_id,
        .stream_id = stream_id,
        .sample_id = ++generated_sample_id_,
        .timestamp_ms = timestamp_ms,
    };
}

bool AutonomyStack::validateEnvelopeForStream(const ContractEnvelope& envelope,
                                              uint64_t now_ms,
                                              const ContractValidationConfig& config,
                                              const std::string& fallback_stream_id) {
    const std::string stream_id = envelope.stream_id.empty() ? fallback_stream_id : envelope.stream_id;
    std::optional<uint64_t> last_sample_id = std::nullopt;
    const auto it = last_sample_id_by_stream_.find(stream_id);
    if (it != last_sample_id_by_stream_.end()) {
        last_sample_id = it->second;
    }

    const auto result = contract_enforcer_.validate(envelope, now_ms, config, last_sample_id);
    if (!result.valid) {
        return false;
    }

    last_sample_id_by_stream_[stream_id] = envelope.sample_id;
    return true;
}

} // namespace autonomy
