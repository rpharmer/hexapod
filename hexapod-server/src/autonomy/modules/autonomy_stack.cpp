#include "autonomy/modules/autonomy_stack.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <string>

namespace autonomy {
namespace {

constexpr const char* kStaleLocalization = "stale localization";
constexpr const char* kPlannerUnavailable = "planner unavailable";
constexpr const char* kLocomotionDispatchFailure = "locomotion dispatch failure";

} // namespace

AutonomyStack::AutonomyStack(const AutonomyStackConfig& config)
    : locomotion_interface_module_(config.locomotion_command_sink),
      progress_monitor_module_(config.no_progress_timeout_ms),
      recovery_manager_module_(config.recovery_retry_budget),
      supervisor_states_({
          SupervisorState{.contract = ModuleProcessContract{.module_name = "mission_executive", .criticality = ProcessCriticality::Critical, .heartbeat_timeout_ms = 100, .max_restarts = 1, .dependencies = {}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "mission_scripting", .criticality = ProcessCriticality::NonCritical, .heartbeat_timeout_ms = 500, .max_restarts = 1, .dependencies = {}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "navigation_manager", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 100, .max_restarts = 2, .dependencies = {"mission_executive"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "recovery_manager", .criticality = ProcessCriticality::NonCritical, .heartbeat_timeout_ms = 500, .max_restarts = 1, .dependencies = {"progress_monitor"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "motion_arbiter", .criticality = ProcessCriticality::Critical, .heartbeat_timeout_ms = 100, .max_restarts = 1, .dependencies = {"navigation_manager", "recovery_manager"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "localization", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 75, .max_restarts = 3, .dependencies = {"navigation_manager"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "world_model", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 100, .max_restarts = 2, .dependencies = {"localization"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "traversability_analyzer", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 100, .max_restarts = 2, .dependencies = {"world_model"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "global_planner", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 120, .max_restarts = 2, .dependencies = {"navigation_manager", "traversability_analyzer"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "local_planner", .criticality = ProcessCriticality::SoftRealtime, .heartbeat_timeout_ms = 120, .max_restarts = 2, .dependencies = {"global_planner"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "locomotion_interface", .criticality = ProcessCriticality::Critical, .heartbeat_timeout_ms = 75, .max_restarts = 2, .dependencies = {"motion_arbiter", "local_planner"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
          SupervisorState{.contract = ModuleProcessContract{.module_name = "progress_monitor", .criticality = ProcessCriticality::NonCritical, .heartbeat_timeout_ms = 500, .max_restarts = 1, .dependencies = {"mission_executive"}}, .restart_count = 0, .isolated_fault = false, .crashed = false, .timed_out = false, .last_fault = {}},
      }) {}

bool AutonomyStack::init() {
    const auto logger = logging::GetDefaultLogger();
    for (auto* module : modules()) {
        if (!module->init()) {
            if (logger) {
                LOG_ERROR(logger, "[autonomy] failed to initialize module: ", module->health().module_name);
            }
            return false;
        }
    }
    return true;
}

bool AutonomyStack::start() {
    const auto logger = logging::GetDefaultLogger();
    for (auto* module : modules()) {
        if (!module->start()) {
            if (logger) {
                LOG_ERROR(logger, "[autonomy] failed to start module: ", module->health().module_name);
            }
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
        if (const auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "[autonomy] step called with null output");
        }
        return false;
    }

    if (!validateEnvelopeForStream(envelope, input.now_ms, config, "autonomy_step_input")) {
        return false;
    }

    for (auto* module : modules()) {
        const bool crashed = faultInjected(input, module->health().module_name, &ModuleFaultInjection::crash);
        const bool timed_out = faultInjected(input, module->health().module_name, &ModuleFaultInjection::timeout);

        if (!crashed && !timed_out && !module->step(input.now_ms)) {
            return false;
        }
        if (!monitorAndRecoverModule(module, input.now_ms, crashed, timed_out)) {
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

    LocalizationSourceObservation localization_observation{};
    if (input.has_estimator_state) {
        localization_observation = localizationObservationFromEstimator(
            input.estimator_state,
            input.localization_frame_id);
    } else if (output->navigation_update.has_intent) {
        localization_observation = localizationObservationFromOdometry(
            output->navigation_update.intent.target.x_m,
            output->navigation_update.intent.target.y_m,
            output->navigation_update.intent.target.yaw_rad,
            input.now_ms,
            output->navigation_update.intent.target.frame_id.empty()
                ? "map"
                : output->navigation_update.intent.target.frame_id);
    }
    const auto localization_envelope = makeInternalEnvelope("localization.estimate", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(localization_envelope, input.now_ms, config, "localization.estimate")) {
        return false;
    }
    output->localization_estimate = localization_module_.update(localization_observation, input.now_ms, localization_envelope);

    const auto world_model_envelope = makeInternalEnvelope("world_model.snapshot", "map", correlation_id, input.now_ms);
    if (!validateEnvelopeForStream(world_model_envelope, input.now_ms, config, "world_model.snapshot")) {
        return false;
    }
    output->world_model_snapshot = world_model_module_.update(
        output->localization_estimate,
        input.map_slice_input,
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
                                                    input.now_ms,
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
    if (output->mission_event.state == MissionState::Exec || output->mission_event.state == MissionState::Paused) {
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
    if (output->locomotion_command.status == LocomotionCommand::DispatchStatus::DispatchFailed) {
        output->recovery_decision = recovery_manager_module_.onNoProgress(true, recovery_envelope);
        const auto recovery_event = mission_executive_.onRecoveryDecision(output->recovery_decision);
        if (recovery_event.accepted) {
            output->mission_event = recovery_event;
        }

        output->motion_decision = motion_arbiter_module_.arbitrate(
            input.estop,
            input.hold,
            output->recovery_decision.recovery_active,
            output->navigation_update,
            motion_envelope);
    }

    applyDegradedMode(input, output);

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

std::vector<ModuleProcessContract> AutonomyStack::processContracts() const {
    std::vector<ModuleProcessContract> contracts;
    contracts.reserve(supervisor_states_.size());
    for (const auto& state : supervisor_states_) {
        contracts.push_back(state.contract);
    }
    return contracts;
}

std::vector<ModuleSupervisorStatus> AutonomyStack::supervisorStatuses() const {
    std::vector<ModuleSupervisorStatus> statuses;
    statuses.reserve(supervisor_states_.size());

    for (const auto& state : supervisor_states_) {
        statuses.emplace_back(ModuleSupervisorStatus{
            .module_name = state.contract.module_name,
            .criticality = state.contract.criticality,
            .alive = !state.crashed,
            .timed_out = state.timed_out,
            .crashed = state.crashed,
            .isolated_fault = state.isolated_fault,
            .restart_count = state.restart_count,
            .heartbeat_timestamp_ms = 0,
            .last_fault = state.last_fault,
        });

        const auto* module = moduleByName(state.contract.module_name);
        if (module != nullptr) {
            statuses.back().heartbeat_timestamp_ms = module->health().heartbeat_timestamp_ms;
        }
    }

    return statuses;
}

std::optional<ModuleSupervisorStatus> AutonomyStack::supervisorStatus(std::string_view module_name) const {
    const auto statuses = supervisorStatuses();
    const auto iter = std::find_if(statuses.cbegin(), statuses.cend(), [&](const auto& status) {
        return status.module_name == module_name;
    });
    if (iter == statuses.cend()) {
        return std::nullopt;
    }
    return *iter;
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
        if (const auto logger = logging::GetDefaultLogger()) {
            LOG_WARN(logger,
                     "[autonomy] contract validation failed for stream=",
                     stream_id,
                     " sample_id=",
                     envelope.sample_id,
                     " reason=",
                     result.message);
        }
        return false;
    }

    last_sample_id_by_stream_[stream_id] = envelope.sample_id;
    return true;
}

AutonomyModuleStub* AutonomyStack::moduleByName(std::string_view module_name) {
    auto module_array = modules();
    const auto iter = std::find_if(module_array.begin(), module_array.end(), [&](auto* module) {
        return module->health().module_name == module_name;
    });
    if (iter == module_array.end()) {
        return nullptr;
    }
    return *iter;
}

const AutonomyModuleStub* AutonomyStack::moduleByName(std::string_view module_name) const {
    auto* mutable_this = const_cast<AutonomyStack*>(this);
    return mutable_this->moduleByName(module_name);
}

bool AutonomyStack::faultInjected(const AutonomyStepInput& input,
                                  std::string_view module_name,
                                  bool ModuleFaultInjection::*selector) const {
    return std::any_of(input.fault_injections.cbegin(), input.fault_injections.cend(), [&](const auto& fault) {
        return fault.module_name == module_name && fault.*selector;
    });
}

bool AutonomyStack::monitorAndRecoverModule(AutonomyModuleStub* module,
                                            uint64_t now_ms,
                                            bool crashed,
                                            bool timed_out) {
    const auto logger = logging::GetDefaultLogger();
    auto supervisor_iter = std::find_if(supervisor_states_.begin(), supervisor_states_.end(), [&](const auto& state) {
        return state.contract.module_name == module->health().module_name;
    });
    if (supervisor_iter == supervisor_states_.end()) {
        if (logger) {
            LOG_ERROR(logger, "[autonomy] missing supervisor state for module: ", module->health().module_name);
        }
        return false;
    }

    auto& supervisor = *supervisor_iter;
    supervisor.crashed = crashed;
    supervisor.timed_out = timed_out;
    supervisor.isolated_fault = false;

    const auto module_health = module->health();
    const bool heartbeat_timed_out = !timed_out &&
                                     (now_ms > module_health.heartbeat_timestamp_ms) &&
                                     ((now_ms - module_health.heartbeat_timestamp_ms) > supervisor.contract.heartbeat_timeout_ms);

    if (!crashed && !timed_out && !heartbeat_timed_out) {
        supervisor.last_fault.clear();
        return true;
    }

    supervisor.timed_out = timed_out || heartbeat_timed_out;
    supervisor.last_fault = crashed ? "crash" : "timeout";

    if (supervisor.restart_count >= supervisor.contract.max_restarts) {
        if (logger) {
            LOG_WARN(logger,
                     "[autonomy] module restart budget exhausted: ",
                     supervisor.contract.module_name,
                     " fault=",
                     supervisor.last_fault);
        }
        if (supervisor.contract.criticality == ProcessCriticality::Critical) {
            return false;
        }
        supervisor.isolated_fault = true;
        return true;
    }

    module->stop();
    if (!module->init() || !module->start()) {
        if (logger) {
            LOG_ERROR(logger, "[autonomy] module recovery restart failed: ", supervisor.contract.module_name);
        }
        return false;
    }
    if (logger) {
        LOG_WARN(logger,
                 "[autonomy] module recovered via restart: ",
                 supervisor.contract.module_name,
                 " fault=",
                 supervisor.last_fault);
    }
    supervisor.restart_count += 1;
    supervisor.crashed = false;
    supervisor.timed_out = false;
    supervisor.isolated_fault = true;
    return true;
}

bool AutonomyStack::dependencyHealthy(const std::string& dependency_name, uint64_t now_ms) const {
    const auto status = supervisorStatus(dependency_name);
    if (!status.has_value()) {
        return false;
    }
    if (status->crashed || status->timed_out || status->isolated_fault) {
        return false;
    }

    const auto contract_iter = std::find_if(supervisor_states_.cbegin(), supervisor_states_.cend(), [&](const auto& state) {
        return state.contract.module_name == dependency_name;
    });
    if (contract_iter == supervisor_states_.cend()) {
        return false;
    }

    if ((now_ms > status->heartbeat_timestamp_ms) &&
        ((now_ms - status->heartbeat_timestamp_ms) > contract_iter->contract.heartbeat_timeout_ms)) {
        return false;
    }
    return true;
}

void AutonomyStack::applyDegradedMode(const AutonomyStepInput& input, AutonomyStepOutput* output) {
    const auto logger = logging::GetDefaultLogger();
    output->degraded_mode = false;
    output->degraded_reason.clear();

    if (!dependencyHealthy("localization", input.now_ms) || !output->localization_estimate.valid) {
        output->degraded_mode = true;
        output->degraded_reason = kStaleLocalization;
        output->motion_decision.allow_motion = false;
        output->motion_decision.reason = kStaleLocalization;
        output->locomotion_command = LocomotionCommand{.sent = false, .target = {}, .reason = kStaleLocalization};
        if (logger) {
            LOG_WARN(logger, "[autonomy] degraded mode enabled: ", kStaleLocalization);
        }
        return;
    }

    if (!dependencyHealthy("global_planner", input.now_ms) ||
        !dependencyHealthy("local_planner", input.now_ms) ||
        !output->global_plan.has_plan ||
        !output->local_plan.has_command) {
        output->degraded_mode = true;
        output->degraded_reason = kPlannerUnavailable;
        output->motion_decision.allow_motion = false;
        output->motion_decision.reason = kPlannerUnavailable;
        output->locomotion_command = LocomotionCommand{.sent = false, .target = {}, .reason = kPlannerUnavailable};
        if (logger) {
            LOG_WARN(logger, "[autonomy] degraded mode enabled: ", kPlannerUnavailable);
        }
        return;
    }

    if (!dependencyHealthy("locomotion_interface", input.now_ms) || !output->locomotion_command.sent) {
        output->degraded_mode = true;
        output->degraded_reason = kLocomotionDispatchFailure;
        output->motion_decision.allow_motion = false;
        output->motion_decision.reason = kLocomotionDispatchFailure;
        output->locomotion_command.reason = kLocomotionDispatchFailure;
        if (logger) {
            LOG_WARN(logger, "[autonomy] degraded mode enabled: ", kLocomotionDispatchFailure);
        }
    }
}

} // namespace autonomy
