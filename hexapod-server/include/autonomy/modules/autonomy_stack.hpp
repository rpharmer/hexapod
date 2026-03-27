#pragma once

#include "autonomy/contract_enforcer.hpp"
#include "autonomy/mission_executive.hpp"
#include "autonomy/mission_scripting.hpp"
#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_shells.hpp"

#include "kinematics/types.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace autonomy {

enum class ProcessCriticality {
    Critical,
    SoftRealtime,
    NonCritical,
};

struct ModuleProcessContract {
    std::string module_name{};
    ProcessCriticality criticality{ProcessCriticality::NonCritical};
    uint64_t heartbeat_timeout_ms{0};
    uint64_t max_restarts{0};
    std::vector<std::string> dependencies{};
};

struct ModuleSupervisorStatus {
    std::string module_name{};
    ProcessCriticality criticality{ProcessCriticality::NonCritical};
    bool alive{true};
    bool timed_out{false};
    bool crashed{false};
    bool isolated_fault{false};
    uint64_t restart_count{0};
    uint64_t heartbeat_timestamp_ms{0};
    std::string last_fault{};
};

struct AutonomyStackConfig {
    uint64_t no_progress_timeout_ms{1000};
    uint64_t recovery_retry_budget{2};
    LocomotionInterfaceModuleShell::CommandSink locomotion_command_sink{};
};

struct ModuleFaultInjection {
    std::string module_name{};
    bool crash{false};
    bool timeout{false};
};

struct AutonomyStepInput {
    uint64_t now_ms{0};
    bool estop{false};
    bool hold{false};
    bool blocked{false};
    MapSliceInput map_slice_input{};
    bool waypoint_reached{false};

    bool has_estimator_state{false};
    RobotState estimator_state{};
    std::string localization_frame_id{"map"};

    std::vector<ModuleFaultInjection> fault_injections{};
};

struct AutonomyStepOutput {
    MissionEvent mission_event{};
    NavigationUpdate navigation_update{};
    LocalizationEstimate localization_estimate{};
    WorldModelSnapshot world_model_snapshot{};
    TraversabilityReport traversability_report{};
    GlobalPlan global_plan{};
    LocalPlan local_plan{};
    ProgressEvaluation progress_evaluation{};
    RecoveryDecision recovery_decision{};
    MotionDecision motion_decision{};
    LocomotionCommand locomotion_command{};
    bool degraded_mode{false};
    std::string degraded_reason{};
};

struct MissionScriptLoadResult {
    bool accepted{false};
    MissionEvent mission_event{};
    ContractValidationResult contract_validation{};
    MissionParseResult parse_result{};
    std::string error{};
};

class AutonomyStack {
public:
    explicit AutonomyStack(const AutonomyStackConfig& config = {});

    bool init();
    bool start();
    bool step(const AutonomyStepInput& input, AutonomyStepOutput* output);
    bool step(const AutonomyStepInput& input,
              const ContractEnvelope& envelope,
              AutonomyStepOutput* output,
              const ContractValidationConfig& config = {});
    void stop();

    MissionEvent loadMission(const WaypointMission& mission);
    MissionScriptLoadResult loadMissionScript(const std::string& script,
                                              const ContractEnvelope& envelope,
                                              uint64_t now_ms,
                                              const ContractValidationConfig& config = {});
    MissionEvent startMission();

    [[nodiscard]] std::vector<ModuleProcessContract> processContracts() const;
    [[nodiscard]] std::vector<ModuleSupervisorStatus> supervisorStatuses() const;
    [[nodiscard]] std::optional<ModuleSupervisorStatus> supervisorStatus(std::string_view module_name) const;

private:
    static constexpr size_t kModuleCount = 12;
    using ModuleArray = std::array<AutonomyModuleStub*, kModuleCount>;

    struct SupervisorState {
        ModuleProcessContract contract{};
        uint64_t restart_count{0};
        bool isolated_fault{false};
        bool crashed{false};
        bool timed_out{false};
        std::string last_fault{};
    };

    ModuleArray modules();
    ContractEnvelope makeInternalEnvelope(const std::string& stream_id,
                                          const std::string& frame_id,
                                          const std::string& correlation_id,
                                          uint64_t timestamp_ms);
    bool validateEnvelopeForStream(const ContractEnvelope& envelope,
                                   uint64_t now_ms,
                                   const ContractValidationConfig& config,
                                   const std::string& fallback_stream_id);
    [[nodiscard]] AutonomyModuleStub* moduleByName(std::string_view module_name);
    [[nodiscard]] const AutonomyModuleStub* moduleByName(std::string_view module_name) const;
    bool faultInjected(const AutonomyStepInput& input,
                       std::string_view module_name,
                       bool ModuleFaultInjection::*selector) const;
    bool monitorAndRecoverModule(AutonomyModuleStub* module,
                                 uint64_t now_ms,
                                 bool crashed,
                                 bool timed_out);
    bool dependencyHealthy(const std::string& dependency_name, uint64_t now_ms) const;
    void applyDegradedMode(const AutonomyStepInput& input, AutonomyStepOutput* output);

    MissionExecutiveModule mission_executive_module_{};
    MissionScriptingModule mission_scripting_module_{};
    NavigationManagerModuleShell navigation_manager_module_{};
    MotionArbiterModuleShell motion_arbiter_module_{};
    LocalizationModuleShell localization_module_{};
    WorldModelModuleShell world_model_module_{};
    TraversabilityAnalyzerModuleShell traversability_analyzer_module_{};
    GlobalPlannerModuleShell global_planner_module_{};
    LocalPlannerModuleShell local_planner_module_{};
    LocomotionInterfaceModuleShell locomotion_interface_module_{};
    ProgressMonitorModuleShell progress_monitor_module_;
    RecoveryManagerModuleShell recovery_manager_module_;

    MissionExecutive mission_executive_{};
    MissionScripting mission_scripting_{};
    ContractEnforcer contract_enforcer_{};
    std::unordered_map<std::string, uint64_t> last_sample_id_by_stream_{};
    uint64_t generated_sample_id_{0};
    uint64_t generated_correlation_id_{0};

    std::vector<SupervisorState> supervisor_states_{};
};

} // namespace autonomy
