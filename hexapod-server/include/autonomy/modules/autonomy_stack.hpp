#pragma once

#include "autonomy/mission_executive.hpp"
#include "autonomy/contract_enforcer.hpp"
#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_shells.hpp"
#include "autonomy/mission_scripting.hpp"

#include <optional>
#include <string>
#include <array>
#include <cstdint>
#include <unordered_map>

namespace autonomy {

struct AutonomyStackConfig {
    uint64_t no_progress_timeout_ms{1000};
    uint64_t recovery_retry_budget{2};
};

struct AutonomyStepInput {
    uint64_t now_ms{0};
    bool estop{false};
    bool hold{false};
    bool blocked{false};
    bool waypoint_reached{false};
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

private:
    static constexpr size_t kModuleCount = 12;
    using ModuleArray = std::array<AutonomyModuleStub*, kModuleCount>;

    ModuleArray modules();
    ContractEnvelope makeInternalEnvelope(const std::string& stream_id,
                                          const std::string& frame_id,
                                          const std::string& correlation_id,
                                          uint64_t timestamp_ms);
    bool validateEnvelopeForStream(const ContractEnvelope& envelope,
                                   uint64_t now_ms,
                                   const ContractValidationConfig& config,
                                   const std::string& fallback_stream_id);

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
};

} // namespace autonomy
