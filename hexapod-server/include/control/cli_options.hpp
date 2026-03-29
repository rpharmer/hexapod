#ifndef CLI_OPTIONS_HPP
#define CLI_OPTIONS_HPP

#include <optional>
#include <string>

#include "runtime_stage_approval.hpp"
#include "scenario_driver.hpp"

enum class ServerMode
{
  Interactive,
  Scenario,
};

struct CliOptions
{
  ServerMode mode{ServerMode::Interactive};
  ScenarioDriver::ValidationMode scenarioValidationMode{ScenarioDriver::ValidationMode::Permissive};
  bool lintScenarioOnly{false};
  std::string configFile{"config.txt"};
  std::string scenarioFile;
  std::string controllerDevice;
  std::string logFilePath;
  bool consoleOnlyLogging{false};
  std::optional<bool> telemetryEnabledOverride;
  std::optional<std::string> telemetryHostOverride;
  std::optional<int> telemetryPortOverride;
  std::optional<double> telemetryPublishRateHzOverride;
  std::optional<double> telemetryGeometryResendIntervalSecOverride;
  std::optional<RuntimeStageToggles> runtimeStageTogglesOverride;
  int runtimeStageApproveNextCount{0};
};

bool parseCliOptions(int argc, char** argv, CliOptions& out, std::string& error);

#endif // CLI_OPTIONS_HPP
