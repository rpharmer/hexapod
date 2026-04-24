#include "cli_options.hpp"

#include <cstdlib>

bool parseCliOptions(int argc, char** argv, CliOptions& out, std::string& error)
{
  const auto consumeRequiredValue = [&](int& index, const std::string& option, const std::string& reason) -> const char* {
    if (index + 1 >= argc || std::string(argv[index + 1]).rfind("--", 0) == 0) {
      error = option + " requires " + reason;
      return nullptr;
    }
    return argv[++index];
  };
  const auto consumeDoubleValue = [&](int& index,
                                      const std::string& option,
                                      const std::string& reason,
                                      double& parsed) -> bool {
    const char* value = consumeRequiredValue(index, option, reason);
    if (value == nullptr) {
      return false;
    }
    char* end = nullptr;
    const double converted = std::strtod(value, &end);
    if (end == value || (end != nullptr && *end != '\0')) {
      error = option + " requires " + reason;
      return false;
    }
    parsed = converted;
    return true;
  };

  const auto consumeIntValue = [&](int& index,
                                   const std::string& option,
                                   const std::string& reason,
                                   int& parsed) -> bool {
    double as_double = 0.0;
    if (!consumeDoubleValue(index, option, reason, as_double)) {
      return false;
    }
    if (static_cast<int>(as_double) != as_double) {
      error = option + " requires " + reason;
      return false;
    }
    parsed = static_cast<int>(as_double);
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config") {
      const char* value = consumeRequiredValue(i, arg, "a file path");
      if (value == nullptr) {
        return false;
      }
      out.configFile = value;
    } else if (arg == "--scenario") {
      const char* value = consumeRequiredValue(i, arg, "a file path");
      if (value == nullptr) {
        return false;
      }
      out.scenarioFile = value;
      out.mode = ServerMode::Scenario;
    } else if (arg == "--scenario-strict") {
      out.scenarioValidationMode = ScenarioDriver::ValidationMode::Strict;
    } else if (arg == "--scenario-lint") {
      out.lintScenarioOnly = true;
      out.scenarioValidationMode = ScenarioDriver::ValidationMode::Strict;
      out.mode = ServerMode::Scenario;
    } else if (arg == "--xbox-device" || arg == "--controller-device") {
      const char* value = consumeRequiredValue(i, arg, "a device path");
      if (value == nullptr) {
        return false;
      }
      out.controllerDevice = value;
    } else if (arg == "--log-file") {
      if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0) {
        error = "--log-file requires a file path";
        return false;
      }
      out.logFilePath = argv[++i];
    } else if (arg == "--console-only") {
      out.consoleOnlyLogging = true;
    } else if (arg == "--telemetry-enable") {
      out.telemetryEnabledOverride = true;
    } else if (arg == "--telemetry-disable") {
      out.telemetryEnabledOverride = false;
    } else if (arg == "--telemetry-host") {
      const char* value = consumeRequiredValue(i, arg, "a host");
      if (value == nullptr) {
        return false;
      }
      out.telemetryHostOverride = std::string(value);
    } else if (arg == "--telemetry-port") {
      int value = 0;
      if (!consumeIntValue(i, arg, "an integer port", value)) {
        return false;
      }
      out.telemetryPortOverride = value;
    } else if (arg == "--telemetry-publish-hz") {
      double value = 0.0;
      if (!consumeDoubleValue(i, arg, "a numeric Hz value", value)) {
        return false;
      }
      out.telemetryPublishRateHzOverride = value;
    } else if (arg == "--telemetry-geometry-resend-sec") {
      double value = 0.0;
      if (!consumeDoubleValue(i, arg, "a numeric seconds value", value)) {
        return false;
      }
      out.telemetryGeometryResendIntervalSecOverride = value;
    } else if (arg == "--investigation-disable-terrain-stance-bias") {
      out.investigationDisableTerrainStanceBiasOverride = true;
    } else if (arg == "--investigation-disable-terrain-swing-clearance") {
      out.investigationDisableTerrainSwingClearanceOverride = true;
    } else if (arg == "--investigation-disable-terrain-swing-xy-nudge") {
      out.investigationDisableTerrainSwingXYNudgeOverride = true;
    } else if (arg == "--investigation-disable-stance-tilt-leveling") {
      out.investigationDisableStanceTiltLevelingOverride = true;
    } else if (arg == "--investigation-suppress-fusion-corrections") {
      out.investigationSuppressFusionCorrectionsOverride = true;
    } else if (arg == "--investigation-suppress-fusion-resets") {
      out.investigationSuppressFusionResetsOverride = true;
    } else {
      if (arg.rfind("--", 0) == 0) {
        error = "Unknown option: " + arg;
      } else {
        error = "Unexpected positional argument: " + arg;
      }
      return false;
    }
  }

  if (out.mode == ServerMode::Scenario && out.scenarioFile.empty()) {
    error = "scenario mode selected but no --scenario <file> provided";
    return false;
  }

  return true;
}
