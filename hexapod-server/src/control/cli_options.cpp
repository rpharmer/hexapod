#include "cli_options.hpp"

bool parseCliOptions(int argc, char** argv, CliOptions& out, std::string& error)
{
  const auto consumeRequiredValue = [&](int& index, const std::string& option, const std::string& reason) -> const char* {
    if (index + 1 >= argc || std::string(argv[index + 1]).rfind("--", 0) == 0) {
      error = option + " requires " + reason;
      return nullptr;
    }
    return argv[++index];
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
