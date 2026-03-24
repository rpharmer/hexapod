#include "cli_options.hpp"

bool parseCliOptions(int argc, char** argv, CliOptions& out, std::string& error)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario") {
      if (i + 1 >= argc) {
        error = "--scenario requires a file path";
        return false;
      }
      out.scenarioFile = argv[++i];
      out.mode = ServerMode::Scenario;
    } else if (arg == "--scenario-strict") {
      out.scenarioValidationMode = ScenarioDriver::ValidationMode::Strict;
    } else if (arg == "--scenario-lint") {
      out.lintScenarioOnly = true;
      out.scenarioValidationMode = ScenarioDriver::ValidationMode::Strict;
      out.mode = ServerMode::Scenario;
    } else if ((arg == "--xbox-device" || arg == "--controller-device") && i + 1 < argc) {
      out.controllerDevice = argv[++i];
    } else if (arg == "--xbox-device" || arg == "--controller-device") {
      error = arg + " requires a device path";
      return false;
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
