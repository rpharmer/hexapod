#ifndef CLI_OPTIONS_HPP
#define CLI_OPTIONS_HPP

#include <string>

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
  std::string scenarioFile;
  std::string controllerDevice;
};

bool parseCliOptions(int argc, char** argv, CliOptions& out, std::string& error);

#endif // CLI_OPTIONS_HPP
