#include "mode_runners.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

bool expect(bool cond, const std::string& message)
{
  if (!cond) {
    std::cerr << "[FAIL] " << message << '\n';
    ++g_failures;
    return false;
  }
  return true;
}

std::vector<char*> argvFrom(std::vector<std::string>& args)
{
  std::vector<char*> argv;
  argv.reserve(args.size());
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }
  return argv;
}

bool testDefaultInteractiveMode()
{
  std::vector<std::string> args{"hexapod-server"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "default CLI should parse") &&
         expect(options.mode == ServerMode::Interactive, "default mode should be interactive");
}

bool testScenarioModeSelection()
{
  std::vector<std::string> args{"hexapod-server", "--scenario", "scenarios/test.toml", "--scenario-strict"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "scenario CLI should parse") &&
         expect(options.mode == ServerMode::Scenario, "mode should switch to scenario") &&
         expect(options.scenarioValidationMode == ScenarioDriver::ValidationMode::Strict,
                "strict flag should select strict scenario validation");
}

bool testScenarioLintRequiresScenarioFile()
{
  std::vector<std::string> args{"hexapod-server", "--scenario-lint"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "scenario lint without file should fail") &&
         expect(error.find("--scenario") != std::string::npos,
                "missing scenario error should mention --scenario");
}


bool testControllerDeviceOption()
{
  std::vector<std::string> args{"hexapod-server", "--controller-device", "/dev/input/js0"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "controller-device CLI should parse") &&
         expect(options.mode == ServerMode::Interactive,
                "controller-device alone should keep interactive mode") &&
         expect(options.controllerDevice == "/dev/input/js0",
                "controller-device should be persisted");
}

bool testUnknownFlagFailsParsing()
{
  std::vector<std::string> args{"hexapod-server", "--does-not-exist"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "unknown option should fail parsing") &&
         expect(error.find("--does-not-exist") != std::string::npos,
                "unknown option error should include the token");
}

bool testUnexpectedPositionalArgumentFailsParsing()
{
  std::vector<std::string> args{"hexapod-server", "stray-token"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "unexpected positional argument should fail parsing") &&
         expect(error.find("stray-token") != std::string::npos,
                "positional argument error should include the token");
}

} // namespace

int main()
{
  testDefaultInteractiveMode();
  testScenarioModeSelection();
  testScenarioLintRequiresScenarioFile();
  testControllerDeviceOption();
  testUnknownFlagFailsParsing();
  testUnexpectedPositionalArgumentFailsParsing();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
