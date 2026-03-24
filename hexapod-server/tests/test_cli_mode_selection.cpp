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

bool testUnknownFlagFails()
{
  std::vector<std::string> args{"hexapod-server", "--bogus"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "unknown flag should fail") &&
         expect(error.find("Unknown option") != std::string::npos,
                "unknown option error should mention Unknown option");
}

bool testPositionalArgFails()
{
  std::vector<std::string> args{"hexapod-server", "orphan-positional"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "unexpected positional arg should fail") &&
         expect(error.find("Unexpected positional argument") != std::string::npos,
                "positional arg error should mention unexpected positional argument");
}

bool testScenarioOptionRequiresNonFlagValue()
{
  std::vector<std::string> args{"hexapod-server", "--scenario", "--scenario-strict"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "scenario followed by another flag should fail") &&
         expect(error.find("--scenario") != std::string::npos,
                "scenario missing value error should mention --scenario");
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

bool testXboxDeviceOptionRequiresNonFlagValue()
{
  std::vector<std::string> args{"hexapod-server", "--xbox-device", "--scenario"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "xbox-device followed by another flag should fail") &&
         expect(error.find("--xbox-device") != std::string::npos,
                "missing xbox-device value error should mention --xbox-device");
}

bool testControllerDeviceOptionRequiresNonFlagValue()
{
  std::vector<std::string> args{"hexapod-server", "--controller-device", "--scenario"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "controller-device followed by another flag should fail") &&
         expect(error.find("--controller-device") != std::string::npos,
                "missing controller-device value error should mention --controller-device");
}

bool testScenarioOptionRequiresValueAtEndOfArgv()
{
  std::vector<std::string> args{"hexapod-server", "--scenario"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "scenario at end-of-argv should fail") &&
         expect(error.find("--scenario") != std::string::npos,
                "missing scenario value error should mention --scenario");
}

bool testXboxDeviceOptionRequiresValueAtEndOfArgv()
{
  std::vector<std::string> args{"hexapod-server", "--xbox-device"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "xbox-device at end-of-argv should fail") &&
         expect(error.find("--xbox-device") != std::string::npos,
                "missing xbox-device value error should mention --xbox-device");
}

bool testControllerDeviceOptionRequiresValueAtEndOfArgv()
{
  std::vector<std::string> args{"hexapod-server", "--controller-device"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "controller-device at end-of-argv should fail") &&
         expect(error.find("--controller-device") != std::string::npos,
                "missing controller-device value error should mention --controller-device");
}

bool testConfigOptionPersistsPath()
{
  std::vector<std::string> args{"hexapod-server", "--config", "custom-config.txt"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "config CLI should parse") &&
         expect(options.configFile == "custom-config.txt", "config path should be persisted");
}

bool testConfigOptionRequiresValue()
{
  std::vector<std::string> args{"hexapod-server", "--config", "--scenario", "scenario.toml"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "config without explicit value should fail") &&
         expect(error.find("--config") != std::string::npos,
                "missing config value error should mention --config");
}

bool testScenarioOptionRequiresExplicitValue()
{
  std::vector<std::string> args{"hexapod-server", "--scenario", "--scenario-strict"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "scenario without explicit value should fail") &&
         expect(error.find("--scenario") != std::string::npos,
                "missing scenario value error should mention --scenario");
}

bool testControllerDeviceOptionRequiresExplicitValue()
{
  std::vector<std::string> args{"hexapod-server", "--xbox-device", "--scenario-lint"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "controller-device without explicit value should fail") &&
         expect(error.find("--xbox-device") != std::string::npos,
                "missing controller-device value error should mention option name");
}

bool testConfigWithScenarioAndControllerOptions()
{
  std::vector<std::string> args{"hexapod-server",
                                "--config",
                                "custom-config.txt",
                                "--scenario",
                                "scenarios/test.toml",
                                "--xbox-device",
                                "/dev/input/js2"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "config + scenario + controller CLI should parse") &&
         expect(options.mode == ServerMode::Scenario, "scenario flag should set scenario mode") &&
         expect(options.configFile == "custom-config.txt", "config path should be persisted") &&
         expect(options.scenarioFile == "scenarios/test.toml", "scenario path should be persisted") &&
         expect(options.controllerDevice == "/dev/input/js2",
                "controller device should be persisted");
}


bool testLogFileOptionPersistsPath()
{
  std::vector<std::string> args{"hexapod-server", "--log-file", "custom-app.log"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "log-file CLI should parse") &&
         expect(options.logFilePath == "custom-app.log", "log-file path should be persisted");
}

bool testConsoleOnlyOptionSetsFlag()
{
  std::vector<std::string> args{"hexapod-server", "--console-only"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(ok, "console-only CLI should parse") &&
         expect(options.consoleOnlyLogging, "console-only flag should be persisted");
}

bool testLogFileOptionRequiresValue()
{
  std::vector<std::string> args{"hexapod-server", "--log-file", "--scenario", "scenario.toml"};
  std::vector<char*> argv = argvFrom(args);

  CliOptions options{};
  std::string error;
  const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
  return expect(!ok, "log-file without explicit value should fail") &&
         expect(error.find("--log-file") != std::string::npos,
                "missing log-file value error should mention --log-file");
}

} // namespace

int main()
{
  testDefaultInteractiveMode();
  testScenarioModeSelection();
  testScenarioLintRequiresScenarioFile();
  testUnknownFlagFails();
  testPositionalArgFails();
  testScenarioOptionRequiresNonFlagValue();
  testControllerDeviceOption();
  testXboxDeviceOptionRequiresNonFlagValue();
  testControllerDeviceOptionRequiresNonFlagValue();
  testScenarioOptionRequiresValueAtEndOfArgv();
  testXboxDeviceOptionRequiresValueAtEndOfArgv();
  testControllerDeviceOptionRequiresValueAtEndOfArgv();
  testConfigOptionPersistsPath();
  testConfigOptionRequiresValue();
  testScenarioOptionRequiresExplicitValue();
  testControllerDeviceOptionRequiresExplicitValue();
  testConfigWithScenarioAndControllerOptions();
  testLogFileOptionPersistsPath();
  testConsoleOnlyOptionSetsFlag();
  testLogFileOptionRequiresValue();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
