#include <algorithm>
#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "control_config.hpp"
#include "estimator.hpp"
#include "geometry_config.hpp"
#include "hardware_bridge.hpp"
#include "hexapod-server.hpp"
#include "logger.hpp"
#include "mode_runners.hpp"
#include "robot_control.hpp"
#include "sim_hardware_bridge.hpp"
#include "toml_parser.hpp"

using namespace logging;

static std::atomic<bool> g_exit{false};

namespace {

std::shared_ptr<AsyncLogger> makeLogger(bool enableFileLogging,
                                        const std::string& logFilePath,
                                        const std::shared_ptr<AsyncLogger>& fallbackLogger = nullptr)
{
  auto logger = std::make_shared<AsyncLogger>("app", LogLevel::Debug, 10000);
  logger->AddSink(std::make_shared<ConsoleSink>());
  if (enableFileLogging) {
    try {
      logger->AddSink(std::make_shared<FileSink>(logFilePath));
    } catch (const std::exception& ex) {
      if (fallbackLogger) {
        LOG_WARN(fallbackLogger,
                 "Failed to initialize file logging at '",
                 logFilePath,
                 "'. Falling back to console-only logging: ",
                 ex.what());
      }
    }
  }
  return logger;
}

std::unique_ptr<IHardwareBridge> makeHardwareBridge(const ParsedToml& config,
                                                    const std::shared_ptr<AsyncLogger>& logger)
{
  if (config.runtimeMode == "sim") {
    SimHardwareFaultToggles sim_faults{};
    sim_faults.nominal_voltage = static_cast<float>(config.simInitialVoltageV);
    sim_faults.nominal_current = static_cast<float>(config.simInitialCurrentA);
    sim_faults.drop_bus = config.simDropBus;
    sim_faults.low_voltage = config.simLowVoltage;
    sim_faults.high_current = config.simHighCurrent;
    sim_faults.low_voltage_value = std::max(0.0f, sim_faults.nominal_voltage * 0.5f);
    sim_faults.high_current_value =
        std::max(sim_faults.nominal_current * 2.0f, sim_faults.nominal_current + 1.0f);

    const double read_period_s = 1.0 / std::max(config.simResponseRateHz, 1.0);
    return std::make_unique<SimHardwareBridge>(sim_faults, DurationSec{read_period_s},
                                               DurationSec{0.08});
  }

  return std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate,
                                                config.timeout, config.minMaxPulses, logger);
}

} // namespace

void signalHandler(int)
{
  g_exit.store(true);
}

int main(int argc, char** argv)
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  CliOptions options;
  std::string cli_error;
  if (!parseCliOptions(argc, argv, options, cli_error)) {
    std::cerr << "CLI error: " << cli_error << '\n';
    return 1;
  }

  auto bootstrapLogger = makeLogger(false, "");
  SetDefaultLogger(bootstrapLogger);

  const std::string effectiveConfigPath =
      options.configFile.empty() ? std::string("config.txt") : options.configFile;

  ParsedToml config;
  if (!tomlParser(effectiveConfigPath, config)) {
    bootstrapLogger->Flush();
    bootstrapLogger->Stop();
    return 1;
  }

  const std::string effectiveLogPath =
      options.logFilePath.empty() ? config.logFilePath : options.logFilePath;
  const bool enableFileLogging = config.logToFile && !options.consoleOnlyLogging;

  auto logger = makeLogger(enableFileLogging, effectiveLogPath, bootstrapLogger);
  SetDefaultLogger(logger);
  bootstrapLogger->Flush();
  bootstrapLogger->Stop();

  const control_config::ControlConfig control_cfg = control_config::fromParsedToml(config);
  geometry_config::loadFromParsedToml(config);

  LOG_INFO(logger, "Runtime.Mode=", config.runtimeMode);
  LOG_INFO(logger, "Logging.FileEnabled=", enableFileLogging, ", Path=", effectiveLogPath);

  auto hw = makeHardwareBridge(config, logger);
  auto estimator = std::make_unique<SimpleEstimator>();
  RobotControl robot(std::move(hw), std::move(estimator), logger, control_cfg);

  if (!robot.init()) {
    return 1;
  }

  robot.start();

  int runner_rc = 0;
  if (options.mode == ServerMode::Scenario) {
    const ScenarioRunner runner;
    runner_rc = runner.run(robot, options, logger);
  } else {
    const InteractiveRunner runner;
    runner_rc = runner.run(robot, options, control_cfg, logger, g_exit);
  }

  bool teardown_ok = true;
  try {
    robot.stop();
  } catch (const std::exception& ex) {
    teardown_ok = false;
    LOG_ERROR(logger, "Robot teardown failed: ", ex.what());
  } catch (...) {
    teardown_ok = false;
    LOG_ERROR(logger, "Robot teardown failed: unknown exception");
  }

  if (teardown_ok) {
    LOG_INFO(logger, "All workers joined");
  } else {
    LOG_ERROR(logger, "All workers joined: false");
  }

  const std::size_t dropped_messages = logger->DroppedMessageCount();
  if (dropped_messages > 0) {
    LOG_ERROR(logger, "Dropped messages=", dropped_messages);
  } else {
    LOG_INFO(logger, "Dropped messages=", dropped_messages);
  }

  const bool shutdown_success = teardown_ok && dropped_messages == 0;
  const LogLevel shutdown_summary_level = shutdown_success ? LogLevel::Info : LogLevel::Error;
  logger->LogStream(shutdown_summary_level,
                    LOG_SOURCE_LOCATION,
                    "shutdown_summary"
                    " success=",
                    shutdown_success ? 1 : 0,
                    " teardown_ok=",
                    teardown_ok ? 1 : 0,
                    " dropped_messages=",
                    dropped_messages,
                    " runner_rc=",
                    runner_rc);

  logger->Flush();
  logger->Stop();

  return runner_rc;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  const TomlParser parser(GetDefaultLogger());
  return parser.parse(filename, out);
}
