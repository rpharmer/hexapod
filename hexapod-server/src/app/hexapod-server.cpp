#include <algorithm>
#include <atomic>
#include <csignal>
#include <memory>
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

  auto logger = std::make_shared<AsyncLogger>("app", LogLevel::Debug, 10000);
  logger->AddSink(std::make_shared<ConsoleSink>());
  logger->AddSink(std::make_shared<FileSink>("app.log"));
  SetDefaultLogger(logger);

  ParsedToml config;
  if (!tomlParser("config.txt", config)) {
    return 1;
  }

  const control_config::ControlConfig control_cfg = control_config::fromParsedToml(config);
  geometry_config::loadFromParsedToml(config);

  CliOptions options;
  std::string cli_error;
  if (!parseCliOptions(argc, argv, options, cli_error)) {
    LOG_ERROR(logger, "CLI error: ", cli_error);
    return 1;
  }

  LOG_INFO(logger, "Runtime.Mode=", config.runtimeMode);

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

  robot.stop();
  LOG_WARN(logger, "All workers joined");
  LOG_ERROR(logger, "Dropped messages=", logger->DroppedMessageCount());

  logger->Flush();
  logger->Stop();

  return runner_rc;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  const TomlParser parser(GetDefaultLogger());
  return parser.parse(filename, out);
}
