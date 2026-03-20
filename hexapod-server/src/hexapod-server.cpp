#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <memory>

#include "logger.hpp"
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "toml_parser.hpp"
#include "serialCommsServer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "sim_hardware_bridge.hpp"
#include "robot_control.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"
#include "scenario_driver.hpp"

using namespace mn::CppLinuxSerial;
using namespace logging;

static std::atomic<bool> g_exit{false};

namespace {

MotionIntent buildMotionIntent(RobotMode mode, GaitType gait, double body_height_m)
{
  MotionIntent cmd{};
  cmd.requested_mode = mode;
  cmd.gait = gait;
  cmd.twist.twist_pos_rad = {0.0, 0.0, 0.0};
  cmd.twist.body_trans_m = {0.00, 0.00, body_height_m};
  cmd.twist.body_trans_mps = {0.00, 0.00, 0.00};
  cmd.timestamp_us = now_us();
  return cmd;
}

std::unique_ptr<IHardwareBridge> makeHardwareBridge(const ParsedToml& config)
{
  if (config.runtimeMode == "sim") {
    SimHardwareFaultToggles sim_faults{};
    sim_faults.nominal_voltage = static_cast<float>(config.simInitialVoltageV);
    sim_faults.nominal_current = static_cast<float>(config.simInitialCurrentA);
    sim_faults.drop_bus = config.simDropBus;
    sim_faults.low_voltage = config.simLowVoltage;
    sim_faults.high_current = config.simHighCurrent;
    sim_faults.low_voltage_value = std::max(0.0f, sim_faults.nominal_voltage * 0.5f);
    sim_faults.high_current_value = std::max(sim_faults.nominal_current * 2.0f, sim_faults.nominal_current + 1.0f);

    const double read_period_s = 1.0 / std::max(config.simResponseRateHz, 1.0);
    return std::make_unique<SimHardwareBridge>(
        sim_faults, DurationSec{read_period_s}, DurationSec{0.08});
  }

  return std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate,
                                                config.timeout, config.minMaxPulses);
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

  control_config::loadFromParsedToml(config);
  geometry_config::loadFromParsedToml(config);

  LOG_INFO(logger, "Runtime.Mode=", config.runtimeMode);

  auto hw = makeHardwareBridge(config);
  auto estimator = std::make_unique<SimpleEstimator>();

  RobotControl robot(std::move(hw), std::move(estimator), logger);

  if (!robot.init()) {
    return 1;
  }

  robot.start();

  bool run_default_loop = true;
  std::string scenario_file;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario" && i + 1 < argc) {
      scenario_file = argv[++i];
      run_default_loop = false;
    }
  }

  if (!run_default_loop) {
    ScenarioDefinition scenario{};
    std::string scenario_error;
    if (!ScenarioDriver::loadFromToml(scenario_file, scenario, scenario_error)) {
      LOG_ERROR(logger, "Failed to load scenario file '", scenario_file, "': ", scenario_error);
      robot.stop();
      logger->Flush();
      logger->Stop();
      return 1;
    }

    LOG_INFO(logger, "Running scenario: ", scenario.name);
    if (!ScenarioDriver::run(robot, scenario, logger)) {
      robot.stop();
      logger->Flush();
      logger->Stop();
      return 1;
    }
  } else {
    robot.setMotionIntent(buildMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.20));

    std::this_thread::sleep_for(control_config::kStandSettlingDelay);

    while (!g_exit.load()) {
      robot.setMotionIntent(buildMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.20));

      std::this_thread::sleep_for(control_config::kCommandRefreshPeriod); // refresh command watchdog
    }
  }

  robot.stop();

  LOG_WARN(logger, "All workers joined");
  LOG_ERROR(logger, "Dropped messages=", logger->DroppedMessageCount());

  logger->Flush();
  logger->Stop();

  return 0;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  const TomlParser parser;
  return parser.parse(filename, out);
}
