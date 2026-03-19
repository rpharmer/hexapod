#include <vector>
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
#include "robot_control.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"

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

} // namespace

void signalHandler(int)
{
  g_exit.store(true);
}

int main()
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

  auto hw = std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate,
                                                    config.timeout, config.minMaxPulses);
  auto estimator = std::make_unique<SimpleEstimator>();

  RobotControl robot(std::move(hw), std::move(estimator), logger);

  if (!robot.init()) {
    return 1;
  }

  robot.start();

  robot.setMotionIntent(buildMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.20));

  std::this_thread::sleep_for(control_config::kStandSettlingDelay);

  while (!g_exit.load()) {
    robot.setMotionIntent(buildMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.20));

    std::this_thread::sleep_for(control_config::kCommandRefreshPeriod); // refresh command watchdog
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
