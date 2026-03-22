#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "control_config.hpp"
#include "estimator.hpp"
#include "geometry_config.hpp"
#include "hardware_bridge.hpp"
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "logger.hpp"
#include "motion_intent_utils.hpp"
#include "robot_control.hpp"
#include "scenario_driver.hpp"
#include "serialCommsServer.hpp"
#include "sim_hardware_bridge.hpp"
#include "toml_parser.hpp"
#include "xbox_controller.hpp"

using namespace mn::CppLinuxSerial;
using namespace logging;

static std::atomic<bool> g_exit{false};

namespace {

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

MotionIntent makeControllerMotionIntent(const XboxController& controller,
                                        RobotMode mode,
                                        GaitType gait,
                                        double body_height_m)
{
  MotionIntent cmd = makeMotionIntent(mode, gait, body_height_m);

  constexpr double kMaxCommandSpeedMps = 0.25;
  constexpr double kMaxYawRad = 0.45;
  constexpr double kBodyHeightAdjustRangeM = 0.08;

  const double right_x = static_cast<double>(controller.getRightX()) / 32767.0;
  const double lt = static_cast<double>(controller.getLeftTrigger()) / 1023.0;
  const double rt = static_cast<double>(controller.getRightTrigger()) / 1023.0;

  cmd.speed_mps = LinearRateMps{controller.getLeftMag() * kMaxCommandSpeedMps};
  cmd.heading_rad = AngleRad{static_cast<double>(controller.getLeftAng())};

  const double body_height_cmd = body_height_m + (rt - lt) * kBodyHeightAdjustRangeM;
  cmd.twist.body_trans_m = Vec3{0.0, 0.0, std::clamp(body_height_cmd, 0.10, 0.35)};
  cmd.twist.body_trans_mps = Vec3{};
  cmd.twist.twist_pos_rad = Vec3{0.0, 0.0, right_x * kMaxYawRad};
  cmd.twist.twist_vel_radps = Vec3{};

  return cmd;
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

  LOG_INFO(logger, "Runtime.Mode=", config.runtimeMode);

  auto hw = makeHardwareBridge(config);
  auto estimator = std::make_unique<SimpleEstimator>();

  RobotControl robot(std::move(hw), std::move(estimator), logger, control_cfg);

  if (!robot.init()) {
    return 1;
  }

  bool run_default_loop = true;
  bool lint_scenario_only = false;
  ScenarioDriver::ValidationMode scenario_validation_mode = ScenarioDriver::ValidationMode::Permissive;
  std::string scenario_file;
  std::string xbox_device;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario" && i + 1 < argc) {
      scenario_file = argv[++i];
      run_default_loop = false;
    } else if (arg == "--scenario-strict") {
      scenario_validation_mode = ScenarioDriver::ValidationMode::Strict;
    } else if (arg == "--scenario-lint") {
      lint_scenario_only = true;
      scenario_validation_mode = ScenarioDriver::ValidationMode::Strict;
    } else if (arg == "--xbox-device" && i + 1 < argc) {
      xbox_device = argv[++i];
    }
  }

  robot.start();

  if (!run_default_loop) {
    ScenarioDefinition scenario{};
    std::string scenario_error;
    if (!ScenarioDriver::loadFromToml(scenario_file, scenario, scenario_error, scenario_validation_mode)) {
      LOG_ERROR(logger, "Failed to load scenario file '", scenario_file, "': ", scenario_error);
      robot.stop();
      logger->Flush();
      logger->Stop();
      return 1;
    }

    if (lint_scenario_only) {
      LOG_INFO(logger, "Scenario lint passed: ", scenario_file);
      robot.stop();
      logger->Flush();
      logger->Stop();
      return 0;
    }

    LOG_INFO(logger, "Running scenario: ", scenario.name);
    if (!ScenarioDriver::run(robot, scenario, logger)) {
      robot.stop();
      logger->Flush();
      logger->Stop();
      return 1;
    }
  } else {
    std::optional<XboxController> controller;
    if (!xbox_device.empty()) {
      controller.emplace(xbox_device);
      if (!controller->start()) {
        LOG_WARN(logger, "Xbox controller unavailable at ", xbox_device, ", using fallback command loop");
        controller.reset();
      } else {
        LOG_INFO(logger, "Xbox controller enabled from device ", xbox_device);
      }
    }

    RobotMode mode = RobotMode::STAND;
    GaitType gait = GaitType::TRIPOD;

    robot.setMotionIntent(makeMotionIntent(mode, gait, 0.20));
    std::this_thread::sleep_for(control_cfg.loop_timing.stand_settling_delay);

    while (!g_exit.load()) {
      if (controller.has_value()) {
        while (auto ev = controller->getQueue().pop()) {
          if (ev->type != XboxEvent::Type::Button || ev->value == 0) {
            continue;
          }
          if (ev->name == "A") {
            mode = RobotMode::WALK;
          } else if (ev->name == "B") {
            mode = RobotMode::STAND;
          } else if (ev->name == "X") {
            gait = GaitType::RIPPLE;
          } else if (ev->name == "Y") {
            gait = GaitType::TRIPOD;
          }
        }

        robot.setMotionIntent(makeControllerMotionIntent(*controller, mode, gait, 0.20));
      } else {
        robot.setMotionIntent(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.20));
      }

      std::this_thread::sleep_for(control_cfg.loop_timing.command_refresh_period); // refresh command watchdog
    }

    if (controller.has_value()) {
      controller->stop();
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
