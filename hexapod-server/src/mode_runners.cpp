#include "mode_runners.hpp"

#include <algorithm>
#include <thread>

#include "control_device.hpp"
#include "evdev_gamepad_controller.hpp"
#include "motion_intent_utils.hpp"
#include "xbox_controller.hpp"

using namespace logging;

namespace {

MotionIntent makeControllerMotionIntent(const IControlDevice& controller,
                                        RobotMode mode,
                                        GaitType gait,
                                        double body_height_m)
{
  MotionIntent cmd = makeMotionIntent(mode, gait, body_height_m);

  constexpr double kMaxCommandSpeedMps = 0.25;
  constexpr double kMaxYawRad = 0.45;
  constexpr double kBodyHeightAdjustRangeM = 0.08;

  const double right_x = static_cast<double>(controller.getRightX());
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
    }
  }

  if (out.mode == ServerMode::Scenario && out.scenarioFile.empty()) {
    error = "scenario mode selected but no --scenario <file> provided";
    return false;
  }

  return true;
}

int ScenarioRunner::run(RobotControl& robot,
                        const CliOptions& options,
                        const std::shared_ptr<AsyncLogger>& logger) const
{
  ScenarioDefinition scenario{};
  std::string scenario_error;
  if (!ScenarioDriver::loadFromToml(options.scenarioFile, scenario, scenario_error,
                                    options.scenarioValidationMode)) {
    LOG_ERROR(logger, "Failed to load scenario file '", options.scenarioFile, "': ", scenario_error);
    return 1;
  }

  if (options.lintScenarioOnly) {
    LOG_INFO(logger, "Scenario lint passed: ", options.scenarioFile);
    return 0;
  }

  LOG_INFO(logger, "Running scenario: ", scenario.name);
  return ScenarioDriver::run(robot, scenario, logger) ? 0 : 1;
}

int InteractiveRunner::run(RobotControl& robot,
                           const CliOptions& options,
                           const control_config::ControlConfig& control_cfg,
                           const std::shared_ptr<AsyncLogger>& logger,
                           const std::atomic<bool>& exit_flag) const
{
  std::unique_ptr<IControlDevice> controller;
  if (!options.controllerDevice.empty()) {
    controller = std::make_unique<EvdevGamepadController>(options.controllerDevice,
                                                           makeGenericGamepadMapping());
    if (!controller->start()) {
      LOG_WARN(logger, "Controller unavailable at ", options.controllerDevice,
               ", using fallback command loop");
      controller.reset();
    } else {
      LOG_INFO(logger, "Controller enabled from device ", options.controllerDevice);
    }
  }

  RobotMode mode = RobotMode::STAND;
  GaitType gait = GaitType::TRIPOD;

  robot.setMotionIntent(makeMotionIntent(mode, gait, 0.20));
  std::this_thread::sleep_for(control_cfg.loop_timing.stand_settling_delay);

  while (!exit_flag.load()) {
    if (controller != nullptr) {
      while (auto ev = controller->getQueue().pop()) {
        if (ev->type != ControllerEvent::Type::Button || ev->value == 0) {
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

    std::this_thread::sleep_for(control_cfg.loop_timing.command_refresh_period);
  }

  if (controller != nullptr) {
    controller->stop();
  }

  return 0;
}
