#include "mode_runners.hpp"

#include <thread>

#include "evdev_gamepad_controller.hpp"
#include "geometry_config.hpp"
#include "geometry_profile_service.hpp"
#include "motion_intent_utils.hpp"
#include "interactive_calibration_actions.hpp"
#include "interactive_input_mapper.hpp"
#include "xbox_controller.hpp"

using namespace logging;

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

  InteractiveControllerState state{};

  robot.setMotionIntent(makeMotionIntent(RobotMode::STAND, state.gait, state.walk_body_height_m));
  std::this_thread::sleep_for(control_cfg.loop_timing.stand_settling_delay);

  while (!exit_flag.load()) {
    if (controller != nullptr) {
      while (auto ev = controller->getQueue().pop()) {
        const InteractiveButtonMappingResult event_result =
            mapInteractiveButtonEvent(*ev, state);
        if (event_result.info_log.has_value()) {
          LOG_INFO(logger, event_result.info_log.value());
        }
        executeCalibrationAction(event_result.calibration_action, logger);
      }

      updateControllerDerivedState(*controller, state);
      robot.setMotionIntent(makeControllerMotionIntent(*controller, state));
    } else {
      robot.setMotionIntent(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.05));
    }

    std::this_thread::sleep_for(control_cfg.loop_timing.command_refresh_period);
  }

  if (controller != nullptr) {
    controller->stop();
  }

  return 0;
}
