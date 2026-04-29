#include "mode_runners.hpp"

#include <thread>

#include "evdev_gamepad_controller.hpp"
#include "geometry_config.hpp"
#include "geometry_profile_service.hpp"
#include "motion_intent_utils.hpp"
#include "interactive_calibration_actions.hpp"
#include "interactive_input_mapper.hpp"
#include "status_reporter.hpp"
#include "xbox_controller.hpp"

using namespace logging;

namespace {

std::chrono::milliseconds interactiveRefreshPeriod(const control_config::ControlConfig& control_cfg,
                                                   const bool controller_enabled) {
  if (!controller_enabled) {
    return control_cfg.loop_timing.command_refresh_period;
  }

  constexpr auto kMaxInteractiveRefreshPeriod = std::chrono::milliseconds{20};
  return std::min(control_cfg.loop_timing.command_refresh_period, kMaxInteractiveRefreshPeriod);
}

}  // namespace

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
  auto last_probe_log_at = std::chrono::steady_clock::time_point{};
  const auto refresh_period = interactiveRefreshPeriod(control_cfg, controller != nullptr);
  if (logger) {
    LOG_INFO(logger,
             "Interactive command refresh period ms=",
             refresh_period.count(),
             " (configured=",
             control_cfg.loop_timing.command_refresh_period.count(),
             ")");
  }

  const auto maybeLogControllerProbe = [&]() {
    if (!logger || controller == nullptr) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_probe_log_at != std::chrono::steady_clock::time_point{} &&
        now - last_probe_log_at < std::chrono::seconds(1)) {
      return;
    }
    last_probe_log_at = now;

    const MotionIntent intent = makeControllerMotionIntent(*controller, state);
    const ControlStatus status = robot.getStatus();
    const SafetyState safety = robot.getSafetyState();
    const RobotState est = robot.estimatedSnapshot();
    const auto intent_age_us =
        (intent.timestamp_us.isZero() || intent.timestamp_us.value > now_us().value)
            ? 0ULL
            : (now_us().value - intent.timestamp_us.value);
    const auto est_age_us =
        (est.timestamp_us.isZero() || est.timestamp_us.value > now_us().value)
            ? 0ULL
            : (now_us().value - est.timestamp_us.value);
    LOG_INFO(logger,
             "[probe] mode=",
             controllerInputModeName(state.input_mode),
             " walk_mode=",
             static_cast<int>(state.walk_mode),
             " gait=",
             static_cast<int>(state.gait),
             " lx=",
             controller->getLeftX(),
             " ly=",
             controller->getLeftY(),
             " rx=",
             controller->getRightX(),
             " ry=",
             controller->getRightY(),
             " cmd_vx=",
             intent.cmd_vx_mps.value,
             " cmd_vy=",
             intent.cmd_vy_mps.value,
             " cmd_yaw=",
             intent.cmd_yaw_radps.value,
             " intent_sid=",
             intent.sample_id,
             " intent_age_us=",
             intent_age_us,
             " est_sid=",
             est.sample_id,
             " est_age_us=",
             est_age_us,
             " status_mode=",
             static_cast<int>(status.active_mode),
             " status_fault=",
             static_cast<int>(status.active_fault),
             " safety_fault=",
             static_cast<int>(safety.active_fault),
             " safety_lifecycle=",
             static_cast<int>(safety.fault_lifecycle),
             " inhibit_motion=",
             safety.inhibit_motion ? 1 : 0,
             " est_valid=",
             status.estimator_valid ? 1 : 0,
             " bus_ok=",
             status.bus_ok ? 1 : 0);
  };

  const auto process_controller_events = [&](const bool allow_motion_update) {
    if (controller == nullptr) {
      return;
    }

    updateControllerDerivedState(*controller, state);
    if (allow_motion_update) {
      robot.setMotionIntent(makeControllerMotionIntent(*controller, state));
    } else {
      robot.setMotionIntent(makeMotionIntent(RobotMode::SAFE_IDLE, state.gait, state.walk_body_height_m));
    }

    bool saw_events = false;
    while (auto ev = controller->getQueue().pop()) {
      saw_events = true;
      const InteractiveButtonMappingResult event_result =
          mapInteractiveButtonEvent(*ev, state);
      if (event_result.info_log.has_value()) {
        LOG_INFO(logger, event_result.info_log.value());
      }
      executeCalibrationAction(event_result.calibration_action, logger);
    }

    if (allow_motion_update && saw_events) {
      updateControllerDerivedState(*controller, state);
      robot.setMotionIntent(makeControllerMotionIntent(*controller, state));
    }
  };

  const auto settle_deadline = std::chrono::steady_clock::now() + control_cfg.loop_timing.stand_settling_delay;
  while (!exit_flag.load() && std::chrono::steady_clock::now() < settle_deadline) {
    process_controller_events(false);
    if (controller == nullptr) {
      robot.setMotionIntent(makeMotionIntent(RobotMode::SAFE_IDLE, GaitType::TRIPOD, state.walk_body_height_m));
    }
    else {
      robot.setMotionIntent(makeMotionIntent(RobotMode::SAFE_IDLE, state.gait, state.walk_body_height_m));
    }
    maybeLogControllerProbe();
    std::this_thread::sleep_for(refresh_period);
  }

  while (!exit_flag.load()) {
    if (controller != nullptr) {
      process_controller_events(true);
      maybeLogControllerProbe();
    } else {
      // With no controller attached, keep the robot in a neutral hold instead of
      // forcing a walk cycle that can fight the estimator/fusion stack.
      robot.setMotionIntent(makeMotionIntent(RobotMode::SAFE_IDLE, state.gait, state.walk_body_height_m));
    }

    std::this_thread::sleep_for(refresh_period);
  }

  if (controller != nullptr) {
    controller->stop();
  }

  return 0;
}
