#include "mode_runners.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "calibration_probe.hpp"
#include "control_device.hpp"
#include "evdev_gamepad_controller.hpp"
#include "geometry_config.hpp"
#include "geometry_profile_service.hpp"
#include "motion_intent_utils.hpp"
#include "xbox_controller.hpp"

using namespace logging;

namespace {

enum class ControllerInputMode
{
  HeadingWalk = 0,
  BodyPose = 1,
  Calibration = 2,
};

const char* controllerInputModeName(ControllerInputMode mode)
{
  switch (mode) {
    case ControllerInputMode::HeadingWalk:
      return "heading_walk";
    case ControllerInputMode::BodyPose:
      return "body_pose";
    case ControllerInputMode::Calibration:
      return "calibration";
    default:
      return "unknown";
  }
}

ControllerInputMode nextControllerInputMode(ControllerInputMode mode)
{
  switch (mode) {
    case ControllerInputMode::HeadingWalk:
      return ControllerInputMode::BodyPose;
    case ControllerInputMode::BodyPose:
      return ControllerInputMode::Calibration;
    case ControllerInputMode::Calibration:
      return ControllerInputMode::HeadingWalk;
    default:
      return ControllerInputMode::HeadingWalk;
  }
}

struct InteractiveControllerState
{
  ControllerInputMode input_mode{ControllerInputMode::HeadingWalk};
  RobotMode walk_mode{RobotMode::WALK};
  GaitType gait{GaitType::TRIPOD};
  double walk_body_height_m{0.20};
  double walk_facing_yaw_rad{0.0};
  bool walk_facing_valid{false};

  double body_height_m{0.20};
};

bool runHeightDetectionProbe(const std::shared_ptr<AsyncLogger>& logger)
{
  const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
  const std::vector<BaseClearanceSample> samples{};
  const BaseClearanceEstimateResult result =
      estimateToBottomFromSynchronousLift(geometry, samples);
  if (!result.success) {
    LOG_WARN(logger,
             "calibration.height_detection failed: no lift transition detected. "
             "Collect probe samples before running.");
    return false;
  }

  LOG_INFO(logger, "calibration.height_detection success: to_bottom_m=",
           result.estimated_to_bottom_m.value,
           " ground_plane_height_m=", result.ground_plane_height_m);
  return true;
}

bool runServoCalibrationProbe(const std::shared_ptr<AsyncLogger>& logger)
{
  const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
  const LegGeometry& leg = geometry.legGeometry[0];
  const std::vector<CalibrationTouchSample> samples{};
  const CalibrationLegFitResult result = fitServoCalibrationFromTouches(leg, samples);
  if (!result.success) {
    LOG_WARN(logger,
             "calibration.servo_fit failed: insufficient contact samples. "
             "Collect touch samples before running.");
    return false;
  }

  LOG_INFO(logger, "calibration.servo_fit success: delta_rad=[",
           result.coxa_delta.value, ", ", result.femur_delta.value, ", ",
           result.tibia_delta.value, "]");
  return true;
}

bool runServoSpeedCalibrationProbe(const std::shared_ptr<AsyncLogger>& logger)
{
  const std::vector<ServoDynamicsSample> samples{};
  const LegServoDynamicsFitResult result = fitLegServoDynamicsFromSamples(samples);

  bool fit_success = true;
  for (int joint = 0; joint < kJointsPerLeg; ++joint) {
    fit_success = fit_success && result.positive_direction[joint].success &&
                  result.negative_direction[joint].success;
  }

  if (!fit_success) {
    LOG_WARN(logger,
             "calibration.servo_speed_fit failed: insufficient dynamics samples. "
             "Collect command/measurement time-series before running.");
    return false;
  }

  for (int joint = 0; joint < kJointsPerLeg; ++joint) {
    LOG_INFO(logger, "calibration.servo_speed_fit success: joint=", joint,
             " +tau_s=", result.positive_direction[joint].tau_s,
             " +vmax_radps=", result.positive_direction[joint].vmax_radps,
             " -tau_s=", result.negative_direction[joint].tau_s,
             " -vmax_radps=", result.negative_direction[joint].vmax_radps);
  }

  HexapodGeometry preview_geometry = geometry_config::activeHexapodGeometry();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
      auto& joint_dynamics = preview_geometry.legGeometry[leg].servoDynamics[joint];
      joint_dynamics.positive_direction.tau_s = result.positive_direction[joint].tau_s;
      joint_dynamics.positive_direction.vmax_radps = result.positive_direction[joint].vmax_radps;
      joint_dynamics.negative_direction.tau_s = result.negative_direction[joint].tau_s;
      joint_dynamics.negative_direction.vmax_radps = result.negative_direction[joint].vmax_radps;
    }
  }

  std::string error;
  if (!geometry_profile_service::preview(preview_geometry, &error) ||
      !geometry_profile_service::apply(&error)) {
    LOG_WARN(logger, "calibration.servo_speed_fit failed to apply geometry profile: ",
             error);
    return false;
  }

  LOG_INFO(logger, "calibration.servo_speed_fit applied to active geometry dynamics profile");
  return true;
}

MotionIntent makeControllerMotionIntent(const IControlDevice& controller,
                                        const InteractiveControllerState& state)
{
  MotionIntent cmd = makeMotionIntent(state.walk_mode, state.gait, state.walk_body_height_m);

  constexpr double kMaxCommandSpeedMps = 0.25;
  constexpr double kMaxWalkYawRad = 0.90;
  constexpr double kMaxBodyTranslateXYM = 0.08;
  constexpr double kMaxBodyRollPitchRad = 0.40;
  constexpr double kMaxBodyYawRad = 0.60;
  constexpr double kBodyHeightAdjustRangeM = 0.08;
  constexpr double kMinBodyHeightM = 0.10;
  constexpr double kMaxBodyHeightM = 0.35;

  const double left_x = static_cast<double>(controller.getLeftX());
  const double left_y = static_cast<double>(controller.getLeftY());
  const double right_mag = static_cast<double>(controller.getRightMag());
  const double right_ang = static_cast<double>(controller.getRightAng());
  const double right_x = static_cast<double>(controller.getRightX());
  const double right_y = static_cast<double>(controller.getRightY());
  const double lt = static_cast<double>(controller.getLeftTrigger()) / 1023.0;
  const double rt = static_cast<double>(controller.getRightTrigger()) / 1023.0;

  if (state.input_mode == ControllerInputMode::HeadingWalk) {
    cmd.speed_mps = LinearRateMps{controller.getLeftMag() * kMaxCommandSpeedMps};
    cmd.heading_rad = AngleRad{static_cast<double>(controller.getLeftAng())};

    const double body_height_cmd = state.walk_body_height_m + (rt - lt) * kBodyHeightAdjustRangeM;
    cmd.twist.body_trans_m = Vec3{0.0, 0.0, std::clamp(body_height_cmd, kMinBodyHeightM, kMaxBodyHeightM)};
    cmd.twist.body_trans_mps = Vec3{};
    const double facing_yaw_rad =
        state.walk_facing_valid ? state.walk_facing_yaw_rad : (right_x * kMaxWalkYawRad);
    cmd.twist.twist_pos_rad = Vec3{0.0, 0.0, facing_yaw_rad};
    cmd.twist.twist_vel_radps = Vec3{};
    return cmd;
  }

  if (state.input_mode == ControllerInputMode::BodyPose) {
    cmd.speed_mps = LinearRateMps{0.0};
    cmd.heading_rad = AngleRad{0.0};
    cmd.twist.body_trans_m = Vec3{
        -left_y * kMaxBodyTranslateXYM,
        left_x * kMaxBodyTranslateXYM,
        std::clamp(state.body_height_m, kMinBodyHeightM, kMaxBodyHeightM)};
    cmd.twist.body_trans_mps = Vec3{};
    cmd.twist.twist_pos_rad = Vec3{
        right_x * kMaxBodyRollPitchRad,
        -right_y * kMaxBodyRollPitchRad,
        std::clamp(rt - lt, -1.0, 1.0) * kMaxBodyYawRad};
    cmd.twist.twist_vel_radps = Vec3{};
    return cmd;
  }

  (void)right_mag;
  (void)right_ang;
  cmd.requested_mode = RobotMode::STAND;
  cmd.speed_mps = LinearRateMps{0.0};
  cmd.heading_rad = AngleRad{0.0};
  cmd.twist.body_trans_m = Vec3{0.0, 0.0, state.walk_body_height_m};
  cmd.twist.body_trans_mps = Vec3{};
  cmd.twist.twist_pos_rad = Vec3{};
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

  InteractiveControllerState state{};

  robot.setMotionIntent(makeMotionIntent(RobotMode::STAND, state.gait, state.walk_body_height_m));
  std::this_thread::sleep_for(control_cfg.loop_timing.stand_settling_delay);

  while (!exit_flag.load()) {
    if (controller != nullptr) {
      while (auto ev = controller->getQueue().pop()) {
        if (ev->type != ControllerEvent::Type::Button || ev->value == 0) {
          continue;
        }

        if (ev->name == "A") {
          state.input_mode = nextControllerInputMode(state.input_mode);
          LOG_INFO(logger, "Controller mode switched to ",
                   controllerInputModeName(state.input_mode));
          continue;
        }

        if (state.input_mode == ControllerInputMode::BodyPose) {
          if (ev->name == "B") {
            state.body_height_m = 0.20;
            LOG_INFO(logger, "Body pose offsets reset to neutral");
          } else if (ev->name == "LB") {
            state.body_height_m = std::clamp(state.body_height_m + 0.01, 0.10, 0.35);
          } else if (ev->name == "RB") {
            state.body_height_m = std::clamp(state.body_height_m - 0.01, 0.10, 0.35);
          }
          continue;
        }

        if (state.input_mode == ControllerInputMode::Calibration) {
          if (ev->name == "B") {
            runHeightDetectionProbe(logger);
          } else if (ev->name == "X") {
            runServoCalibrationProbe(logger);
          } else if (ev->name == "LB") {
            runServoSpeedCalibrationProbe(logger);
          } else if (ev->name == "Y") {
            runHeightDetectionProbe(logger);
            runServoCalibrationProbe(logger);
          }
          continue;
        }

        if (ev->name == "X") {
          state.gait = GaitType::RIPPLE;
        } else if (ev->name == "Y") {
          state.gait = GaitType::TRIPOD;
        }
      }

      if (state.input_mode == ControllerInputMode::HeadingWalk) {
        if (controller->getRightMag() > 0.20f) {
          state.walk_facing_yaw_rad = static_cast<double>(controller->getRightAng());
          state.walk_facing_valid = true;
        }
        state.walk_mode = RobotMode::WALK;
      } else {
        state.walk_mode = RobotMode::STAND;
      }

      robot.setMotionIntent(makeControllerMotionIntent(*controller, state));
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
