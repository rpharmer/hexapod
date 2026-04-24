#include "interactive_input_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace {

double smoothstep01(const double x)
{
  const double t = std::clamp(x, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double remapRadialMagnitude(const double magnitude, const double deadzone)
{
  if (magnitude <= deadzone) {
    return 0.0;
  }
  return std::clamp((magnitude - deadzone) / std::max(1.0 - deadzone, 1e-6), 0.0, 1.0);
}

} // namespace

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

InteractiveButtonMappingResult mapInteractiveButtonEvent(const ControllerEvent& event,
                                                         InteractiveControllerState& state)
{
  InteractiveButtonMappingResult result{};
  if (event.type != ControllerEvent::Type::Button || event.value == 0) {
    return result;
  }

  if (event.name == "A") {
    state.input_mode = nextControllerInputMode(state.input_mode);
    result.info_log =
        std::string{"Controller mode switched to "} + controllerInputModeName(state.input_mode);
    return result;
  }

  if (state.input_mode == ControllerInputMode::BodyPose) {
    if (event.name == "B") {
      state.body_height_m = 0.14;
      result.info_log = "Body pose offsets reset to neutral";
    } else if (event.name == "LB") {
      state.body_height_m = std::clamp(state.body_height_m + 0.01, 0.05, 0.35);
    } else if (event.name == "RB") {
      state.body_height_m = std::clamp(state.body_height_m - 0.01, 0.05, 0.35);
    }
    return result;
  }

  if (state.input_mode == ControllerInputMode::Calibration) {
    if (event.name == "B") {
      result.calibration_action = CalibrationAction::HeightDetection;
    } else if (event.name == "X") {
      result.calibration_action = CalibrationAction::ServoFit;
    } else if (event.name == "LB") {
      result.calibration_action = CalibrationAction::ServoSpeedFit;
    } else if (event.name == "Y") {
      result.calibration_action = CalibrationAction::HeightAndServoFit;
    }
    return result;
  }

  if (event.name == "X") {
    state.gait = GaitType::RIPPLE;
  } else if (event.name == "Y") {
    state.gait = GaitType::TRIPOD;
  }

  return result;
}

void updateControllerDerivedState(const IControlDevice&, InteractiveControllerState& state)
{
  if (state.input_mode == ControllerInputMode::HeadingWalk) {
    state.walk_mode = RobotMode::WALK;
  } else {
    state.walk_mode = RobotMode::STAND;
  }
}

MotionIntent makeControllerMotionIntent(const IControlDevice& controller,
                                        InteractiveControllerState& state)
{
  MotionIntent cmd = makeMotionIntent(state.walk_mode, state.gait, state.walk_body_height_m);

  constexpr double kMaxCommandSpeedMps = 0.04;
  constexpr double kMaxBodyTranslateXYM = 0.08;
  constexpr double kMaxBodyRollPitchRad = 0.40;
  constexpr double kMaxBodyYawRad = 0.60;
  constexpr double kBodyHeightAdjustRangeM = 0.08;
  constexpr double kMinBodyHeightM = 0.05;
  constexpr double kMaxBodyHeightM = 0.35;

  const double left_x = static_cast<double>(controller.getLeftX());
  const double left_y = static_cast<double>(controller.getLeftY());
  const double right_x = static_cast<double>(controller.getRightX());
  const double right_y = static_cast<double>(controller.getRightY());
  const double lt = static_cast<double>(controller.getLeftTrigger()) / 1023.0;
  const double rt = static_cast<double>(controller.getRightTrigger()) / 1023.0;

  if (state.input_mode == ControllerInputMode::HeadingWalk) {
    constexpr double kWalkDeadzone = 0.12;
    constexpr double kMaxYawRateRadps = 0.07;
    const double left_x = static_cast<double>(controller.getLeftX());
    const double left_y = static_cast<double>(controller.getLeftY());
    const double right_x = static_cast<double>(controller.getRightX());
    const double left_mag = std::hypot(left_x, left_y);
    if (left_mag <= kWalkDeadzone) {
      state.walk_mode = RobotMode::SAFE_IDLE;
      cmd.requested_mode = RobotMode::SAFE_IDLE;
      cmd.speed_mps = LinearRateMps{0.0};
      cmd.heading_rad = AngleRad{0.0};
      cmd.cmd_vx_mps = LinearRateMps{0.0};
      cmd.cmd_vy_mps = LinearRateMps{0.0};
      cmd.cmd_yaw_radps = AngularRateRadPerSec{0.0};
      cmd.twist.twist_pos_rad = Vec3{};
      cmd.twist.twist_vel_radps = Vec3{};
      cmd.twist.body_trans_m = Vec3{0.0, 0.0, std::clamp(state.walk_body_height_m, kMinBodyHeightM, kMaxBodyHeightM)};
      cmd.twist.body_trans_mps = Vec3{};
      return cmd;
    }

    state.walk_mode = RobotMode::WALK;
    const double walk_mag = smoothstep01(remapRadialMagnitude(left_mag, kWalkDeadzone));
    const double walk_dir_x = left_x / std::max(left_mag, 1e-6);
    const double walk_dir_y = left_y / std::max(left_mag, 1e-6);
    const double vx = -walk_dir_y * kMaxCommandSpeedMps * walk_mag;
    const double vy = walk_dir_x * kMaxCommandSpeedMps * walk_mag;
    const double yaw_mag = smoothstep01(std::clamp(std::abs(right_x), 0.0, 1.0));
    cmd.speed_mps = LinearRateMps{std::hypot(vx, vy)};
    cmd.heading_rad = AngleRad{std::atan2(vy, vx)};
    cmd.cmd_vx_mps = LinearRateMps{vx};
    cmd.cmd_vy_mps = LinearRateMps{vy};
    cmd.cmd_yaw_radps = AngularRateRadPerSec{std::copysign(yaw_mag * kMaxYawRateRadps, right_x)};

    const double body_height_cmd = state.walk_body_height_m + (rt - lt) * kBodyHeightAdjustRangeM;
    cmd.twist.body_trans_m = Vec3{0.0, 0.0, std::clamp(body_height_cmd, kMinBodyHeightM, kMaxBodyHeightM)};
    cmd.twist.body_trans_mps = Vec3{};
    cmd.twist.twist_pos_rad = Vec3{};
    cmd.twist.twist_vel_radps = Vec3{};
    return cmd;
  }

  if (state.input_mode == ControllerInputMode::BodyPose) {
    cmd.speed_mps = LinearRateMps{0.0};
    cmd.heading_rad = AngleRad{0.0};
    cmd.cmd_vx_mps = LinearRateMps{0.0};
    cmd.cmd_vy_mps = LinearRateMps{0.0};
    cmd.cmd_yaw_radps = AngularRateRadPerSec{0.0};
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

  cmd.requested_mode = RobotMode::STAND;
  cmd.speed_mps = LinearRateMps{0.0};
  cmd.heading_rad = AngleRad{0.0};
  cmd.cmd_vx_mps = LinearRateMps{0.0};
  cmd.cmd_vy_mps = LinearRateMps{0.0};
  cmd.cmd_yaw_radps = AngularRateRadPerSec{0.0};
  cmd.twist.body_trans_m = Vec3{0.0, 0.0, state.walk_body_height_m};
  cmd.twist.body_trans_mps = Vec3{};
  cmd.twist.twist_pos_rad = Vec3{};
  cmd.twist.twist_vel_radps = Vec3{};

  return cmd;
}
