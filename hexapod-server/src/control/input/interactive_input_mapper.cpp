#include "interactive_input_mapper.hpp"

#include <algorithm>

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
      state.body_height_m = 0.20;
      result.info_log = "Body pose offsets reset to neutral";
    } else if (event.name == "LB") {
      state.body_height_m = std::clamp(state.body_height_m + 0.01, 0.10, 0.35);
    } else if (event.name == "RB") {
      state.body_height_m = std::clamp(state.body_height_m - 0.01, 0.10, 0.35);
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

void updateControllerDerivedState(const IControlDevice& controller, InteractiveControllerState& state)
{
  if (state.input_mode == ControllerInputMode::HeadingWalk) {
    if (controller.getRightMag() > 0.20f) {
      state.walk_facing_yaw_rad = static_cast<double>(controller.getRightAng());
      state.walk_facing_valid = true;
    }
    state.walk_mode = RobotMode::WALK;
  } else {
    state.walk_mode = RobotMode::STAND;
  }
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
  const double right_x = static_cast<double>(controller.getRightX());
  const double right_y = static_cast<double>(controller.getRightY());
  const double lt = static_cast<double>(controller.getLeftTrigger()) / 1023.0;
  const double rt = static_cast<double>(controller.getRightTrigger()) / 1023.0;

  if (state.input_mode == ControllerInputMode::HeadingWalk) {
    cmd.speed_mps = LinearRateMps{controller.getLeftMag() * kMaxCommandSpeedMps};
    cmd.heading_rad = AngleRad{static_cast<double>(controller.getLeftAng())};

    const double body_height_cmd = state.walk_body_height_m + (rt - lt) * kBodyHeightAdjustRangeM;
    cmd.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, std::clamp(body_height_cmd, kMinBodyHeightM, kMaxBodyHeightM)};
    cmd.body_pose_setpoint.body_trans_mps = Vec3{};
    const double facing_yaw_rad =
        state.walk_facing_valid ? state.walk_facing_yaw_rad : (right_x * kMaxWalkYawRad);
    cmd.body_pose_setpoint.orientation_rad = Vec3{0.0, 0.0, facing_yaw_rad};
    cmd.body_pose_setpoint.angular_velocity_radps = Vec3{};
    return cmd;
  }

  if (state.input_mode == ControllerInputMode::BodyPose) {
    cmd.speed_mps = LinearRateMps{0.0};
    cmd.heading_rad = AngleRad{0.0};
    cmd.body_pose_setpoint.body_trans_m = Vec3{
        -left_y * kMaxBodyTranslateXYM,
        left_x * kMaxBodyTranslateXYM,
        std::clamp(state.body_height_m, kMinBodyHeightM, kMaxBodyHeightM)};
    cmd.body_pose_setpoint.body_trans_mps = Vec3{};
    cmd.body_pose_setpoint.orientation_rad = Vec3{
        right_x * kMaxBodyRollPitchRad,
        -right_y * kMaxBodyRollPitchRad,
        std::clamp(rt - lt, -1.0, 1.0) * kMaxBodyYawRad};
    cmd.body_pose_setpoint.angular_velocity_radps = Vec3{};
    return cmd;
  }

  cmd.requested_mode = RobotMode::STAND;
  cmd.speed_mps = LinearRateMps{0.0};
  cmd.heading_rad = AngleRad{0.0};
  cmd.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, state.walk_body_height_m};
  cmd.body_pose_setpoint.body_trans_mps = Vec3{};
  cmd.body_pose_setpoint.orientation_rad = Vec3{};
  cmd.body_pose_setpoint.angular_velocity_radps = Vec3{};

  return cmd;
}
