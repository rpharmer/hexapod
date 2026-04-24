#ifndef INTERACTIVE_INPUT_MAPPER_HPP
#define INTERACTIVE_INPUT_MAPPER_HPP

#include <optional>
#include <string>

#include "control_device.hpp"
#include "motion_intent_utils.hpp"

enum class ControllerInputMode
{
  HeadingWalk = 0,
  BodyPose = 1,
  Calibration = 2,
};

enum class CalibrationAction
{
  None,
  HeightDetection,
  ServoFit,
  ServoSpeedFit,
  HeightAndServoFit,
};

struct InteractiveControllerState
{
  ControllerInputMode input_mode{ControllerInputMode::HeadingWalk};
  RobotMode walk_mode{RobotMode::WALK};
  GaitType gait{GaitType::TRIPOD};
  double walk_body_height_m{0.14};

  double body_height_m{0.14};
};

struct InteractiveButtonMappingResult
{
  CalibrationAction calibration_action{CalibrationAction::None};
  std::optional<std::string> info_log;
};

const char* controllerInputModeName(ControllerInputMode mode);
ControllerInputMode nextControllerInputMode(ControllerInputMode mode);

InteractiveButtonMappingResult mapInteractiveButtonEvent(const ControllerEvent& event,
                                                         InteractiveControllerState& state);

void updateControllerDerivedState(const IControlDevice& controller, InteractiveControllerState& state);

MotionIntent makeControllerMotionIntent(const IControlDevice& controller,
                                        InteractiveControllerState& state);

#endif // INTERACTIVE_INPUT_MAPPER_HPP
