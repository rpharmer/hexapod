#include "interactive_input_mapper.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

bool expect(bool cond, const std::string& message)
{
  if (!cond) {
    std::cerr << "[FAIL] " << message << '\n';
    ++g_failures;
    return false;
  }
  return true;
}

ControllerEvent buttonEvent(const std::string& name)
{
  ControllerEvent ev{};
  ev.type = ControllerEvent::Type::Button;
  ev.name = name;
  ev.value = 1;
  return ev;
}

bool testModeCyclingAndInfoLog()
{
  InteractiveControllerState state{};

  const InteractiveButtonMappingResult first = mapInteractiveButtonEvent(buttonEvent("A"), state);
  const InteractiveButtonMappingResult second = mapInteractiveButtonEvent(buttonEvent("A"), state);
  const InteractiveButtonMappingResult third = mapInteractiveButtonEvent(buttonEvent("A"), state);

  return expect(first.info_log.has_value() && first.info_log.value().find("body_pose") != std::string::npos,
                "A press should switch from heading_walk to body_pose") &&
         expect(second.info_log.has_value() && second.info_log.value().find("calibration") != std::string::npos,
                "A press should switch from body_pose to calibration") &&
         expect(third.info_log.has_value() && third.info_log.value().find("heading_walk") != std::string::npos,
                "A press should switch from calibration to heading_walk");
}

bool testCalibrationButtonMapping()
{
  InteractiveControllerState state{};
  state.input_mode = ControllerInputMode::Calibration;

  const InteractiveButtonMappingResult b_result = mapInteractiveButtonEvent(buttonEvent("B"), state);
  const InteractiveButtonMappingResult x_result = mapInteractiveButtonEvent(buttonEvent("X"), state);
  const InteractiveButtonMappingResult lb_result = mapInteractiveButtonEvent(buttonEvent("LB"), state);
  const InteractiveButtonMappingResult y_result = mapInteractiveButtonEvent(buttonEvent("Y"), state);

  return expect(b_result.calibration_action == CalibrationAction::HeightDetection,
                "B in calibration mode should map to height detection") &&
         expect(x_result.calibration_action == CalibrationAction::ServoFit,
                "X in calibration mode should map to servo fit") &&
         expect(lb_result.calibration_action == CalibrationAction::ServoSpeedFit,
                "LB in calibration mode should map to servo speed fit") &&
         expect(y_result.calibration_action == CalibrationAction::HeightAndServoFit,
                "Y in calibration mode should map to combined height+servo fit");
}

bool testHeadingModeGaitButtons()
{
  InteractiveControllerState state{};
  state.gait = GaitType::TRIPOD;

  mapInteractiveButtonEvent(buttonEvent("X"), state);
  const bool ripple_selected =
      expect(state.gait == GaitType::RIPPLE, "X in heading mode should select ripple gait");

  mapInteractiveButtonEvent(buttonEvent("Y"), state);
  const bool tripod_selected =
      expect(state.gait == GaitType::TRIPOD, "Y in heading mode should select tripod gait");

  return ripple_selected && tripod_selected;
}

} // namespace

int main()
{
  testModeCyclingAndInfoLog();
  testCalibrationButtonMapping();
  testHeadingModeGaitButtons();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
