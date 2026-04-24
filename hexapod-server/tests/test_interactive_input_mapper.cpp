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

class StubController final : public IControlDevice {
public:
  bool start() override { return true; }
  void stop() override {}
  EventQueue<ControllerEvent>& getQueue() override { return queue_; }

  float getLeftX() const override { return left_x_; }
  float getLeftY() const override { return left_y_; }
  float getLeftMag() const override { return left_mag_; }
  float getLeftAng() const override { return left_ang_; }

  float getRightX() const override { return right_x_; }
  float getRightY() const override { return right_y_; }
  float getRightMag() const override { return right_mag_; }
  float getRightAng() const override { return right_ang_; }

  int getLeftTrigger() const override { return left_trigger_; }
  int getRightTrigger() const override { return right_trigger_; }

  void setRadialDeadzone(const std::string&, int) override {}

  EventQueue<ControllerEvent> queue_{};
  float left_x_{0.0f};
  float left_y_{0.0f};
  float left_mag_{0.0f};
  float left_ang_{0.0f};
  float right_x_{0.0f};
  float right_y_{0.0f};
  float right_mag_{0.0f};
  float right_ang_{0.0f};
  int left_trigger_{0};
  int right_trigger_{0};
};

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

bool testHeadingWalkDoesNotDoubleCountYawOrPose()
{
  StubController controller{};
  controller.left_x_ = 0.0f;
  controller.left_y_ = -1.0f;
  controller.left_mag_ = 1.0f;
  controller.left_ang_ = -1.5707964f;
  controller.right_x_ = 0.5f;
  controller.right_y_ = 0.0f;
  controller.right_mag_ = 0.5f;
  controller.right_ang_ = 0.0f;

  InteractiveControllerState state{};
  state.input_mode = ControllerInputMode::HeadingWalk;
  updateControllerDerivedState(controller, state);

  const MotionIntent intent = makeControllerMotionIntent(controller, state);
  return expect(intent.cmd_vx_mps.value > 0.0, "heading walk should map left stick up to forward motion") &&
         expect(intent.cmd_vx_mps.value <= 0.04 + 1e-9,
                "heading walk should clamp full-stick forward speed to the interactive limit") &&
         expect(std::abs(intent.cmd_vy_mps.value) < 1e-9,
                "heading walk should not add sideways motion when the stick is pushed straight up") &&
         expect(intent.cmd_yaw_radps.value > 0.0, "heading walk should command yaw rate from the right stick") &&
         expect(intent.cmd_yaw_radps.value <= 0.07 + 1e-9,
                "heading walk should clamp yaw rate to the interactive limit") &&
         expect(std::abs(intent.twist.twist_vel_radps.z) < 1e-9,
                "heading walk should not duplicate yaw into twist velocity") &&
         expect(std::abs(intent.twist.twist_pos_rad.z) < 1e-9,
                "heading walk should not command an absolute body yaw pose");
}

bool testHeadingWalkIdleDoesNotCommandMotion()
{
  StubController controller{};
  InteractiveControllerState state{};
  state.input_mode = ControllerInputMode::HeadingWalk;
  updateControllerDerivedState(controller, state);

  const MotionIntent intent = makeControllerMotionIntent(controller, state);
  return expect(intent.requested_mode == RobotMode::SAFE_IDLE,
                "heading walk should stay in SAFE_IDLE when the left stick is centered") &&
         expect(std::abs(intent.cmd_vx_mps.value) < 1e-9,
                "heading walk idle should not command forward motion") &&
         expect(std::abs(intent.cmd_vy_mps.value) < 1e-9,
                "heading walk idle should not command lateral motion") &&
         expect(std::abs(intent.cmd_yaw_radps.value) < 1e-9,
                "heading walk idle should not command yaw rate");
}

} // namespace

int main()
{
  testModeCyclingAndInfoLog();
  testCalibrationButtonMapping();
  testHeadingModeGaitButtons();
  testHeadingWalkDoesNotDoubleCountYawOrPose();
  testHeadingWalkIdleDoesNotCommandMotion();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
  }
  return 0;
}
