#include "interactive_calibration_actions.hpp"

#include <vector>

#include "calibration_probe.hpp"
#include "geometry_config.hpp"

using namespace logging;

namespace {

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

  for (int leg = 0; leg < kNumLegs; ++leg) {
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
      auto& joint_dynamics = geometry_config::kHexapodGeometry.legGeometry[leg].servoDynamics[joint];
      joint_dynamics.positive_direction.tau_s = result.positive_direction[joint].tau_s;
      joint_dynamics.positive_direction.vmax_radps = result.positive_direction[joint].vmax_radps;
      joint_dynamics.negative_direction.tau_s = result.negative_direction[joint].tau_s;
      joint_dynamics.negative_direction.vmax_radps = result.negative_direction[joint].vmax_radps;
    }
  }

  LOG_INFO(logger, "calibration.servo_speed_fit applied to active geometry dynamics profile");
  return true;
}

} // namespace

bool executeCalibrationAction(CalibrationAction action,
                              const std::shared_ptr<AsyncLogger>& logger)
{
  switch (action) {
    case CalibrationAction::None:
      return true;
    case CalibrationAction::HeightDetection:
      return runHeightDetectionProbe(logger);
    case CalibrationAction::ServoFit:
      return runServoCalibrationProbe(logger);
    case CalibrationAction::ServoSpeedFit:
      return runServoSpeedCalibrationProbe(logger);
    case CalibrationAction::HeightAndServoFit: {
      const bool height_ok = runHeightDetectionProbe(logger);
      const bool servo_ok = runServoCalibrationProbe(logger);
      return height_ok && servo_ok;
    }
    default:
      return false;
  }
}
