#pragma once

#include <vector>

#include "calibration_probe.hpp"

Vec3 computeFootInBodyFromServoAngles(const LegState& servo_angles,
                                      const LegGeometry& leg_geometry,
                                      const ServoCalibration& calibration);

double touchResidualMeters(const CalibrationTouchSample& sample,
                           const LegGeometry& leg_geometry,
                           const ServoCalibration& calibration,
                           double plane_height_m);

std::vector<const CalibrationTouchSample*> filterContactSamples(
    const std::vector<CalibrationTouchSample>& samples);

double computeTouchResidualRms(const std::vector<const CalibrationTouchSample*>& samples,
                               const LegGeometry& leg_geometry,
                               const ServoCalibration& calibration,
                               double plane_height_m);
