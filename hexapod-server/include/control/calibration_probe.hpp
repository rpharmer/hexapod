#pragma once

#include <vector>

#include "types.hpp"

struct CalibrationTouchSample {
    LegRawState servo_angles{};
    BodyPose body_pose{};
    bool contact{true};
};

struct CalibrationFitOptions {
    double plane_height_m{0.0};
    int max_iterations{20};
    double learning_rate{0.2};
    double finite_diff_step_rad{deg2rad(0.25)};
    double max_offset_delta_rad{deg2rad(12.0)};
    double improvement_epsilon{1e-6};
};

struct CalibrationLegFitResult {
    bool success{false};
    int used_samples{0};
    ServoCalibration initial_calibration{};
    ServoCalibration fitted_calibration{};
    AngleRad coxa_delta{};
    AngleRad femur_delta{};
    AngleRad tibia_delta{};
    double rms_error_before_m{0.0};
    double rms_error_after_m{0.0};
};

CalibrationLegFitResult fitServoCalibrationFromTouches(
    const LegGeometry& leg_geometry,
    const std::vector<CalibrationTouchSample>& samples,
    const CalibrationFitOptions& options = CalibrationFitOptions{});
