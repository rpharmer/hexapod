#pragma once

#include <array>
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

struct BaseClearanceSample {
    std::array<LegRawState, kNumLegs> servo_angles{};
    BodyPose body_pose{};
    std::array<bool, kNumLegs> foot_contacts{};
};

struct BaseClearanceEstimateOptions {
    double min_contact_fraction_before_loss{0.66};
};

struct BaseClearanceEstimateResult {
    bool success{false};
    int transition_index{-1};
    int contact_legs_before_loss{0};
    double ground_plane_height_m{0.0};
    LengthM estimated_to_bottom_m{};
};

BaseClearanceEstimateResult estimateToBottomFromSynchronousLift(
    const HexapodGeometry& geometry,
    const std::vector<BaseClearanceSample>& samples,
    const BaseClearanceEstimateOptions& options = BaseClearanceEstimateOptions{});

struct ServoDynamicsSample {
    double time_s{0.0};
    LegRawState command_servo_angles{};
    LegRawState measured_servo_angles{};
};

struct SingleJointDynamicsFit {
    bool success{false};
    double tau_s{0.0};
    double vmax_radps{0.0};
    int used_samples{0};
};

struct LegServoDynamicsFitResult {
    std::array<SingleJointDynamicsFit, kJointsPerLeg> positive_direction{};
    std::array<SingleJointDynamicsFit, kJointsPerLeg> negative_direction{};
};

struct ServoDynamicsFitOptions {
    double min_dt_s{1e-4};
    double min_err_rad{1e-3};
    int min_samples{6};
    double unsaturated_velocity_fraction{0.85};
};

LegServoDynamicsFitResult fitLegServoDynamicsFromSamples(
    const std::vector<ServoDynamicsSample>& samples,
    const ServoDynamicsFitOptions& options = ServoDynamicsFitOptions{});

enum class ProbeDestinationStatus {
    Reached = 0,
    EarlyContact = 1,
    TimeoutNoContact = 2,
    TimeoutNoProgress = 3,
    NotReached = 4,
};

struct ProbeDestinationSample {
    double time_s{0.0};
    Vec3 foot_position_body{};
    Vec3 target_position_body{};
    bool foot_contact{false};
};

struct ProbeDestinationOptions {
    double position_tolerance_m{0.003};
    int contact_debounce_samples{3};
    double timeout_s{1.0};
};

struct ProbeDestinationResult {
    ProbeDestinationStatus status{ProbeDestinationStatus::NotReached};
    bool reached{false};
    bool contact_debounced{false};
    bool in_position{false};
    double decision_time_s{0.0};
    double contact_rise_time_s{-1.0};
};

ProbeDestinationResult evaluateProbeDestinationReached(
    const std::vector<ProbeDestinationSample>& samples,
    const ProbeDestinationOptions& options = ProbeDestinationOptions{});
