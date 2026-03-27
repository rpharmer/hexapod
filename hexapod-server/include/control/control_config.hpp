#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

#include "types.hpp"

struct ParsedToml;

namespace control_config {

inline constexpr int kDefaultBusLoopPeriodUs = 2000;
inline constexpr int kDefaultEstimatorLoopPeriodUs = 2000;
inline constexpr int kDefaultControlLoopPeriodUs = 4000;
inline constexpr int kDefaultSafetyLoopPeriodUs = 2000;
inline constexpr int kDefaultDiagnosticsPeriodMs = 500;
inline constexpr int kDefaultCommandRefreshPeriodMs = 100;
inline constexpr int kDefaultStandSettlingDelayMs = 2000;
inline constexpr int kDefaultTelemetryPublishPeriodMs = 50;
inline constexpr int kDefaultTelemetryGeometryRefreshPeriodMs = 2000;
inline constexpr int kDefaultTelemetryUdpPort = 9870;
inline constexpr bool kDefaultImuEnableReads = false;
inline constexpr AngleRad kDefaultMaxTiltRad{0.70};
inline constexpr DurationUs kDefaultCommandTimeoutUs{300000};
inline constexpr DurationUs kDefaultEstimatorMaxAgeUs{300000};
inline constexpr DurationUs kDefaultIntentMaxAgeUs{300000};
inline constexpr LinearRateMps kDefaultFallbackSpeedMag{0.01};
inline constexpr double kDefaultGaitFrequencyMinHz{0.5};
inline constexpr double kDefaultGaitFrequencyMaxHz{2.5};
inline constexpr double kDefaultGaitNominalMaxSpeedMps{0.25};
inline constexpr double kDefaultGaitReachEnvelopeMinScale{0.25};
inline constexpr double kDefaultGaitReachEnvelopeSoftLimit{0.15};
inline constexpr double kDefaultTripodDutyCycle{0.5};
inline constexpr double kDefaultRippleDutyCycle{0.5};
inline constexpr double kDefaultWaveDutyCycle{0.5};
inline constexpr std::array<double, kNumLegs> kDefaultTripodPhaseOffsets{{0.0, 0.5, 0.0, 0.5, 0.0, 0.5}};
inline constexpr std::array<double, kNumLegs> kDefaultRipplePhaseOffsets{{0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0}};
inline constexpr std::array<double, kNumLegs> kDefaultWavePhaseOffsets{{0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0}};
inline constexpr double kDefaultSwingHeightM{0.03};
inline constexpr double kDefaultFootholdStepLengthM{0.06};
inline constexpr double kDefaultStanceFieldCenterXM{0.0};
inline constexpr double kDefaultStanceFieldCenterYM{0.0};
inline constexpr double kDefaultStanceFieldRadiusXM{0.12};
inline constexpr double kDefaultStanceFieldRadiusYM{0.10};
inline constexpr double kDefaultStabilityPriority{1.0};
inline constexpr double kDefaultReachSuppressionGain{1.0};
inline constexpr double kDefaultTurnSuppressionGain{1.0};
inline constexpr double kDefaultTurnYawRateEnterRadps{0.4};
inline constexpr double kDefaultTurnYawRateExitRadps{0.3};
inline constexpr double kDefaultTurnSpeedEnterMps{0.05};
inline constexpr double kDefaultTurnSpeedExitMps{0.03};
inline constexpr float kDefaultMinBusVoltageV{10.5f};
inline constexpr float kDefaultMaxBusCurrentA{25.0f};
inline constexpr int kDefaultMinFootContacts{0};
inline constexpr int kDefaultMaxFootContacts{kNumLegs};
inline constexpr double kDefaultTraversabilityOccupancyRiskWeight{0.65};
inline constexpr double kDefaultTraversabilityGradientRiskWeight{0.35};
inline constexpr double kDefaultTraversabilityConfidenceCostWeight{1.0};
inline constexpr double kDefaultTraversabilityRiskBlockThreshold{0.85};
inline constexpr double kDefaultTraversabilityConfidenceBlockThreshold{0.3};

struct LoopTimingConfig {
    std::chrono::microseconds bus_loop_period{std::chrono::microseconds{kDefaultBusLoopPeriodUs}};
    std::chrono::microseconds estimator_loop_period{std::chrono::microseconds{kDefaultEstimatorLoopPeriodUs}};
    std::chrono::microseconds control_loop_period{std::chrono::microseconds{kDefaultControlLoopPeriodUs}};
    std::chrono::microseconds safety_loop_period{std::chrono::microseconds{kDefaultSafetyLoopPeriodUs}};
    std::chrono::milliseconds diagnostics_period{std::chrono::milliseconds{kDefaultDiagnosticsPeriodMs}};
    std::chrono::milliseconds command_refresh_period{std::chrono::milliseconds{kDefaultCommandRefreshPeriodMs}};
    std::chrono::milliseconds stand_settling_delay{std::chrono::milliseconds{kDefaultStandSettlingDelayMs}};
};

struct SafetyConfig {
    AngleRad max_tilt_rad{kDefaultMaxTiltRad};
    DurationUs command_timeout_us{kDefaultCommandTimeoutUs};
    float min_bus_voltage_v{kDefaultMinBusVoltageV};
    float max_bus_current_a{kDefaultMaxBusCurrentA};
    int min_foot_contacts{kDefaultMinFootContacts};
    int max_foot_contacts{kDefaultMaxFootContacts};
    AngleRad max_joint_position_step_rad{AngleRad{1.0}};
};

struct GaitFrequencyConfig {
    FrequencyHz min_hz{FrequencyHz{kDefaultGaitFrequencyMinHz}};
    FrequencyHz max_hz{FrequencyHz{kDefaultGaitFrequencyMaxHz}};
    LinearRateMps nominal_max_speed_mps{LinearRateMps{kDefaultGaitNominalMaxSpeedMps}};
    double reach_envelope_soft_limit{kDefaultGaitReachEnvelopeSoftLimit};
    double reach_envelope_min_scale{kDefaultGaitReachEnvelopeMinScale};
};

struct GaitDutyConfig {
    double tripod{kDefaultTripodDutyCycle};
    double ripple{kDefaultRippleDutyCycle};
    double wave{kDefaultWaveDutyCycle};
};

struct GaitPhaseOffsetsConfig {
    std::array<double, kNumLegs> tripod{kDefaultTripodPhaseOffsets};
    std::array<double, kNumLegs> ripple{kDefaultRipplePhaseOffsets};
    std::array<double, kNumLegs> wave{kDefaultWavePhaseOffsets};
};

struct GaitSwingConfig {
    LengthM height_m{LengthM{kDefaultSwingHeightM}};
};

struct GaitFootholdConfig {
    LengthM step_length_m{LengthM{kDefaultFootholdStepLengthM}};
};

struct GaitStanceFieldConfig {
    LengthM center_x_m{LengthM{kDefaultStanceFieldCenterXM}};
    LengthM center_y_m{LengthM{kDefaultStanceFieldCenterYM}};
    LengthM radius_x_m{LengthM{kDefaultStanceFieldRadiusXM}};
    LengthM radius_y_m{LengthM{kDefaultStanceFieldRadiusYM}};
};

struct GaitPrioritySuppressionConfig {
    double stability_priority{kDefaultStabilityPriority};
    double reach_suppression_gain{kDefaultReachSuppressionGain};
    double turn_suppression_gain{kDefaultTurnSuppressionGain};
};

struct TurnModeThresholdConfig {
    AngularRateRadPerSec yaw_rate_enter_radps{AngularRateRadPerSec{kDefaultTurnYawRateEnterRadps}};
    AngularRateRadPerSec yaw_rate_exit_radps{AngularRateRadPerSec{kDefaultTurnYawRateExitRadps}};
    LinearRateMps speed_enter_mps{LinearRateMps{kDefaultTurnSpeedEnterMps}};
    LinearRateMps speed_exit_mps{LinearRateMps{kDefaultTurnSpeedExitMps}};
};

struct DynamicGaitAcceptanceGateConfig {
    bool feature_flag_enabled{false};
    bool simulator_first_required{true};
    int simulator_validation_runs_required{5};
    int simulator_validation_runs_passed{0};
    double max_control_latency_p95_ms{8.0};
    double observed_control_latency_p95_ms{0.0};
    double max_safety_faults_per_hour{0.20};
    double observed_safety_faults_per_hour{0.0};
    double min_stability_margin_m{0.015};
    double observed_min_stability_margin_m{0.0};
};

struct GaitConfig {
    LinearRateMps fallback_speed_mag{kDefaultFallbackSpeedMag};
    GaitFrequencyConfig frequency{};
    GaitDutyConfig duty{};
    GaitPhaseOffsetsConfig phase_offsets{};
    GaitSwingConfig swing{};
    GaitFootholdConfig foothold{};
    GaitStanceFieldConfig stance_field{};
    GaitPrioritySuppressionConfig priority_suppression{};
    TurnModeThresholdConfig turn_mode_thresholds{};
    DynamicGaitAcceptanceGateConfig acceptance_gate{};
};

struct StreamFreshnessConfig {
    DurationUs max_allowed_age_us{kDefaultCommandTimeoutUs};
    bool require_timestamp{true};
    bool require_nonzero_sample_id{true};
    bool require_monotonic_sample_id{true};
};

struct FreshnessConfig {
    StreamFreshnessConfig estimator{
        kDefaultEstimatorMaxAgeUs,
        true,
        true,
        true};
    StreamFreshnessConfig intent{
        kDefaultIntentMaxAgeUs,
        true,
        true,
        true};
};

struct TelemetryConfig {
    bool enabled{false};
    std::string host{"127.0.0.1"};
    int port{9870};
    double publish_rate_hz{30.0};
    double geometry_resend_interval_sec{1.0};
    std::string udp_host{"127.0.0.1"};
    int udp_port{kDefaultTelemetryUdpPort};
    std::chrono::milliseconds publish_period{std::chrono::milliseconds{kDefaultTelemetryPublishPeriodMs}};
    std::chrono::milliseconds geometry_refresh_period{std::chrono::milliseconds{kDefaultTelemetryGeometryRefreshPeriodMs}};
};

struct RuntimeImuConfig {
    bool enable_reads{kDefaultImuEnableReads};
};

struct AutonomyRuntimeConfig {
    struct TraversabilityConfig {
        double occupancy_risk_weight{kDefaultTraversabilityOccupancyRiskWeight};
        double gradient_risk_weight{kDefaultTraversabilityGradientRiskWeight};
        double confidence_cost_weight{kDefaultTraversabilityConfidenceCostWeight};
        double risk_block_threshold{kDefaultTraversabilityRiskBlockThreshold};
        double confidence_block_threshold{kDefaultTraversabilityConfidenceBlockThreshold};
    };

    bool enabled{false};
    uint64_t no_progress_timeout_ms{1000};
    uint64_t recovery_retry_budget{2};
    TraversabilityConfig traversability{};
};

struct ControlConfig {
    LoopTimingConfig loop_timing{};
    SafetyConfig safety{};
    GaitConfig gait{};
    FreshnessConfig freshness{};
    TelemetryConfig telemetry{};
    RuntimeImuConfig runtime_imu{};
    AutonomyRuntimeConfig autonomy{};
};

ControlConfig fromParsedToml(const ParsedToml& config);

} // namespace control_config
