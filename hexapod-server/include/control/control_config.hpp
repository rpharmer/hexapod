#pragma once

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
inline constexpr AngleRad kDefaultMaxTiltRad{0.70};
inline constexpr DurationUs kDefaultCommandTimeoutUs{300000};
inline constexpr DurationUs kDefaultEstimatorMaxAgeUs{300000};
inline constexpr DurationUs kDefaultIntentMaxAgeUs{300000};
inline constexpr LinearRateMps kDefaultFallbackSpeedMag{0.01};
inline constexpr float kDefaultMinBusVoltageV{10.5f};
inline constexpr float kDefaultMaxBusCurrentA{25.0f};
inline constexpr int kDefaultMinFootContacts{0};
inline constexpr int kDefaultMaxFootContacts{kNumLegs};

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

struct GaitConfig {
    LinearRateMps fallback_speed_mag{kDefaultFallbackSpeedMag};
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

struct ControlConfig {
    LoopTimingConfig loop_timing{};
    SafetyConfig safety{};
    GaitConfig gait{};
    FreshnessConfig freshness{};
    TelemetryConfig telemetry{};
};

ControlConfig fromParsedToml(const ParsedToml& config);

} // namespace control_config
