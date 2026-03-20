#pragma once

#include <chrono>
#include <cstdint>

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
inline constexpr AngleRad kDefaultMaxTiltRad{0.70};
inline constexpr DurationUs kDefaultCommandTimeoutUs{300000};
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
};

struct GaitConfig {
    LinearRateMps fallback_speed_mag{kDefaultFallbackSpeedMag};
};

struct ControlConfig {
    LoopTimingConfig loop_timing{};
    SafetyConfig safety{};
    GaitConfig gait{};
};

ControlConfig fromParsedToml(const ParsedToml& config);

} // namespace control_config
