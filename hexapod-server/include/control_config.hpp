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

extern std::chrono::microseconds kBusLoopPeriod;
extern std::chrono::microseconds kEstimatorLoopPeriod;
extern std::chrono::microseconds kControlLoopPeriod;
extern std::chrono::microseconds kSafetyLoopPeriod;
extern std::chrono::milliseconds kDiagnosticsPeriod;
extern std::chrono::milliseconds kCommandRefreshPeriod;
extern std::chrono::milliseconds kStandSettlingDelay;

extern AngleRad kMaxTiltRad;
extern DurationUs kCommandTimeoutUs;
extern LinearRateMps kFallbackSpeedMag;

void loadFromParsedToml(const ParsedToml& config);

} // namespace control_config
