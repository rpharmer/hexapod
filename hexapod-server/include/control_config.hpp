#pragma once

#include <chrono>
#include <cstdint>

struct ParsedToml;

namespace control_config {

inline constexpr int kDefaultBusLoopPeriodUs = 2000;
inline constexpr int kDefaultEstimatorLoopPeriodUs = 2000;
inline constexpr int kDefaultControlLoopPeriodUs = 4000;
inline constexpr int kDefaultSafetyLoopPeriodUs = 2000;
inline constexpr int kDefaultDiagnosticsPeriodMs = 500;
inline constexpr int kDefaultCommandRefreshPeriodMs = 100;
inline constexpr int kDefaultStandSettlingDelayMs = 2000;
inline constexpr double kDefaultMaxTiltRad = 0.70;
inline constexpr uint64_t kDefaultCommandTimeoutUs = 300000;
inline constexpr double kDefaultFallbackSpeedMag = 0.01;

extern std::chrono::microseconds kBusLoopPeriod;
extern std::chrono::microseconds kEstimatorLoopPeriod;
extern std::chrono::microseconds kControlLoopPeriod;
extern std::chrono::microseconds kSafetyLoopPeriod;
extern std::chrono::milliseconds kDiagnosticsPeriod;
extern std::chrono::milliseconds kCommandRefreshPeriod;
extern std::chrono::milliseconds kStandSettlingDelay;

extern double kMaxTiltRad;
extern uint64_t kCommandTimeoutUs;
extern double kFallbackSpeedMag;

void loadFromParsedToml(const ParsedToml& config);

} // namespace control_config
