#pragma once

#include <chrono>
#include <cstdint>

namespace control_config {

inline constexpr std::chrono::microseconds kBusLoopPeriod{2000};
inline constexpr std::chrono::microseconds kEstimatorLoopPeriod{2000};
inline constexpr std::chrono::microseconds kControlLoopPeriod{4000};
inline constexpr std::chrono::microseconds kSafetyLoopPeriod{2000};
inline constexpr std::chrono::milliseconds kDiagnosticsPeriod{500};
inline constexpr std::chrono::milliseconds kCommandRefreshPeriod{100};
inline constexpr std::chrono::seconds kStandSettlingDelay{2};

inline constexpr double kMaxTiltRad = 0.70;
inline constexpr uint64_t kCommandTimeoutUs = 300000; // 300 ms

inline constexpr double kFallbackSpeedMag = 0.01;

} // namespace control_config
