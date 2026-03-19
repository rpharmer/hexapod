#include "control_config.hpp"

#include "hexapod-server.hpp"

namespace control_config {

std::chrono::microseconds kBusLoopPeriod{kDefaultBusLoopPeriodUs};
std::chrono::microseconds kEstimatorLoopPeriod{kDefaultEstimatorLoopPeriodUs};
std::chrono::microseconds kControlLoopPeriod{kDefaultControlLoopPeriodUs};
std::chrono::microseconds kSafetyLoopPeriod{kDefaultSafetyLoopPeriodUs};
std::chrono::milliseconds kDiagnosticsPeriod{kDefaultDiagnosticsPeriodMs};
std::chrono::milliseconds kCommandRefreshPeriod{kDefaultCommandRefreshPeriodMs};
std::chrono::milliseconds kStandSettlingDelay{kDefaultStandSettlingDelayMs};

AngleRad kMaxTiltRad = kDefaultMaxTiltRad;
uint64_t kCommandTimeoutUs = kDefaultCommandTimeoutUs;
LinearRateMps kFallbackSpeedMag = kDefaultFallbackSpeedMag;

void loadFromParsedToml(const ParsedToml& config) {
    kBusLoopPeriod = std::chrono::microseconds{config.busLoopPeriodUs};
    kEstimatorLoopPeriod = std::chrono::microseconds{config.estimatorLoopPeriodUs};
    kControlLoopPeriod = std::chrono::microseconds{config.controlLoopPeriodUs};
    kSafetyLoopPeriod = std::chrono::microseconds{config.safetyLoopPeriodUs};
    kDiagnosticsPeriod = std::chrono::milliseconds{config.diagnosticsPeriodMs};
    kCommandRefreshPeriod = std::chrono::milliseconds{config.commandRefreshPeriodMs};
    kStandSettlingDelay = std::chrono::milliseconds{config.standSettlingDelayMs};

    kMaxTiltRad = AngleRad{config.maxTiltRad};
    kCommandTimeoutUs = config.commandTimeoutUs;
    kFallbackSpeedMag = LinearRateMps{config.fallbackSpeedMag};
}

} // namespace control_config
