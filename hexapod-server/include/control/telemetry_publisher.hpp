#pragma once

#include "geometry_config.hpp"
#include "types.hpp"
#include "logger.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace telemetry {

inline constexpr int kDefaultPublishPeriodMs = 50;
inline constexpr int kDefaultGeometryRefreshPeriodMs = 2000;
inline constexpr int kDefaultUdpPort = 9870;

struct TelemetryPublisherConfig {
    bool enabled{false};
    std::string udp_host{"127.0.0.1"};
    int udp_port{kDefaultUdpPort};
    int publish_period_ms{kDefaultPublishPeriodMs};
    int geometry_refresh_period_ms{kDefaultGeometryRefreshPeriodMs};
};

struct ControlStepTelemetry {
    RobotState estimated_state{};
    JointTargets joint_targets{};
    ControlStatus status{};
    TimePointUs timestamp_us{};
};

class ITelemetryPublisher {
public:
    virtual ~ITelemetryPublisher() = default;

    virtual void publishGeometry(const HexapodGeometry& geometry) = 0;
    virtual void publishControlStep(const ControlStepTelemetry& telemetry) = 0;
};

std::unique_ptr<ITelemetryPublisher> makeNoopTelemetryPublisher();
std::unique_ptr<ITelemetryPublisher> makeUdpTelemetryPublisher(
    const TelemetryPublisherConfig& config,
    std::shared_ptr<logging::AsyncLogger> logger);

} // namespace telemetry
