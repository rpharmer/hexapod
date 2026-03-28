#pragma once

#include "geometry_config.hpp"
#include "types.hpp"
#include "logger.hpp"

#include <cstdint>
#include <memory>
#include <optional>
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
    struct AutonomyDebugPose {
        double x_m{0.0};
        double y_m{0.0};
        double yaw_rad{0.0};
        std::string frame_id{"map"};
    };

    struct AutonomyDebugTelemetry {
        std::optional<AutonomyDebugPose> localization_pose{};
    };

    struct RuntimeFreshnessTelemetry {
        uint64_t consecutive_rejects{0};
        uint64_t total_rejects{0};
        uint64_t stale_age_rejects{0};
        uint64_t invalid_sample_id_rejects{0};
        uint64_t non_monotonic_id_rejects{0};
    };

    RobotState estimated_state{};
    MotionIntent motion_intent{};
    JointTargets joint_targets{};
    ControlStatus status{};
    TimePointUs timestamp_us{};
    bool imu_reads_enabled{false};
    AutonomyDebugTelemetry autonomy_debug{};
    RuntimeFreshnessTelemetry runtime_freshness{};
};

struct TelemetryPublishCounters {
    uint64_t packets_sent{0};
    uint64_t socket_send_failures{0};
    TimePointUs last_successful_send_timestamp{};
};

class ITelemetryPublisher {
public:
    virtual ~ITelemetryPublisher() = default;

    virtual void publishGeometry(const HexapodGeometry& geometry) = 0;
    virtual void publishControlStep(const ControlStepTelemetry& telemetry) = 0;
    virtual TelemetryPublishCounters counters() const = 0;
};

std::unique_ptr<ITelemetryPublisher> makeNoopTelemetryPublisher();
std::unique_ptr<ITelemetryPublisher> makeUdpTelemetryPublisher(
    const TelemetryPublisherConfig& config,
    std::shared_ptr<logging::AsyncLogger> logger);

} // namespace telemetry
