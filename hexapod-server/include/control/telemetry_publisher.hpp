#pragma once

#include "command_governor.hpp"
#include "geometry_config.hpp"
#include "logger.hpp"
#include "navigation_manager.hpp"
#include "process_resource_monitoring.hpp"
#include "types.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace telemetry {

inline constexpr std::size_t kTelemetryResourceSectionLimit = 18;
using ResourceSectionSummary =
    resource_monitoring::ResourceSectionSummary<kTelemetryResourceSectionLimit>;

enum class FusionCorrectionMode : std::uint8_t {
    None = 0,
    Soft = 1,
    Strong = 2,
    HardReset = 3,
};

struct FusionCorrectionTelemetry {
    bool has_data{false};
    FusionCorrectionMode mode{FusionCorrectionMode::None};
    uint64_t sample_id{0};
    TimePointUs timestamp_us{};
    double correction_strength{0.0};
    FusionResidualSummary residuals{};
};

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

struct FusionTelemetrySnapshot {
    bool has_data{false};
    FusionDiagnostics diagnostics{};
    std::array<FootContactFusion, kNumLegs> foot_contact_fusion{};
    FusionCorrectionTelemetry correction{};
};

struct ControlStepTelemetry {
    RobotState estimated_state{};
    JointTargets joint_targets{};
    ControlStatus status{};
    CommandGovernorState governor{};
    FusionTelemetrySnapshot fusion{};
    std::optional<NavigationMonitorSnapshot> navigation{};
    std::optional<resource_monitoring::ProcessResourceSnapshot> process_resources{};
    std::optional<ResourceSectionSummary> resource_sections{};
    TimePointUs timestamp_us{};
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
