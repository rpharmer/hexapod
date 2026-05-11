#pragma once

#include "command_governor.hpp"
#include "geometry_config.hpp"
#include "logger.hpp"
#include "locomotion_debug.hpp"
#include "locomotion_feasibility.hpp"
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
    struct EpochTelemetry {
        uint64_t control_seq_id{0};
        uint64_t raw_seq_id{0};
        uint64_t est_seq_id{0};
        uint64_t intent_seq_id{0};
        uint64_t raw_timestamp_us{0};
        uint64_t est_timestamp_us{0};
        uint64_t intent_timestamp_us{0};
        uint64_t terrain_timestamp_us{0};
        uint64_t raw_age_us{0};
        uint64_t est_age_us{0};
        uint64_t intent_age_us{0};
        uint64_t terrain_age_us{0};
        bool raw_valid{false};
        bool est_valid{false};
        bool intent_valid{false};
        bool terrain_valid{false};
    };

    RobotState estimated_state{};
    JointTargets joint_targets{};
    LocomotionDebugSnapshot locomotion_debug{};
    ControlStatus status{};
    CommandGovernorState governor{};
    control_config::CommandGovernorConfig governor_config{};
    LocomotionFeasibility locomotion_feasibility{};
    EpochTelemetry epoch{};
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
