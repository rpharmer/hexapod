#pragma once

#include "freshness_policy.hpp"
#include "hardware_bridge.hpp"
#include "joint_oscillation_tracker.hpp"
#include "logger.hpp"
#include "telemetry_publisher.hpp"
#include "types.hpp"

#include <cstdint>
#include <memory>
#include <optional>

struct VisualizerTelemetryDiagnostics {
    uint64_t packets_sent{0};
    uint64_t serialization_failures{0};
    uint64_t socket_send_failures{0};
    uint64_t dropped_rate_limited_frames{0};
    TimePointUs last_successful_send_timestamp{};
};

class RuntimeDiagnosticsReporter {
public:
    RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                               const FreshnessPolicy& freshness_policy);

    void recordVisualizerTelemetry(const telemetry::TelemetryPublishCounters& telemetry_counters,
                                   TimePointUs now);
    void recordJointTargets(const JointTargets& targets, TimePointUs now);

    void report(const ControlStatus& status,
                const std::optional<BridgeCommandResultMetadata>& bridge_result,
                uint64_t loops,
                uint64_t avg_control_dt_us,
                uint64_t max_control_jitter_us,
                uint64_t stale_intent_events,
                uint64_t stale_estimator_events);

private:
    void maybeLogVisualizerFailureWarning(TimePointUs now);
    void maybeLogVisualizerRecovery();

    std::shared_ptr<logging::AsyncLogger> logger_;
    const FreshnessPolicy& freshness_policy_;
    VisualizerTelemetryDiagnostics telemetry_diag_{};
    telemetry::TelemetryPublishCounters last_telemetry_counters_{};
    uint64_t consecutive_failures_{0};
    uint64_t warning_failures_observed_{0};
    JointOscillationTracker joint_oscillation_tracker_{};
};
