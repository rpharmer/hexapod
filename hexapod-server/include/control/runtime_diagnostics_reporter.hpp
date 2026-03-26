#pragma once

#include "freshness_policy.hpp"
#include "hardware_bridge.hpp"
#include "joint_oscillation_tracker.hpp"
#include "logger.hpp"
#include "runtime_timing_metrics.hpp"
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

struct LegTargetVariabilityDiagnostics {
    double peak_foot_target_velocity_mps{0.0};
    std::array<double, kNumLegs> peak_foot_target_velocity_mps_by_leg{};
    uint64_t rapid_change_events{0};
};

struct GaitVariabilityDiagnostics {
    double peak_cadence_delta_hz_per_s{0.0};
    double peak_reach_utilization_delta_per_s{0.0};
    double peak_phase_delta_per_s{0.0};
    std::array<double, kNumLegs> peak_phase_delta_per_s_by_leg{};
    uint64_t rapid_change_events{0};
};

class RuntimeDiagnosticsReporter {
public:
    RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                               const FreshnessPolicy& freshness_policy);

    void recordVisualizerTelemetry(const telemetry::TelemetryPublishCounters& telemetry_counters,
                                   TimePointUs now);
    void recordControlOutputs(const JointTargets& targets,
                              const ControlStatus& status,
                              TimePointUs now,
                              const LegTargets* leg_targets = nullptr);

    void report(const ControlStatus& status,
                const std::optional<BridgeCommandResultMetadata>& bridge_result,
                uint64_t loops,
                uint64_t avg_control_dt_us,
                uint64_t max_control_jitter_us,
                const LoopTimingRollingMetrics& loop_timing_metrics,
                uint64_t stale_intent_events,
                uint64_t stale_estimator_events);

private:
    bool isTransitionStep(const ControlStatus& status) const;
    void maybeLogVisualizerFailureWarning(TimePointUs now);
    void maybeLogVisualizerRecovery();
    void maybeLogLoopTimingWarning(const LoopTimingRollingMetrics& loop_timing_metrics);

    std::shared_ptr<logging::AsyncLogger> logger_;
    const FreshnessPolicy& freshness_policy_;
    VisualizerTelemetryDiagnostics telemetry_diag_{};
    telemetry::TelemetryPublishCounters last_telemetry_counters_{};
    uint64_t consecutive_failures_{0};
    uint64_t warning_failures_observed_{0};
    JointOscillationTracker joint_oscillation_tracker_{};
    LegTargetVariabilityDiagnostics leg_target_variability_diag_{};
    GaitVariabilityDiagnostics gait_variability_diag_{};
    bool has_previous_leg_targets_{false};
    LegTargets previous_leg_targets_{};
    bool has_previous_status_{false};
    ControlStatus previous_status_{};
    TimePointUs last_control_output_timestamp_{};
    bool diagnostics_tracking_active_{false};
    uint64_t last_overrun_warning_consecutive_{0};
};
