#pragma once

#include "control_pipeline.hpp"
#include "control_config.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "freshness_policy.hpp"
#include "hardware_bridge.hpp"
#include "logger.hpp"
#include "navigation_manager.hpp"
#include "replay_logger.hpp"
#include "process_resource_monitoring.hpp"
#include "runtime_resource_monitoring.hpp"
#include "runtime_diagnostics_reporter.hpp"
#include "runtime_freshness_gate.hpp"
#include "runtime_timing_metrics.hpp"
#include "safety_supervisor.hpp"
#include "telemetry_publisher.hpp"
#include "sim_hardware_bridge.hpp"
#include "types.hpp"

#include <atomic>
#include <cstdint>
#include <memory>

class RobotRuntime {
public:
    RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger,
                 control_config::ControlConfig config = {},
                 std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher = telemetry::makeNoopTelemetryPublisher(),
                 std::unique_ptr<replay::IReplayLogger> replay_logger = replay::makeNoopReplayLogger());

    bool init();
    void startTelemetry();

    void busStep();
    void estimatorStep();
    void controlStep();
    void safetyStep();
    void diagnosticsStep();

    void setMotionIntent(const MotionIntent& intent);
    void setMotionIntentForTest(const MotionIntent& intent);
    bool setSimFaultToggles(const SimHardwareFaultToggles& toggles);
    void setNavigationManager(std::unique_ptr<NavigationManager> navigation_manager);
    ControlStatus getStatus() const;
    SafetyState getSafetyState() const;

    /** Latest estimated state (for navigation / tests; same buffer the control step reads). */
    [[nodiscard]] RobotState estimatedSnapshot() const { return estimated_state_.read(); }
    [[nodiscard]] const NavigationManager* navigationManager() const { return navigation_manager_.get(); }
    [[nodiscard]] NavigationManager* navigationManager() { return navigation_manager_.get(); }

private:
    MotionIntent resolveEffectiveIntent(const RobotState& est, TimePointUs now);
    struct FusionControlPolicy {
        double aggressiveness_scale{1.0};
        bool force_stand{false};
    };
    FusionControlPolicy applyFusionConsistency(const RobotState& est,
                                              TimePointUs now,
                                              const LocalMapSnapshot* terrain_snapshot);
    static void scaleMotionIntent(MotionIntent& intent, double scale);
    void maybePublishTelemetry(const TimePointUs& now);
    void maybeWriteReplayRecord(const TimePointUs& now);

    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    control_config::ControlConfig config_;
    std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher_;
    std::unique_ptr<replay::IReplayLogger> replay_logger_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;
    FreshnessPolicy freshness_policy_;
    std::unique_ptr<NavigationManager> navigation_manager_{};

    std::atomic<uint64_t> control_loop_counter_{0};
    std::atomic<uint64_t> control_dt_sum_us_{0};
    std::atomic<uint64_t> control_jitter_max_us_{0};
    std::atomic<uint64_t> stale_intent_count_{0};
    std::atomic<uint64_t> stale_estimator_count_{0};
    std::atomic<uint64_t> raw_sample_seq_{0};
    std::atomic<uint64_t> intent_sample_seq_{0};
    RuntimeFreshnessGate freshness_gate_;
    RuntimeTimingMetrics timing_metrics_;
    resource_monitoring::ProcessResourceSampler process_resource_sampler_;
    runtime_resource_monitoring::Profiler resource_profiler_{runtime_resource_monitoring::kSectionLabels};
    std::optional<resource_monitoring::ProcessResourceSnapshot> last_process_resources_{};
    std::optional<telemetry::ResourceSectionSummary> last_resource_sections_{};
    RuntimeDiagnosticsReporter diagnostics_reporter_;
    TimePointUs last_estimator_snapshot_log_us_{};

    DoubleBuffer<RobotState> raw_state_;
    DoubleBuffer<RobotState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<MotionIntent> effective_motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<LegTargets> leg_targets_;
    DoubleBuffer<GaitState> gait_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;

    TimePointUs next_telemetry_publish_at_{};
    TimePointUs next_geometry_refresh_at_{};
    uint64_t last_effective_intent_estimator_sample_id_{0};
    uint64_t last_fusion_reset_sample_id_{0};
    uint64_t last_fusion_correction_sample_id_{0};
    FaultCode last_logged_safety_fault_{FaultCode::NONE};
    std::uint8_t last_fusion_correction_mode_{0};
    telemetry::FusionCorrectionTelemetry last_fusion_correction_{};
    std::atomic<uint64_t> fusion_hard_reset_request_count_{0};
    std::atomic<uint64_t> fusion_resync_request_count_{0};
    std::atomic<uint64_t> fusion_emit_soft_count_{0};
    std::atomic<uint64_t> fusion_emit_strong_count_{0};
    std::atomic<uint64_t> fusion_emit_hard_reset_count_{0};
    std::atomic<uint64_t> fusion_suppressed_count_{0};
};
