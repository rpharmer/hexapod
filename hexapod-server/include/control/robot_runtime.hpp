#pragma once

#include "control_pipeline.hpp"
#include "control_config.hpp"
#include "autonomy/modules/autonomy_stack.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "freshness_policy.hpp"
#include "hardware_bridge.hpp"
#include "imu_unit.hpp"
#include "logger.hpp"
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
#include <optional>

class RobotRuntime {
public:
    RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger,
                 control_config::ControlConfig config = {},
                 std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher = telemetry::makeNoopTelemetryPublisher(),
                 std::unique_ptr<hardware::IImuUnit> imu = nullptr);

    bool init();
    void stop();
    void startTelemetry();

    void busStep();
    void estimatorStep();
    void controlStep();
    void safetyStep();
    void diagnosticsStep();

    void setMotionIntent(const MotionIntent& intent);
    void setMotionIntentForTest(const MotionIntent& intent);
    bool setSimFaultToggles(const SimHardwareFaultToggles& toggles);
    ControlStatus getStatus() const;
    void setAutonomyBlockedForTest(bool blocked);
    void setAutonomyNoProgressForTest(bool no_progress);
    void signalAutonomyWaypointReachedForTest();
    bool loadAutonomyMissionForTest(const autonomy::WaypointMission& mission);
    bool startAutonomyMissionForTest();
    std::optional<autonomy::AutonomyStepOutput> lastAutonomyStepOutputForTest() const;

private:
    void maybePublishTelemetry(const TimePointUs& now);
    MotionIntent resolveAutonomyMotionIntent(const MotionIntent& base_intent,
                                             const SafetyState& safety_state,
                                             const TimePointUs& now);

    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<hardware::IImuUnit> imu_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    control_config::ControlConfig config_;
    std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;
    FreshnessPolicy freshness_policy_;

    std::atomic<uint64_t> control_loop_counter_{0};
    std::atomic<uint64_t> control_dt_sum_us_{0};
    std::atomic<uint64_t> control_jitter_max_us_{0};
    std::atomic<uint64_t> stale_intent_count_{0};
    std::atomic<uint64_t> stale_estimator_count_{0};
    std::atomic<uint64_t> raw_sample_seq_{0};
    std::atomic<uint64_t> intent_sample_seq_{0};
    RuntimeFreshnessGate freshness_gate_;
    RuntimeTimingMetrics timing_metrics_;
    RuntimeDiagnosticsReporter diagnostics_reporter_;

    DoubleBuffer<RobotState> raw_state_;
    DoubleBuffer<RobotState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;
    JointTargets last_written_joint_targets_{};
    bool has_last_written_joint_targets_{false};
    TimePointUs last_joint_write_timestamp_{};

    TimePointUs next_telemetry_publish_at_{};
    TimePointUs next_geometry_refresh_at_{};
    std::unique_ptr<autonomy::AutonomyStack> autonomy_stack_;
    std::optional<autonomy::AutonomyStepOutput> last_autonomy_step_output_{};
    std::atomic<bool> autonomy_blocked_signal_{false};
    std::atomic<bool> autonomy_no_progress_signal_{false};
    std::atomic<bool> autonomy_waypoint_reached_event_{false};
};
