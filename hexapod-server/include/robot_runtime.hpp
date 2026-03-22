#pragma once

#include "control_pipeline.hpp"
#include "control_config.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "logger.hpp"
#include "safety_supervisor.hpp"
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
                 control_config::ControlConfig config = {});

    bool init();

    void busStep();
    void estimatorStep();
    void controlStep();
    void safetyStep();
    void diagnosticsStep();

    void setMotionIntent(const MotionIntent& intent);
    void setMotionIntentForTest(const MotionIntent& intent);
    bool setSimFaultToggles(const SimHardwareFaultToggles& toggles);
    ControlStatus getStatus() const;

private:
    struct StreamDiagnostics {
        std::atomic<uint64_t> stale_age_count{0};
        std::atomic<uint64_t> missing_timestamp_count{0};
        std::atomic<uint64_t> invalid_sample_id_count{0};
        std::atomic<uint64_t> non_monotonic_sample_id_count{0};
    };

    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    control_config::ControlConfig config_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;

    std::atomic<uint64_t> control_loop_counter_{0};
    std::atomic<uint64_t> control_dt_sum_us_{0};
    std::atomic<uint64_t> control_jitter_max_us_{0};
    std::atomic<uint64_t> stale_intent_count_{0};
    std::atomic<uint64_t> stale_estimator_count_{0};
    std::atomic<uint64_t> raw_sample_seq_{0};
    std::atomic<uint64_t> intent_sample_seq_{0};
    uint64_t last_estimator_sample_id_{0};
    uint64_t last_intent_sample_id_{0};
    StreamDiagnostics estimator_diag_{};
    StreamDiagnostics intent_diag_{};
    TimePointUs last_control_step_us_{};

    DoubleBuffer<RawHardwareState> raw_state_;
    DoubleBuffer<EstimatedState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;
};
