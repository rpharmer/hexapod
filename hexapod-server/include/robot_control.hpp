#pragma once

#include "types.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "safety_supervisor.hpp"
#include "control_pipeline.hpp"
#include "loop_executor.hpp"
#include "logger.hpp"

#include <atomic>
#include <memory>

class RobotControl {
public:
    RobotControl(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger);

    ~RobotControl();

    bool init();
    void start();
    void stop();

    void setMotionIntent(const MotionIntent& intent);
    ControlStatus getStatus() const;

private:
    void busStep();
    void estimatorStep();
    void controlStep();
    void safetyStep();
    void diagnosticsStep();

private:
    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;
    LoopExecutor loops_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> control_loop_counter_{0};

    DoubleBuffer<RawHardwareState> raw_state_;
    DoubleBuffer<EstimatedState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;
};
