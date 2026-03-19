#pragma once

#include "control_pipeline.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "logger.hpp"
#include "safety_supervisor.hpp"
#include "types.hpp"

#include <atomic>
#include <memory>

class RobotRuntime {
public:
    RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger);

    bool init();

    void busStep();
    void estimatorStep();
    void controlStep();
    void safetyStep();
    void diagnosticsStep();

    void setMotionIntent(const MotionIntent& intent);
    ControlStatus getStatus() const;

private:
    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;

    std::atomic<uint64_t> control_loop_counter_{0};

    DoubleBuffer<RawHardwareState> raw_state_;
    DoubleBuffer<EstimatedState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;
};
