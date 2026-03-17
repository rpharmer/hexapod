#pragma once

#include "types.hpp"
#include "double_buffer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "safety_supervisor.hpp"
#include "control_pipeline.hpp"
#include "logger.hpp"

#include <atomic>
#include <memory>
#include <thread>

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
    void busLoop();
    void estimatorLoop();
    void controlLoop();
    void safetyLoop();
    void diagnosticsLoop();

    static void joinThread(std::thread& t);

private:
    std::unique_ptr<IHardwareBridge> hw_;
    std::unique_ptr<IEstimator> estimator_;
    std::shared_ptr<logging::AsyncLogger> logger_;

    ControlPipeline pipeline_;
    SafetySupervisor safety_;

    std::atomic<bool> running_{false};

    DoubleBuffer<RawHardwareState> raw_state_;
    DoubleBuffer<EstimatedState> estimated_state_;
    DoubleBuffer<MotionIntent> motion_intent_;
    DoubleBuffer<SafetyState> safety_state_;
    DoubleBuffer<JointTargets> joint_targets_;
    DoubleBuffer<ControlStatus> status_;

    std::thread bus_thread_;
    std::thread estimator_thread_;
    std::thread control_thread_;
    std::thread safety_thread_;
    std::thread diag_thread_;
};
