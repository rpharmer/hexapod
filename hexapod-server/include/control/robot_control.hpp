#pragma once

#include "control_config.hpp"
#include "imu_unit.hpp"
#include "loop_executor.hpp"
#include "logger.hpp"
#include "robot_runtime.hpp"

#include <atomic>
#include <memory>

class RobotControl {
public:
    RobotControl(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger,
                 control_config::ControlConfig config = {},
                 std::unique_ptr<hardware::IImuUnit> imu = nullptr);

    ~RobotControl();

    bool init();
    void start();
    void stop();

    void setMotionIntent(const MotionIntent& intent);
    bool setSimFaultToggles(const SimHardwareFaultToggles& toggles);
    void setRuntimeStageToggles(const RuntimeStageToggles& toggles);
    bool approveNextRuntimeStage();
    ControlStatus getStatus() const;

private:
    control_config::ControlConfig config_;
    RobotRuntime runtime_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    LoopExecutor loops_;
    std::atomic<bool> running_{false};
};
