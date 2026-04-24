#pragma once

#include "control_config.hpp"
#include "loop_executor.hpp"
#include "logger.hpp"
#include "replay_logger.hpp"
#include "robot_runtime.hpp"

#include <atomic>
#include <memory>

class RobotControl {
public:
    RobotControl(std::unique_ptr<IHardwareBridge> hw,
                 std::unique_ptr<IEstimator> estimator,
                 std::shared_ptr<logging::AsyncLogger> logger,
                 control_config::ControlConfig config = {},
                 std::unique_ptr<replay::IReplayLogger> replay_logger = replay::makeNoopReplayLogger());

    ~RobotControl();

    bool init();
    void start();
    void stop();

    void setMotionIntent(const MotionIntent& intent);
    bool setSimFaultToggles(const SimHardwareFaultToggles& toggles);
    void setNavigationManager(std::unique_ptr<NavigationManager> navigation_manager);
    ControlStatus getStatus() const;
    SafetyState getSafetyState() const;

    [[nodiscard]] RobotState estimatedSnapshot() const { return runtime_.estimatedSnapshot(); }
    [[nodiscard]] NavigationManager* navigationManager() { return runtime_.navigationManager(); }
    [[nodiscard]] const NavigationManager* navigationManager() const { return runtime_.navigationManager(); }

private:
    control_config::ControlConfig config_;
    RobotRuntime runtime_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    LoopExecutor loops_;
    std::atomic<bool> running_{false};
};
