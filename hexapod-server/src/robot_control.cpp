#include "robot_control.hpp"

RobotControl::RobotControl(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config)
    : config_(config),
      runtime_(std::move(hw), std::move(estimator), logger, config_),
      logger_(std::move(logger)) {}

RobotControl::~RobotControl() {
    stop();
}

bool RobotControl::init() {
    return runtime_.init();
}

void RobotControl::start() {
    if (running_.exchange(true)) return;

    if (logger_) {
        LOG_INFO(logger_, "Starting robot control loops");
    }

    loops_.start(
        {
            {config_.loop_timing.bus_loop_period, [this]() { runtime_.busStep(); }},
            {config_.loop_timing.estimator_loop_period, [this]() { runtime_.estimatorStep(); }},
            {config_.loop_timing.control_loop_period, [this]() { runtime_.controlStep(); }},
            {config_.loop_timing.safety_loop_period, [this]() { runtime_.safetyStep(); }},
            {config_.loop_timing.diagnostics_period, [this]() { runtime_.diagnosticsStep(); }},
        },
        running_);
}

void RobotControl::stop() {
    if (!running_.exchange(false)) return;

    if (logger_) {
        LOG_INFO(logger_, "Stopping robot control loops");
    }

    loops_.stop();
}

void RobotControl::setMotionIntent(const MotionIntent& intent) {
    runtime_.setMotionIntent(intent);
}

bool RobotControl::setSimFaultToggles(const SimHardwareFaultToggles& toggles) {
    return runtime_.setSimFaultToggles(toggles);
}

ControlStatus RobotControl::getStatus() const {
    return runtime_.getStatus();
}
