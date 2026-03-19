#include "robot_control.hpp"

RobotControl::RobotControl(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger)
    : runtime_(std::move(hw), std::move(estimator), logger), logger_(std::move(logger)) {}

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
            {control_config::kBusLoopPeriod, [this]() { runtime_.busStep(); }},
            {control_config::kEstimatorLoopPeriod, [this]() { runtime_.estimatorStep(); }},
            {control_config::kControlLoopPeriod, [this]() { runtime_.controlStep(); }},
            {control_config::kSafetyLoopPeriod, [this]() { runtime_.safetyStep(); }},
            {control_config::kDiagnosticsPeriod, [this]() { runtime_.diagnosticsStep(); }},
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

ControlStatus RobotControl::getStatus() const {
    return runtime_.getStatus();
}
