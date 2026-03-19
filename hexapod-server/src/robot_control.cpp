#include "robot_control.hpp"

#include "control_config.hpp"
#include "status_reporter.hpp"

#include <chrono>
#include <vector>

using namespace std::chrono_literals;

RobotControl::RobotControl(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger)
    : hw_(std::move(hw)), estimator_(std::move(estimator)), logger_(std::move(logger)) {}

RobotControl::~RobotControl() {
    stop();
}

bool RobotControl::init() {
    if (!hw_ || !estimator_) {
        if (logger_) {
            LOG_ERROR(logger_, "Missing components");
        }
        return false;
    }
    if (!hw_->init()) {
        if (logger_) {
            LOG_ERROR(logger_, "Hardware Bridge init failed");
        }
        return false;
    }

    MotionIntent initial{};
    initial.requested_mode = RobotMode::SAFE_IDLE;
    initial.timestamp_us = now_us();

    motion_intent_.write(initial);
    status_.write(ControlStatus{});
    safety_state_.write(SafetyState{});
    joint_targets_.write(JointTargets{});

    return true;
}

void RobotControl::start() {
    if (running_.exchange(true)) return;

    if (logger_) {
        LOG_INFO(logger_, "Starting robot control loops");
    }

    control_loop_counter_.store(0);

    loops_.start(
        {
            {control_config::kBusLoopPeriod, [this]() { busStep(); }},
            {control_config::kEstimatorLoopPeriod, [this]() { estimatorStep(); }},
            {control_config::kControlLoopPeriod, [this]() { controlStep(); }},
            {control_config::kSafetyLoopPeriod, [this]() { safetyStep(); }},
            {control_config::kDiagnosticsPeriod, [this]() { diagnosticsStep(); }},
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
    motion_intent_.write(intent);
}

ControlStatus RobotControl::getStatus() const {
    return status_.read();
}

void RobotControl::busStep() {
    RawHardwareState raw{};
    if (!hw_->read(raw)) {
        raw.bus_ok = false;
    }
    raw_state_.write(raw);

    const JointTargets cmd = joint_targets_.read();
    (void)hw_->write(cmd);
}

void RobotControl::estimatorStep() {
    const RawHardwareState raw = raw_state_.read();
    const EstimatedState est = estimator_->update(raw);
    estimated_state_.write(est);
}

void RobotControl::controlStep() {
    const EstimatedState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    const PipelineStepResult result = pipeline_.runStep(
        est,
        intent,
        safety_state,
        bus_ok,
        control_loop_counter_.fetch_add(1) + 1);

    joint_targets_.write(result.joint_targets);
    status_.write(result.status);
}

void RobotControl::safetyStep() {
    const RawHardwareState raw = raw_state_.read();
    const EstimatedState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();

    const SafetyState s = safety_.evaluate(raw, est, intent);
    safety_state_.write(s);
}

void RobotControl::diagnosticsStep() {
    const auto st = status_.read();
    status_reporter::logStatus(logger_, st);
}
