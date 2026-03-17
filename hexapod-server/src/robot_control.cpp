#include "robot_control.hpp"

#include "control_config.hpp"
#include "loop_timing.hpp"
#include "status_reporter.hpp"

#include <chrono>

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

    bus_thread_ = std::thread(&RobotControl::busLoop, this);
    estimator_thread_ = std::thread(&RobotControl::estimatorLoop, this);
    control_thread_ = std::thread(&RobotControl::controlLoop, this);
    safety_thread_ = std::thread(&RobotControl::safetyLoop, this);
    diag_thread_ = std::thread(&RobotControl::diagnosticsLoop, this);
}

void RobotControl::stop() {
    if (!running_.exchange(false)) return;

    if (logger_) {
        LOG_INFO(logger_, "Stopping robot control loops");
    }

    joinThread(bus_thread_);
    joinThread(estimator_thread_);
    joinThread(control_thread_);
    joinThread(safety_thread_);
    joinThread(diag_thread_);
}

void RobotControl::setMotionIntent(const MotionIntent& intent) {
    motion_intent_.write(intent);
}

ControlStatus RobotControl::getStatus() const {
    return status_.read();
}

void RobotControl::busLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        RawHardwareState raw{};
        if (!hw_->read(raw)) {
            raw.bus_ok = false;
        }
        raw_state_.write(raw);

        const JointTargets cmd = joint_targets_.read();
        (void)hw_->write(cmd);

        loop_timing::sleepUntil(cycle_start, control_config::kBusLoopPeriod);
    }
}

void RobotControl::estimatorLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const RawHardwareState raw = raw_state_.read();
        const EstimatedState est = estimator_->update(raw);
        estimated_state_.write(est);

        loop_timing::sleepUntil(cycle_start, control_config::kEstimatorLoopPeriod);
    }
}

void RobotControl::controlLoop() {
    uint64_t loop_counter = 0;

    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const EstimatedState est = estimated_state_.read();
        const MotionIntent intent = motion_intent_.read();
        const SafetyState safety_state = safety_state_.read();
        const bool bus_ok = raw_state_.read().bus_ok;

        const PipelineStepResult result = pipeline_.runStep(
            est,
            intent,
            safety_state,
            bus_ok,
            ++loop_counter);

        joint_targets_.write(result.joint_targets);
        status_.write(result.status);

        loop_timing::sleepUntil(cycle_start, control_config::kControlLoopPeriod);
    }
}

void RobotControl::safetyLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const RawHardwareState raw = raw_state_.read();
        const EstimatedState est = estimated_state_.read();
        const MotionIntent intent = motion_intent_.read();

        const SafetyState s = safety_.evaluate(raw, est, intent);
        safety_state_.write(s);

        loop_timing::sleepUntil(cycle_start, control_config::kSafetyLoopPeriod);
    }
}

void RobotControl::diagnosticsLoop() {
    while (running_.load()) {
        const auto st = status_.read();
        status_reporter::logStatus(logger_, st);

        std::this_thread::sleep_for(control_config::kDiagnosticsPeriod);
    }
}

void RobotControl::joinThread(std::thread& t) {
    if (t.joinable()) t.join();
}

