#include "robot_runtime.hpp"

#include <algorithm>

#include "status_reporter.hpp"

RobotRuntime::RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config)
    : hw_(std::move(hw)),
      estimator_(std::move(estimator)),
      logger_(std::move(logger)),
      config_(config),
      pipeline_(config_.gait),
      safety_(config_.safety) {}

bool RobotRuntime::init() {
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
    control_loop_counter_.store(0);
    control_dt_sum_us_.store(0);
    control_jitter_max_us_.store(0);
    stale_intent_count_.store(0);
    last_control_step_us_ = TimePointUs{};

    return true;
}

void RobotRuntime::busStep() {
    RawHardwareState raw{};
    if (!hw_->read(raw)) {
        raw.bus_ok = false;
    }
    raw_state_.write(raw);

    const JointTargets cmd = joint_targets_.read();
    (void)hw_->write(cmd);
}

void RobotRuntime::estimatorStep() {
    const RawHardwareState raw = raw_state_.read();
    const EstimatedState est = estimator_->update(raw);
    estimated_state_.write(est);
}

void RobotRuntime::controlStep() {
    const TimePointUs now = now_us();
    if (!last_control_step_us_.isZero()) {
        const uint64_t dt_us = static_cast<uint64_t>((now - last_control_step_us_).value);
        control_dt_sum_us_.fetch_add(dt_us);
        const uint64_t target_us = static_cast<uint64_t>(config_.loop_timing.control_loop_period.count());
        const uint64_t jitter_us = (dt_us > target_us) ? (dt_us - target_us) : (target_us - dt_us);
        uint64_t current_max = control_jitter_max_us_.load();
        while (jitter_us > current_max &&
               !control_jitter_max_us_.compare_exchange_weak(current_max, jitter_us)) {
        }
    }
    last_control_step_us_ = now;

    const EstimatedState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    if (!intent.timestamp_us.isZero() && ((now - intent.timestamp_us) > config_.safety.command_timeout_us)) {
        stale_intent_count_.fetch_add(1);
    }

    const PipelineStepResult result = pipeline_.runStep(
        est,
        intent,
        safety_state,
        bus_ok,
        control_loop_counter_.fetch_add(1) + 1);

    joint_targets_.write(result.joint_targets);
    status_.write(result.status);
}

void RobotRuntime::safetyStep() {
    const RawHardwareState raw = raw_state_.read();
    const EstimatedState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();

    const SafetyState s = safety_.evaluate(raw, est, intent);
    safety_state_.write(s);
}

void RobotRuntime::diagnosticsStep() {
    const auto st = status_.read();
    status_reporter::logStatus(logger_, st);
    if (logger_) {
        const uint64_t loops = control_loop_counter_.load();
        const uint64_t dt_sum_us = control_dt_sum_us_.load();
        const uint64_t avg_dt_us = (loops > 1) ? (dt_sum_us / (loops - 1)) : 0;
        LOG_INFO(logger_,
                 "runtime.metrics loops=",
                 loops,
                 " avg_control_dt_us=",
                 avg_dt_us,
                 " max_control_jitter_us=",
                 control_jitter_max_us_.load(),
                 " stale_intent_events=",
                 stale_intent_count_.load());
    }
}

void RobotRuntime::setMotionIntent(const MotionIntent& intent) {
    motion_intent_.write(intent);
}

bool RobotRuntime::setSimFaultToggles(const SimHardwareFaultToggles& toggles) {
    auto* sim_hw = dynamic_cast<SimHardwareBridge*>(hw_.get());
    if (sim_hw == nullptr) {
        return false;
    }

    sim_hw->setFaultToggles(toggles);
    return true;
}

ControlStatus RobotRuntime::getStatus() const {
    return status_.read();
}
