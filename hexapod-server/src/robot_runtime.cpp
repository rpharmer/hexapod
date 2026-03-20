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

    motion_intent_.write(StreamSample<MotionIntent>{
        initial,
        intent_sample_id_.fetch_add(1) + 1,
        initial.timestamp_us});
    status_.write(ControlStatus{});
    safety_state_.write(SafetyState{});
    joint_targets_.write(JointTargets{});
    control_loop_counter_.store(0);
    control_dt_sum_us_.store(0);
    control_jitter_max_us_.store(0);
    stale_intent_count_.store(0);
    stale_estimator_count_.store(0);
    raw_sample_id_.store(0);
    estimator_sample_id_.store(0);
    intent_sample_id_.store(1);
    last_control_step_us_ = TimePointUs{};

    return true;
}

void RobotRuntime::busStep() {
    RawHardwareState raw{};
    if (!hw_->read(raw)) {
        raw.bus_ok = false;
    }
    raw_state_.write(StreamSample<RawHardwareState>{
        raw,
        raw_sample_id_.fetch_add(1) + 1,
        now_us()});

    const JointTargets cmd = joint_targets_.read();
    (void)hw_->write(cmd);
}

void RobotRuntime::estimatorStep() {
    const StreamSample<RawHardwareState> raw = raw_state_.read();
    const EstimatedState est = estimator_->update(raw.value);
    estimated_state_.write(StreamSample<EstimatedState>{
        est,
        estimator_sample_id_.fetch_add(1) + 1,
        now_us()});
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

    const StreamSample<EstimatedState> est_sample = estimated_state_.read();
    const StreamSample<MotionIntent> intent_sample = motion_intent_.read();
    const EstimatedState est = est_sample.value;
    const MotionIntent intent = intent_sample.value;
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().value.bus_ok;

    const bool estimator_fresh =
        (!est_sample.timestamp_us.isZero()) &&
        ((now - est_sample.timestamp_us).value <= config_.freshness.max_estimator_age_us.value);
    const bool intent_fresh =
        (!intent_sample.timestamp_us.isZero()) &&
        ((now - intent_sample.timestamp_us).value <= config_.freshness.max_intent_age_us.value);

    if (!intent_fresh) {
        stale_intent_count_.fetch_add(1);
    }
    if (!estimator_fresh) {
        stale_estimator_count_.fetch_add(1);
    }

    if (!estimator_fresh || !intent_fresh) {
        ControlStatus status{};
        status.active_mode = RobotMode::SAFE_IDLE;
        status.estimator_valid = estimator_fresh;
        status.bus_ok = bus_ok;
        status.active_fault = estimator_fresh ? FaultCode::COMMAND_TIMEOUT : FaultCode::ESTIMATOR_INVALID;
        status.loop_counter = control_loop_counter_.fetch_add(1) + 1;
        status_.write(status);
        joint_targets_.write(JointTargets{});
        return;
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
    const RawHardwareState raw = raw_state_.read().value;
    const EstimatedState est = estimated_state_.read().value;
    const MotionIntent intent = motion_intent_.read().value;

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
                 stale_intent_count_.load(),
                 " stale_estimator_events=",
                 stale_estimator_count_.load());
    }
}

void RobotRuntime::setMotionIntent(const MotionIntent& intent) {
    MotionIntent stamped_intent = intent;
    if (stamped_intent.timestamp_us.isZero()) {
        stamped_intent.timestamp_us = now_us();
    }
    motion_intent_.write(StreamSample<MotionIntent>{
        stamped_intent,
        intent_sample_id_.fetch_add(1) + 1,
        stamped_intent.timestamp_us});
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
