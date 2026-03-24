#include "robot_runtime.hpp"

RobotRuntime::RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config)
    : hw_(std::move(hw)),
      estimator_(std::move(estimator)),
      logger_(std::move(logger)),
      config_(config),
      pipeline_(config_.gait),
      safety_(config_.safety),
      freshness_policy_(config_.freshness),
      freshness_gate_(freshness_policy_),
      timing_metrics_(config_, control_dt_sum_us_, control_jitter_max_us_),
      diagnostics_reporter_(logger_, freshness_policy_) {}

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
    initial.sample_id = 1;

    motion_intent_.write(initial);
    status_.write(ControlStatus{});
    safety_state_.write(SafetyState{});
    joint_targets_.write(JointTargets{});
    control_loop_counter_.store(0);
    control_dt_sum_us_.store(0);
    control_jitter_max_us_.store(0);
    stale_intent_count_.store(0);
    stale_estimator_count_.store(0);
    raw_sample_seq_.store(0);
    intent_sample_seq_.store(1);
    freshness_gate_.reset();
    timing_metrics_.reset();

    return true;
}

void RobotRuntime::busStep() {
    RobotState raw{};
    if (!hw_->read(raw)) {
        raw.bus_ok = false;
    }
    if (raw.sample_id == 0) {
        raw.sample_id = raw_sample_seq_.fetch_add(1) + 1;
    }
    raw_state_.write(raw);

    const JointTargets cmd = joint_targets_.read();
    (void)hw_->write(cmd);
}

void RobotRuntime::estimatorStep() {
    const RobotState raw = raw_state_.read();
    const RobotState est = estimator_->update(raw);
    estimated_state_.write(est);
}

void RobotRuntime::controlStep() {
    const TimePointUs now = now_us();
    timing_metrics_.update(now);

    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
        RuntimeFreshnessGate::EvaluationMode::StrictControl, now, est, intent);
    freshness_gate_.recordStrictMetrics(freshness, stale_intent_count_, stale_estimator_count_);

    const uint64_t loop_counter = control_loop_counter_.fetch_add(1) + 1;
    const RuntimeFreshnessGate::Decision decision =
        freshness_gate_.computeControlDecision(freshness, bus_ok, loop_counter);
    if (!decision.allow_pipeline) {
        RuntimeFreshnessGate::maybeLogReject(logger_, freshness, est, intent);
        status_.write(decision.status);
        joint_targets_.write(decision.joint_targets);
        return;
    }

    const PipelineStepResult result = pipeline_.runStep(
        est,
        intent,
        safety_state,
        bus_ok,
        loop_counter);

    joint_targets_.write(result.joint_targets);
    status_.write(result.status);
}

void RobotRuntime::safetyStep() {
    const RobotState raw = raw_state_.read();
    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const TimePointUs now = now_us();
    const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
        RuntimeFreshnessGate::EvaluationMode::SafetyLenient, now, est, intent);

    const SafetySupervisor::FreshnessInputs freshness_inputs{
        freshness.estimator.valid,
        freshness.intent.valid};
    const SafetyState s = safety_.evaluate(raw, est, intent, freshness_inputs);
    safety_state_.write(s);
}

void RobotRuntime::diagnosticsStep() {
    const auto st = status_.read();
    const auto bridge_result = hw_ ? hw_->last_bridge_result() : std::nullopt;
    const uint64_t loops = control_loop_counter_.load();
    diagnostics_reporter_.report(st,
                                 bridge_result,
                                 loops,
                                 timing_metrics_.averageControlDtUs(loops),
                                 control_jitter_max_us_.load(),
                                 stale_intent_count_.load(),
                                 stale_estimator_count_.load());
}

void RobotRuntime::setMotionIntent(const MotionIntent& intent) {
    MotionIntent stamped_intent = intent;
    if (stamped_intent.timestamp_us.isZero()) {
        stamped_intent.timestamp_us = now_us();
    }
    if (stamped_intent.sample_id == 0) {
        stamped_intent.sample_id = intent_sample_seq_.fetch_add(1) + 1;
    }
    motion_intent_.write(stamped_intent);
}

void RobotRuntime::setMotionIntentForTest(const MotionIntent& intent) {
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
