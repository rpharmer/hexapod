#include "robot_runtime.hpp"

#include <algorithm>
#include <atomic>

#include "status_reporter.hpp"

namespace {

void updateControlTimingMetrics(const control_config::ControlConfig& config,
                                TimePointUs now,
                                TimePointUs& last_control_step_us,
                                std::atomic<uint64_t>& control_dt_sum_us,
                                std::atomic<uint64_t>& control_jitter_max_us) {
    if (last_control_step_us.isZero()) {
        last_control_step_us = now;
        return;
    }

    const uint64_t dt_us = static_cast<uint64_t>((now - last_control_step_us).value);
    control_dt_sum_us.fetch_add(dt_us);
    const uint64_t target_us = static_cast<uint64_t>(config.loop_timing.control_loop_period.count());
    const uint64_t jitter_us = (dt_us > target_us) ? (dt_us - target_us) : (target_us - dt_us);
    uint64_t current_max = control_jitter_max_us.load();
    while (jitter_us > current_max &&
           !control_jitter_max_us.compare_exchange_weak(current_max, jitter_us)) {
    }
    last_control_step_us = now;
}

ControlStatus makeFreshnessGateRejectedStatus(bool estimator_fresh,
                                              bool bus_ok,
                                              std::atomic<uint64_t>& control_loop_counter) {
    ControlStatus status{};
    status.active_mode = RobotMode::SAFE_IDLE;
    status.estimator_valid = estimator_fresh;
    status.bus_ok = bus_ok;
    status.active_fault = estimator_fresh ? FaultCode::COMMAND_TIMEOUT : FaultCode::ESTIMATOR_INVALID;
    status.loop_counter = control_loop_counter.fetch_add(1) + 1;
    return status;
}

void maybeLogFreshnessGateReject(const std::shared_ptr<logging::AsyncLogger>& logger,
                                 bool estimator_fresh,
                                 bool intent_fresh,
                                 const FreshnessPolicy::Evaluation& freshness,
                                 const RobotState& est,
                                 const MotionIntent& intent) {
    if (!logger) {
        return;
    }

    LOG_WARN(logger,
             "runtime.freshness_gate_reject estimator_valid=",
             estimator_fresh ? 1 : 0,
             " intent_valid=",
             intent_fresh ? 1 : 0,
             " est_age_us=",
             freshness.estimator.age_us,
             " intent_age_us=",
             freshness.intent.age_us,
             " est_sample_id=",
             est.sample_id,
             " intent_sample_id=",
             intent.sample_id);
}

} // namespace

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
      freshness_policy_(config_.freshness) {}

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
    freshness_policy_.reset();
    latest_freshness_ = {};
    last_control_step_us_ = TimePointUs{};

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
    updateControlTimingMetrics(config_, now, last_control_step_us_, control_dt_sum_us_, control_jitter_max_us_);

    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    latest_freshness_ = freshness_policy_.evaluate(now, est, intent);
    const bool estimator_fresh = latest_freshness_.estimator.valid;
    const bool intent_fresh = latest_freshness_.intent.valid;

    if (!intent_fresh) {
        stale_intent_count_.fetch_add(1);
    }
    if (!estimator_fresh) {
        stale_estimator_count_.fetch_add(1);
    }

    if (!estimator_fresh || !intent_fresh) {
        maybeLogFreshnessGateReject(
            logger_, estimator_fresh, intent_fresh, latest_freshness_, est, intent);
        const ControlStatus status = makeFreshnessGateRejectedStatus(
            estimator_fresh, bus_ok, control_loop_counter_);
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
    const RobotState raw = raw_state_.read();
    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const TimePointUs now = now_us();
    latest_freshness_ = freshness_policy_.evaluate(now, est, intent, false);

    const SafetySupervisor::FreshnessInputs freshness_inputs{
        latest_freshness_.estimator.valid,
        latest_freshness_.intent.valid};
    const SafetyState s = safety_.evaluate(raw, est, intent, freshness_inputs);
    safety_state_.write(s);
}

void RobotRuntime::diagnosticsStep() {
    const auto st = status_.read();
    status_reporter::logStatus(logger_, st);
    if (logger_) {
        const auto& estimator_diag = freshness_policy_.estimatorDiagnostics();
        const auto& intent_diag = freshness_policy_.intentDiagnostics();
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
                 stale_estimator_count_.load(),
                 " estimator_diag={stale_age:",
                 estimator_diag.stale_age_count,
                 ",missing_ts:",
                 estimator_diag.missing_timestamp_count,
                 ",invalid_sample:",
                 estimator_diag.invalid_sample_id_count,
                 ",non_monotonic_sample:",
                 estimator_diag.non_monotonic_sample_id_count,
                 "} intent_diag={stale_age:",
                 intent_diag.stale_age_count,
                 ",missing_ts:",
                 intent_diag.missing_timestamp_count,
                 ",invalid_sample:",
                 intent_diag.invalid_sample_id_count,
                 ",non_monotonic_sample:",
                 intent_diag.non_monotonic_sample_id_count,
                 "}");
    }
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
