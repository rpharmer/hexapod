#include "runtime_freshness_gate.hpp"

namespace {

ControlStatus makeFreshnessGateRejectedStatus(bool estimator_fresh,
                                              bool bus_ok,
                                              uint64_t loop_counter) {
    ControlStatus status{};
    status.active_mode = RobotMode::SAFE_IDLE;
    status.estimator_valid = estimator_fresh;
    status.bus_ok = bus_ok;
    status.active_fault = estimator_fresh ? FaultCode::COMMAND_TIMEOUT : FaultCode::ESTIMATOR_INVALID;
    status.loop_counter = loop_counter;
    return status;
}

JointTargets computeRejectTargets(const JointTargets& last_joint_targets,
                                  bool has_last_joint_targets,
                                  uint64_t consecutive_rejects) {
    (void)consecutive_rejects;
    if (!has_last_joint_targets) {
        return JointTargets{};
    }
    // Hold the last pipeline pose for the whole reject streak. Decaying toward zero
    // caused rapid joint collapses whenever estimator/intent were briefly stale during
    // locomotion (visible on hardware and in UDP joint telemetry).
    return last_joint_targets;
}

} // namespace

RuntimeFreshnessGate::RuntimeFreshnessGate(FreshnessPolicy& freshness_policy)
    : freshness_policy_(freshness_policy) {}

void RuntimeFreshnessGate::reset() {
    freshness_policy_.reset();
}

FreshnessPolicy::Evaluation RuntimeFreshnessGate::evaluate(EvaluationMode mode,
                                                           TimePointUs now,
                                                           const RobotState& est,
                                                           const MotionIntent& intent) {
    const bool update_tracking = (mode == EvaluationMode::StrictControl);
    const bool enforce_age = (mode == EvaluationMode::StrictControl);
    return freshness_policy_.evaluate(now, est, intent, update_tracking, enforce_age);
}

void RuntimeFreshnessGate::recordStrictMetrics(const FreshnessPolicy::Evaluation& freshness,
                                               std::atomic<uint64_t>& stale_intent_count,
                                               std::atomic<uint64_t>& stale_estimator_count) const {
    if (!freshness.intent.valid) {
        stale_intent_count.fetch_add(1);
    }
    if (!freshness.estimator.valid) {
        stale_estimator_count.fetch_add(1);
    }
}

RuntimeFreshnessGate::Decision RuntimeFreshnessGate::computeControlDecision(
    const FreshnessPolicy::Evaluation& freshness,
    bool bus_ok,
    uint64_t loop_counter,
    const JointTargets& last_joint_targets,
    bool has_last_joint_targets,
    uint64_t consecutive_rejects) const {
    const bool estimator_fresh = freshness.estimator.valid;
    const bool intent_fresh = freshness.intent.valid;
    if (estimator_fresh && intent_fresh) {
        return Decision{
            .allow_pipeline = true,
            .status = {},
            .joint_targets = {},
        };
    }

    return Decision{
        .allow_pipeline = false,
        .status = makeFreshnessGateRejectedStatus(estimator_fresh, bus_ok, loop_counter),
        .joint_targets = computeRejectTargets(last_joint_targets, has_last_joint_targets, consecutive_rejects),
    };
}

void RuntimeFreshnessGate::maybeLogReject(const std::shared_ptr<logging::AsyncLogger>& logger,
                                          TimePointUs now,
                                          const FreshnessPolicy::Evaluation& freshness,
                                          const RobotState& est,
                                          const MotionIntent& intent,
                                          uint64_t loop_counter,
                                          uint64_t consecutive_rejects) {
    if (!logger) {
        return;
    }

    LOG_WARN(logger,
             "runtime.freshness_gate_reject estimator_valid=",
             freshness.estimator.valid ? 1 : 0,
             " estimator_stale_age=",
             freshness.estimator.stale_age ? 1 : 0,
             " estimator_invalid_sample_id=",
             freshness.estimator.invalid_sample_id ? 1 : 0,
             " estimator_non_monotonic_id=",
             freshness.estimator.non_monotonic_sample_id ? 1 : 0,
             " intent_valid=",
             freshness.intent.valid ? 1 : 0,
             " intent_stale_age=",
             freshness.intent.stale_age ? 1 : 0,
             " intent_invalid_sample_id=",
             freshness.intent.invalid_sample_id ? 1 : 0,
             " intent_non_monotonic_id=",
             freshness.intent.non_monotonic_sample_id ? 1 : 0,
             " loop_counter=",
             loop_counter,
             " now_ts_us=",
             now.value,
             " consecutive_rejects=",
             consecutive_rejects,
             " est_age_us=",
             freshness.estimator.age_us,
             " est_ts_us=",
             est.timestamp_us.value,
             " intent_age_us=",
             freshness.intent.age_us,
             " intent_ts_us=",
             intent.timestamp_us.value,
             " est_sample_id=",
             est.sample_id,
             " intent_sample_id=",
             intent.sample_id);
}
