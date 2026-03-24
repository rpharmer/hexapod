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
    return freshness_policy_.evaluate(now, est, intent, update_tracking);
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
    uint64_t loop_counter) const {
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
        .joint_targets = JointTargets{},
    };
}

void RuntimeFreshnessGate::maybeLogReject(const std::shared_ptr<logging::AsyncLogger>& logger,
                                          const FreshnessPolicy::Evaluation& freshness,
                                          const RobotState& est,
                                          const MotionIntent& intent) {
    if (!logger) {
        return;
    }

    LOG_WARN(logger,
             "runtime.freshness_gate_reject estimator_valid=",
             freshness.estimator.valid ? 1 : 0,
             " intent_valid=",
             freshness.intent.valid ? 1 : 0,
             " est_age_us=",
             freshness.estimator.age_us,
             " intent_age_us=",
             freshness.intent.age_us,
             " est_sample_id=",
             est.sample_id,
             " intent_sample_id=",
             intent.sample_id);
}
