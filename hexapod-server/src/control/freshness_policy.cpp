#include "freshness_policy.hpp"

FreshnessPolicy::FreshnessPolicy(control_config::FreshnessConfig config)
    : config_(config) {}

void FreshnessPolicy::reset() {
    last_estimator_sample_id_ = 0;
    last_intent_sample_id_ = 0;
    estimator_diag_ = {};
    intent_diag_ = {};
}

FreshnessPolicy::Evaluation FreshnessPolicy::evaluate(TimePointUs now,
                                                      const RobotState& est,
                                                      const MotionIntent& intent,
                                                      bool update_tracking) {
    uint64_t estimator_last_sample_id = last_estimator_sample_id_;
    uint64_t intent_last_sample_id = last_intent_sample_id_;
    StreamDiagnostics estimator_diag = estimator_diag_;
    StreamDiagnostics intent_diag = intent_diag_;

    Evaluation eval{};
    eval.estimator = evaluateStream(
        now,
        est.timestamp_us,
        est.sample_id,
        estimator_last_sample_id,
        config_.estimator,
        estimator_diag,
        update_tracking);
    eval.intent = evaluateStream(
        now,
        intent.timestamp_us,
        intent.sample_id,
        intent_last_sample_id,
        config_.intent,
        intent_diag,
        update_tracking);

    if (update_tracking) {
        last_estimator_sample_id_ = estimator_last_sample_id;
        last_intent_sample_id_ = intent_last_sample_id;
        estimator_diag_ = estimator_diag;
        intent_diag_ = intent_diag;
    }

    return eval;
}

const FreshnessPolicy::StreamDiagnostics& FreshnessPolicy::estimatorDiagnostics() const {
    return estimator_diag_;
}

const FreshnessPolicy::StreamDiagnostics& FreshnessPolicy::intentDiagnostics() const {
    return intent_diag_;
}

FreshnessPolicy::StreamResult FreshnessPolicy::evaluateStream(
    TimePointUs now,
    TimePointUs timestamp_us,
    uint64_t sample_id,
    uint64_t& last_sample_id,
    const control_config::StreamFreshnessConfig& freshness,
    StreamDiagnostics& diagnostics,
    bool update_tracking) const {
    StreamResult result{};

    if (freshness.require_timestamp && timestamp_us.isZero()) {
        result.valid = false;
        result.missing_timestamp = true;
        if (update_tracking) {
            ++diagnostics.missing_timestamp_count;
        }
    }

    if (!timestamp_us.isZero()) {
        result.age_us = (now - timestamp_us).value;
        if (result.age_us > freshness.max_allowed_age_us.value) {
            result.valid = false;
            result.stale_age = true;
            if (update_tracking) {
                ++diagnostics.stale_age_count;
            }
        }
    }

    if (freshness.require_nonzero_sample_id && sample_id == 0) {
        result.valid = false;
        result.invalid_sample_id = true;
        if (update_tracking) {
            ++diagnostics.invalid_sample_id_count;
        }
    }

    if (freshness.require_monotonic_sample_id && sample_id != 0 &&
        last_sample_id != 0 && sample_id < last_sample_id) {
        result.valid = false;
        result.non_monotonic_sample_id = true;
        if (update_tracking) {
            ++diagnostics.non_monotonic_sample_id_count;
        }
    }

    if (update_tracking && sample_id != 0) {
        last_sample_id = sample_id;
    }

    return result;
}
