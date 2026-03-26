#include "control_config.hpp"
#include "joint_oscillation_tracker.hpp"
#include "runtime_freshness_gate.hpp"
#include "runtime_timing_metrics.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testFreshnessGateDecisionMatrix() {
    FreshnessPolicy policy{};
    RuntimeFreshnessGate gate(policy);

    FreshnessPolicy::Evaluation fresh{};
    fresh.estimator.valid = true;
    fresh.intent.valid = true;

    const auto allow = gate.computeControlDecision(fresh, true, 11);
    if (!expect(allow.allow_pipeline, "fresh estimator+intent should allow pipeline")) {
        return false;
    }

    FreshnessPolicy::Evaluation stale_intent{};
    stale_intent.estimator.valid = true;
    stale_intent.intent.valid = false;
    const auto reject_intent = gate.computeControlDecision(stale_intent, true, 12);
    if (!expect(!reject_intent.allow_pipeline, "stale intent should reject pipeline") ||
        !expect(reject_intent.status.active_fault == FaultCode::COMMAND_TIMEOUT,
                "stale intent should map to COMMAND_TIMEOUT")) {
        return false;
    }

    FreshnessPolicy::Evaluation stale_both{};
    stale_both.estimator.valid = false;
    stale_both.intent.valid = false;
    const auto reject_both = gate.computeControlDecision(stale_both, false, 13);
    return expect(!reject_both.allow_pipeline, "stale estimator+intent should reject pipeline") &&
           expect(reject_both.status.active_fault == FaultCode::ESTIMATOR_INVALID,
                  "stale estimator should take precedence for fault mapping") &&
           expect(reject_both.status.bus_ok == false, "rejected status should preserve bus flag") &&
           expect(reject_both.status.loop_counter == 13, "rejected status should preserve loop counter");
}

bool testStrictMetricsOnlyCountInvalidStreams() {
    FreshnessPolicy policy{};
    RuntimeFreshnessGate gate(policy);
    std::atomic<uint64_t> stale_intent_count{0};
    std::atomic<uint64_t> stale_estimator_count{0};

    FreshnessPolicy::Evaluation fresh{};
    fresh.estimator.valid = true;
    fresh.intent.valid = true;
    gate.recordStrictMetrics(fresh, stale_intent_count, stale_estimator_count);

    FreshnessPolicy::Evaluation stale_estimator{};
    stale_estimator.estimator.valid = false;
    stale_estimator.intent.valid = true;
    gate.recordStrictMetrics(stale_estimator, stale_intent_count, stale_estimator_count);

    FreshnessPolicy::Evaluation stale_both{};
    stale_both.estimator.valid = false;
    stale_both.intent.valid = false;
    gate.recordStrictMetrics(stale_both, stale_intent_count, stale_estimator_count);

    return expect(stale_intent_count.load() == 1, "strict metrics should count stale intents") &&
           expect(stale_estimator_count.load() == 2, "strict metrics should count stale estimators");
}

bool testSafetyLenientEvaluationIgnoresAgeExpiry() {
    control_config::FreshnessConfig cfg{};
    cfg.estimator.max_allowed_age_us = DurationUs{1'000};
    cfg.intent.max_allowed_age_us = DurationUs{1'000};
    FreshnessPolicy policy(cfg);
    RuntimeFreshnessGate gate(policy);

    const TimePointUs now{1'000'000};
    RobotState est{};
    est.sample_id = 5;
    est.timestamp_us = TimePointUs{1};
    MotionIntent intent{};
    intent.sample_id = 5;
    intent.timestamp_us = TimePointUs{1};

    const auto strict = gate.evaluate(RuntimeFreshnessGate::EvaluationMode::StrictControl, now, est, intent);
    const auto lenient = gate.evaluate(RuntimeFreshnessGate::EvaluationMode::SafetyLenient, now, est, intent);

    return expect(!strict.estimator.valid && strict.estimator.stale_age,
                  "strict freshness evaluation should reject stale estimator age") &&
           expect(!strict.intent.valid && strict.intent.stale_age,
                  "strict freshness evaluation should reject stale intent age") &&
           expect(lenient.estimator.valid && !lenient.estimator.stale_age,
                  "safety lenient mode should ignore estimator age expiry") &&
           expect(lenient.intent.valid && !lenient.intent.stale_age,
                  "safety lenient mode should ignore intent age expiry");
}

bool testTimingMetricsTracksDeltaJitterAndAverage() {
    control_config::ControlConfig cfg{};
    cfg.loop_timing.control_loop_period = std::chrono::microseconds{2'000};

    std::atomic<uint64_t> dt_sum_us{0};
    std::atomic<uint64_t> jitter_max_us{0};
    RuntimeTimingMetrics timing(cfg, dt_sum_us, jitter_max_us);

    timing.update(TimePointUs{10'000});
    timing.update(TimePointUs{12'500});
    timing.update(TimePointUs{15'000});

    if (!expect(dt_sum_us.load() == 5'000, "timing metrics should sum control dt intervals") ||
        !expect(jitter_max_us.load() == 500, "timing metrics should track max absolute jitter")) {
        return false;
    }

    const uint64_t avg = timing.averageControlDtUs(3);
    if (!expect(avg == 2'500, "average dt should divide by completed intervals")) {
        return false;
    }

    timing.reset();
    timing.update(TimePointUs{30'000});
    return expect(dt_sum_us.load() == 5'000,
                  "first update after reset should establish baseline without changing sums");
}

bool testJointOscillationTrackerCountsDirectionReversals() {
    JointOscillationTracker tracker(0.01);
    JointTargets a{};
    JointTargets b{};
    JointTargets c{};
    JointTargets d{};

    b.leg_states[0].joint_state[0].pos_rad.value = 0.05;
    c.leg_states[0].joint_state[0].pos_rad.value = 0.00;
    d.leg_states[0].joint_state[0].pos_rad.value = 0.04;

    tracker.observe(a, TimePointUs{0});
    tracker.observe(b, TimePointUs{20'000});
    tracker.observe(c, TimePointUs{40'000});
    tracker.observe(d, TimePointUs{60'000});

    const JointOscillationMetrics metrics = tracker.metrics();
    return expect(metrics.direction_reversal_events == 2,
                  "joint tracker should count significant direction reversals") &&
           expect(metrics.peak_joint_velocity_radps > 2.0,
                  "joint tracker should report peak velocity from delta over dt");
}

} // namespace

int main() {
    if (!testFreshnessGateDecisionMatrix() ||
        !testStrictMetricsOnlyCountInvalidStreams() ||
        !testSafetyLenientEvaluationIgnoresAgeExpiry() ||
        !testTimingMetricsTracksDeltaJitterAndAverage() ||
        !testJointOscillationTrackerCountsDirectionReversals()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
