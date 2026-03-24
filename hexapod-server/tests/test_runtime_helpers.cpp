#include "control_config.hpp"
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

} // namespace

int main() {
    if (!testFreshnessGateDecisionMatrix() ||
        !testStrictMetricsOnlyCountInvalidStreams() ||
        !testTimingMetricsTracksDeltaJitterAndAverage()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
