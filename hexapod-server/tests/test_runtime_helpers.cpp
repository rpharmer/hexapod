#include "control_config.hpp"
#include "joint_oscillation_tracker.hpp"
#include "runtime_freshness_gate.hpp"
#include "runtime_timing_metrics.hpp"

#include <atomic>
#include <array>
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

    const auto allow = gate.computeControlDecision(fresh, true, 11, JointTargets{}, false, 0);
    if (!expect(allow.allow_pipeline, "fresh estimator+intent should allow pipeline")) {
        return false;
    }

    FreshnessPolicy::Evaluation stale_intent{};
    stale_intent.estimator.valid = true;
    stale_intent.intent.valid = false;
    JointTargets previous{};
    previous.leg_states[0].joint_state[0].pos_rad.value = 0.8;
    const auto reject_intent = gate.computeControlDecision(stale_intent, true, 12, previous, true, 1);
    if (!expect(!reject_intent.allow_pipeline, "stale intent should reject pipeline") ||
        !expect(reject_intent.status.active_fault == FaultCode::COMMAND_TIMEOUT,
                "stale intent should map to COMMAND_TIMEOUT") ||
        !expect(reject_intent.joint_targets.leg_states[0].joint_state[0].pos_rad.value == 0.8,
                "first reject should hold previous joint targets")) {
        return false;
    }

    FreshnessPolicy::Evaluation stale_both{};
    stale_both.estimator.valid = false;
    stale_both.intent.valid = false;
    const auto reject_both = gate.computeControlDecision(stale_both, false, 13, previous, true, 8);
    return expect(!reject_both.allow_pipeline, "stale estimator+intent should reject pipeline") &&
           expect(reject_both.status.active_fault == FaultCode::ESTIMATOR_INVALID,
                  "stale estimator should take precedence for fault mapping") &&
           expect(reject_both.status.bus_ok == false, "rejected status should preserve bus flag") &&
           expect(reject_both.status.loop_counter == 13, "rejected status should preserve loop counter") &&
           expect(reject_both.joint_targets.leg_states[0].joint_state[0].pos_rad.value < 0.8,
                  "extended reject streak should decay previous joint targets");
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

bool testTimingMetricsRollingWindowAndOverrunThresholds() {
    control_config::ControlConfig cfg{};
    cfg.loop_timing.control_loop_period = std::chrono::microseconds{1'000};

    std::atomic<uint64_t> dt_sum_us{0};
    std::atomic<uint64_t> jitter_max_us{0};
    RuntimeTimingMetrics timing(cfg, dt_sum_us, jitter_max_us);

    TimePointUs now{0};
    timing.update(now);

    for (int i = 0; i < 2; ++i) {
        now.value += 10'000;
        timing.update(now);
    }
    for (std::size_t i = 0; i < RuntimeTimingMetrics::kRollingWindowSize; ++i) {
        now.value += 2'000;
        timing.update(now);
    }

    LoopTimingRollingMetrics rolling = timing.rollingMetrics();
    if (!expect(rolling.sample_count == RuntimeTimingMetrics::kRollingWindowSize,
                "rolling metrics should retain only the fixed-size recent window") ||
        !expect(rolling.p50_control_dt_us == 2'000, "p50 should be based on the retained rolling window") ||
        !expect(rolling.p95_control_dt_us == 2'000, "p95 should be stable for uniform window samples") ||
        !expect(rolling.p99_control_dt_us == 2'000, "p99 should be stable for uniform window samples")) {
        return false;
    }

    timing.reset();
    now = TimePointUs{0};
    timing.update(now);
    const std::array<uint64_t, 11> dts_us{1'000, 1'500, 1'600, 1'700, 1'800, 1'900, 1'000, 2'000, 2'100, 2'200, 2'300};
    for (const uint64_t dt_us : dts_us) {
        now.value += dt_us;
        timing.update(now);
    }

    rolling = timing.rollingMetrics();
    return expect(rolling.warning_consecutive_overrun_threshold == RuntimeTimingMetrics::kWarningConsecutiveOverruns,
                  "rolling metrics should expose warning threshold") &&
           expect(rolling.hard_consecutive_overrun_threshold == RuntimeTimingMetrics::kHardConsecutiveOverruns,
                  "rolling metrics should expose hard threshold") &&
           expect(rolling.overrun_events_total == 9, "overrun counter should track all dt samples above period") &&
           expect(rolling.consecutive_overruns == 4,
                  "consecutive overrun counter should reset after on-time iteration and rebuild") &&
           expect(rolling.max_consecutive_overruns == 5,
                  "max consecutive overruns should capture longest overrun streak") &&
           expect(rolling.hard_overrun_escalation_crossings == 1,
                  "hard escalation crossings should count streak entries that hit the hard threshold");
}

bool testTimingMetricsControlledDelayOverrunEscalations() {
    control_config::ControlConfig cfg{};
    cfg.loop_timing.control_loop_period = std::chrono::microseconds{1'000};

    std::atomic<uint64_t> dt_sum_us{0};
    std::atomic<uint64_t> jitter_max_us{0};
    RuntimeTimingMetrics timing(cfg, dt_sum_us, jitter_max_us);

    TimePointUs now{0};
    timing.update(now);

    const std::array<uint64_t, 13> controlled_delays_us{
        900,  1'400, 1'500, 1'600, 1'700, 1'800, 900,  1'900, 2'000, 2'100, 2'200, 2'300, 2'400,
    };
    for (const uint64_t delay_us : controlled_delays_us) {
        now.value += delay_us;
        timing.update(now);
    }

    const LoopTimingRollingMetrics rolling = timing.rollingMetrics();
    return expect(rolling.overrun_events_total == 11,
                  "controlled delays above period should increment overrun event counter") &&
           expect(rolling.consecutive_overruns == 6,
                  "consecutive overrun tracking should reset on on-time delay then rebuild") &&
           expect(rolling.max_consecutive_overruns == 6,
                  "max consecutive overruns should retain longest post-reset streak") &&
           expect(rolling.hard_overrun_escalation_crossings == 2,
                  "hard escalation crossings should count once per threshold crossing event");
}


bool testJointOscillationTrackerIgnoresSubThresholdDirectionChanges() {
    JointOscillationTracker tracker(0.02, 0);
    JointTargets a{};
    JointTargets b{};
    JointTargets c{};

    b.leg_states[1].joint_state[1].pos_rad.value = 0.01;
    c.leg_states[1].joint_state[1].pos_rad.value = -0.005;

    tracker.observe(a, TimePointUs{0});
    tracker.observe(b, TimePointUs{20'000});
    tracker.observe(c, TimePointUs{40'000});

    const JointOscillationMetrics metrics = tracker.metrics();
    return expect(metrics.direction_reversal_events == 0,
                  "sub-threshold deltas should not count as oscillation reversals") &&
           expect(metrics.peak_joint_velocity_radps > 0.0,
                  "tracker should still accumulate peak velocity for motion samples");
}

bool testJointOscillationTrackerHandlesNonMonotonicTimestamps() {
    JointOscillationTracker tracker(0.01, 0);
    JointTargets a{};
    JointTargets b{};
    JointTargets c{};

    b.leg_states[0].joint_state[2].pos_rad.value = 0.05;
    c.leg_states[0].joint_state[2].pos_rad.value = -0.05;

    tracker.observe(a, TimePointUs{10'000});
    tracker.observe(b, TimePointUs{20'000});
    tracker.observe(c, TimePointUs{15'000});

    const JointOscillationMetrics metrics = tracker.metrics();
    return expect(metrics.direction_reversal_events == 0,
                  "non-monotonic timestamps should not produce reversal artifacts") &&
           expect(metrics.peak_joint_velocity_radps > 0.0,
                  "peak velocity should remain from valid monotonic intervals");
}

bool testJointOscillationTrackerCountsDirectionReversals() {
    JointOscillationTracker tracker(0.01, 0);
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
           expect(metrics.direction_reversal_events_by_joint[0] == 2,
                  "joint tracker should attribute reversals to the active joint") &&
           expect(metrics.peak_joint_velocity_radps > 2.0,
                  "joint tracker should report peak velocity from delta over dt") &&
           expect(metrics.peak_joint_velocity_radps_by_joint[0] > 2.0,
                  "joint tracker should retain per-joint peak velocity diagnostics");
}

bool testJointOscillationTrackerReversalCooldown() {
    JointOscillationTracker tracker(0.01, 50'000);
    JointTargets a{};
    JointTargets b{};
    JointTargets c{};
    JointTargets d{};

    b.leg_states[2].joint_state[1].pos_rad.value = 0.03;
    c.leg_states[2].joint_state[1].pos_rad.value = 0.00;
    d.leg_states[2].joint_state[1].pos_rad.value = 0.03;

    tracker.observe(a, TimePointUs{0});
    tracker.observe(b, TimePointUs{20'000});
    tracker.observe(c, TimePointUs{40'000});
    tracker.observe(d, TimePointUs{60'000});

    const JointOscillationMetrics metrics = tracker.metrics();
    return expect(metrics.direction_reversal_events == 1,
                  "reversal cooldown should suppress rapid flip-flop counting");
}

bool testJointOscillationTrackerWrapAwareDelta() {
    JointOscillationTracker tracker(0.05, 0);
    JointTargets a{};
    JointTargets b{};
    JointTargets c{};

    a.leg_states[0].joint_state[0].pos_rad.value = 3.12;
    b.leg_states[0].joint_state[0].pos_rad.value = -3.12;
    c.leg_states[0].joint_state[0].pos_rad.value = 3.11;

    tracker.observe(a, TimePointUs{0});
    tracker.observe(b, TimePointUs{20'000});
    tracker.observe(c, TimePointUs{40'000});

    const JointOscillationMetrics metrics = tracker.metrics();
    return expect(metrics.direction_reversal_events == 0,
                  "wrapped angle crossings should not count as direction reversals") &&
           expect(metrics.peak_joint_velocity_radps < 5.0,
                  "wrapped angle crossings should not generate implausible peak velocity");
}

} // namespace

int main() {
    if (!testFreshnessGateDecisionMatrix() ||
        !testStrictMetricsOnlyCountInvalidStreams() ||
        !testSafetyLenientEvaluationIgnoresAgeExpiry() ||
        !testTimingMetricsTracksDeltaJitterAndAverage() ||
        !testTimingMetricsRollingWindowAndOverrunThresholds() ||
        !testTimingMetricsControlledDelayOverrunEscalations() ||
        !testJointOscillationTrackerIgnoresSubThresholdDirectionChanges() ||
        !testJointOscillationTrackerHandlesNonMonotonicTimestamps() ||
        !testJointOscillationTrackerCountsDirectionReversals() ||
        !testJointOscillationTrackerReversalCooldown() ||
        !testJointOscillationTrackerWrapAwareDelta()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
