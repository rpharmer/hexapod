#include "freshness_policy.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

control_config::FreshnessConfig strictFreshnessConfig() {
    control_config::FreshnessConfig cfg{};
    cfg.estimator.max_allowed_age_us = DurationUs{1'000};
    cfg.intent.max_allowed_age_us = DurationUs{1'000};
    cfg.estimator.require_timestamp = true;
    cfg.intent.require_timestamp = true;
    cfg.estimator.require_nonzero_sample_id = true;
    cfg.intent.require_nonzero_sample_id = true;
    cfg.estimator.require_monotonic_sample_id = true;
    cfg.intent.require_monotonic_sample_id = true;
    return cfg;
}

bool testNonMonotonicSampleIds() {
    FreshnessPolicy policy(strictFreshnessConfig());
    policy.reset();

    const TimePointUs now{50'000};
    RobotState est_good{};
    est_good.sample_id = 10;
    est_good.timestamp_us = TimePointUs{49'500};

    MotionIntent intent_good{};
    intent_good.sample_id = 10;
    intent_good.timestamp_us = TimePointUs{49'500};

    const auto baseline = policy.evaluate(now, est_good, intent_good);
    if (!expect(baseline.estimator.valid && baseline.intent.valid,
                "baseline sample ids should be valid")) {
        return false;
    }

    RobotState est_bad = est_good;
    est_bad.sample_id = 9;
    MotionIntent intent_bad = intent_good;
    intent_bad.sample_id = 9;

    const auto decreasing = policy.evaluate(now, est_bad, intent_bad);
    return expect(!decreasing.estimator.valid && decreasing.estimator.non_monotonic_sample_id,
                  "decreasing estimator sample id should be invalid") &&
           expect(!decreasing.intent.valid && decreasing.intent.non_monotonic_sample_id,
                  "decreasing intent sample id should be invalid");
}

bool testMissingTimestamps() {
    FreshnessPolicy policy(strictFreshnessConfig());
    policy.reset();

    RobotState est{};
    est.sample_id = 42;
    est.timestamp_us = TimePointUs{};

    MotionIntent intent{};
    intent.sample_id = 42;
    intent.timestamp_us = TimePointUs{};

    const auto eval = policy.evaluate(TimePointUs{100'000}, est, intent);
    return expect(!eval.estimator.valid && eval.estimator.missing_timestamp,
                  "missing estimator timestamp should be invalid") &&
           expect(!eval.intent.valid && eval.intent.missing_timestamp,
                  "missing intent timestamp should be invalid");
}

bool testBoundaryAgeThresholds() {
    FreshnessPolicy policy(strictFreshnessConfig());
    policy.reset();

    const TimePointUs now{200'000};

    RobotState est_just_valid{};
    est_just_valid.sample_id = 1;
    est_just_valid.timestamp_us = TimePointUs{199'000}; // age == 1000us
    MotionIntent intent_just_valid{};
    intent_just_valid.sample_id = 1;
    intent_just_valid.timestamp_us = TimePointUs{199'000}; // age == 1000us

    const auto just_valid = policy.evaluate(now, est_just_valid, intent_just_valid);
    if (!expect(just_valid.estimator.valid && !just_valid.estimator.stale_age,
                "age exactly at threshold should remain valid for estimator")) {
        return false;
    }
    if (!expect(just_valid.intent.valid && !just_valid.intent.stale_age,
                "age exactly at threshold should remain valid for intent")) {
        return false;
    }

    RobotState est_just_stale = est_just_valid;
    est_just_stale.sample_id = 2;
    est_just_stale.timestamp_us = TimePointUs{198'999}; // age == 1001us
    MotionIntent intent_just_stale = intent_just_valid;
    intent_just_stale.sample_id = 2;
    intent_just_stale.timestamp_us = TimePointUs{198'999}; // age == 1001us

    const auto just_stale = policy.evaluate(now, est_just_stale, intent_just_stale);
    return expect(!just_stale.estimator.valid && just_stale.estimator.stale_age,
                  "age over threshold should be stale for estimator") &&
           expect(!just_stale.intent.valid && just_stale.intent.stale_age,
                  "age over threshold should be stale for intent");
}

bool testAgeChecksCanBeDisabledForLenientEvaluation() {
    FreshnessPolicy policy(strictFreshnessConfig());
    policy.reset();

    const TimePointUs now{500'000};
    RobotState est{};
    est.sample_id = 7;
    est.timestamp_us = TimePointUs{1};  // Intentionally stale.

    MotionIntent intent{};
    intent.sample_id = 7;
    intent.timestamp_us = TimePointUs{1};  // Intentionally stale.

    const auto strict = policy.evaluate(now, est, intent, false, true);
    if (!expect(!strict.estimator.valid && strict.estimator.stale_age,
                "strict evaluation should reject stale estimator by age")) {
        return false;
    }
    if (!expect(!strict.intent.valid && strict.intent.stale_age,
                "strict evaluation should reject stale intent by age")) {
        return false;
    }

    const auto lenient = policy.evaluate(now, est, intent, false, false);
    return expect(lenient.estimator.valid && !lenient.estimator.stale_age,
                  "lenient evaluation should ignore stale estimator age") &&
           expect(lenient.intent.valid && !lenient.intent.stale_age,
                  "lenient evaluation should ignore stale intent age");
}

} // namespace

int main() {
    if (!testNonMonotonicSampleIds() ||
        !testMissingTimestamps() ||
        !testBoundaryAgeThresholds() ||
        !testAgeChecksCanBeDisabledForLenientEvaluation()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
