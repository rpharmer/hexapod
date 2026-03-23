#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <cstdint>

class FreshnessPolicy {
public:
    struct StreamDiagnostics {
        uint64_t stale_age_count{0};
        uint64_t missing_timestamp_count{0};
        uint64_t invalid_sample_id_count{0};
        uint64_t non_monotonic_sample_id_count{0};
    };

    struct StreamResult {
        bool valid{true};
        bool stale_age{false};
        bool missing_timestamp{false};
        bool invalid_sample_id{false};
        bool non_monotonic_sample_id{false};
        uint64_t age_us{0};
    };

    struct Evaluation {
        StreamResult estimator{};
        StreamResult intent{};
    };

    explicit FreshnessPolicy(control_config::FreshnessConfig config = {});

    void reset();

    Evaluation evaluate(TimePointUs now,
                        const RobotState& est,
                        const MotionIntent& intent,
                        bool update_tracking = true);

    const StreamDiagnostics& estimatorDiagnostics() const;
    const StreamDiagnostics& intentDiagnostics() const;

private:
    StreamResult evaluateStream(TimePointUs now,
                                TimePointUs timestamp_us,
                                uint64_t sample_id,
                                uint64_t& last_sample_id,
                                const control_config::StreamFreshnessConfig& freshness,
                                StreamDiagnostics& diagnostics,
                                bool update_tracking) const;

    control_config::FreshnessConfig config_{};
    uint64_t last_estimator_sample_id_{0};
    uint64_t last_intent_sample_id_{0};
    StreamDiagnostics estimator_diag_{};
    StreamDiagnostics intent_diag_{};
};
