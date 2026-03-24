#include "runtime_diagnostics_reporter.hpp"

#include "status_reporter.hpp"

RuntimeDiagnosticsReporter::RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                                                       const FreshnessPolicy& freshness_policy)
    : logger_(std::move(logger)),
      freshness_policy_(freshness_policy) {}

void RuntimeDiagnosticsReporter::report(const ControlStatus& status,
                                        const std::optional<BridgeCommandResultMetadata>& bridge_result,
                                        uint64_t loops,
                                        uint64_t avg_control_dt_us,
                                        uint64_t max_control_jitter_us,
                                        uint64_t stale_intent_events,
                                        uint64_t stale_estimator_events) const {
    status_reporter::logStatus(logger_, status, bridge_result);
    if (!logger_) {
        return;
    }

    const auto& estimator_diag = freshness_policy_.estimatorDiagnostics();
    const auto& intent_diag = freshness_policy_.intentDiagnostics();
    LOG_INFO(logger_,
             "runtime.metrics loops=",
             loops,
             " avg_control_dt_us=",
             avg_control_dt_us,
             " max_control_jitter_us=",
             max_control_jitter_us,
             " stale_intent_events=",
             stale_intent_events,
             " stale_estimator_events=",
             stale_estimator_events,
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
