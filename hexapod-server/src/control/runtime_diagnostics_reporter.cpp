#include "runtime_diagnostics_reporter.hpp"

#include "status_reporter.hpp"

RuntimeDiagnosticsReporter::RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                                                       const FreshnessPolicy& freshness_policy)
    : logger_(std::move(logger)),
      freshness_policy_(freshness_policy) {}

void RuntimeDiagnosticsReporter::recordVisualizerTelemetry(
    const std::optional<BridgeCommandResultMetadata>& bridge_result,
    uint64_t control_loops,
    TimePointUs now) {
    if (control_loops == last_telemetry_loop_) {
        ++telemetry_diag_.dropped_rate_limited_frames;
        return;
    }
    last_telemetry_loop_ = control_loops;

    if (!bridge_result.has_value()) {
        ++telemetry_diag_.socket_send_failures;
        ++consecutive_failures_;
        maybeLogVisualizerFailureWarning(now);
        return;
    }

    const BridgeCommandResultMetadata& metadata = *bridge_result;
    if (metadata.error == BridgeError::None) {
        ++telemetry_diag_.packets_sent;
        telemetry_diag_.last_successful_send_timestamp = now;
        maybeLogVisualizerRecovery();
        consecutive_failures_ = 0;
        return;
    }

    if (metadata.error == BridgeError::TransportFailure || metadata.error == BridgeError::Timeout) {
        ++telemetry_diag_.socket_send_failures;
    } else if (metadata.phase == BridgeFailurePhase::CommandDecode ||
               metadata.domain == BridgeFailureDomain::CommandProtocol) {
        ++telemetry_diag_.serialization_failures;
    } else {
        ++telemetry_diag_.socket_send_failures;
    }

    ++consecutive_failures_;
    maybeLogVisualizerFailureWarning(now);
}

void RuntimeDiagnosticsReporter::report(const ControlStatus& status,
                                        const std::optional<BridgeCommandResultMetadata>& bridge_result,
                                        uint64_t loops,
                                        uint64_t avg_control_dt_us,
                                        uint64_t max_control_jitter_us,
                                        uint64_t stale_intent_events,
                                        uint64_t stale_estimator_events) {
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
             "} visualizer_diag={packets_sent:",
             telemetry_diag_.packets_sent,
             ",serialization_failures:",
             telemetry_diag_.serialization_failures,
             ",socket_send_failures:",
             telemetry_diag_.socket_send_failures,
             ",dropped_rate_limited_frames:",
             telemetry_diag_.dropped_rate_limited_frames,
             ",last_successful_send_ts_us:",
             telemetry_diag_.last_successful_send_timestamp.value,
             "}");
}

void RuntimeDiagnosticsReporter::maybeLogVisualizerFailureWarning(TimePointUs now) {
    if (!logger_ || consecutive_failures_ < 3) {
        return;
    }
    if (consecutive_failures_ == warning_failures_observed_) {
        return;
    }
    if ((consecutive_failures_ - warning_failures_observed_) < 10) {
        return;
    }
    warning_failures_observed_ = consecutive_failures_;
    LOG_WARN(logger_,
             "visualizer.transport degraded consecutive_failures=",
             consecutive_failures_,
             " socket_send_failures=",
             telemetry_diag_.socket_send_failures,
             " serialization_failures=",
             telemetry_diag_.serialization_failures,
             " last_successful_send_ts_us=",
             telemetry_diag_.last_successful_send_timestamp.value,
             " now_us=",
             now.value,
             " policy=continue_runtime_and_retry");
}

void RuntimeDiagnosticsReporter::maybeLogVisualizerRecovery() {
    if (!logger_ || consecutive_failures_ == 0) {
        return;
    }
    LOG_INFO(logger_,
             "visualizer.transport recovered after_failures=",
             consecutive_failures_,
             " policy=auto_recovery");
}
