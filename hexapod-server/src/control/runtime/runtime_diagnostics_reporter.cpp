#include "runtime_diagnostics_reporter.hpp"

#include "status_reporter.hpp"

#include <array>
#include <cmath>

namespace {

constexpr std::array<const char*, kNumLegs> kLegNames{"RF", "RM", "RR", "LF", "LM", "LR"};
constexpr std::array<const char*, kJointsPerLeg> kJointNames{"coxa", "femur", "tibia"};

const char* legName(std::size_t leg_idx) {
    return leg_idx < kLegNames.size() ? kLegNames[leg_idx] : "unknown_leg";
}

const char* jointName(std::size_t joint_idx) {
    return joint_idx < kJointNames.size() ? kJointNames[joint_idx] : "unknown_joint";
}

} // namespace

RuntimeDiagnosticsReporter::RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                                                       const FreshnessPolicy& freshness_policy)
    : logger_(std::move(logger)),
      freshness_policy_(freshness_policy) {}

void RuntimeDiagnosticsReporter::recordVisualizerTelemetry(
    const telemetry::TelemetryPublishCounters& telemetry_counters,
    TimePointUs now) {
    const uint64_t sent_delta = telemetry_counters.packets_sent >= last_telemetry_counters_.packets_sent
                                    ? telemetry_counters.packets_sent - last_telemetry_counters_.packets_sent
                                    : telemetry_counters.packets_sent;
    const uint64_t failures_delta =
        telemetry_counters.socket_send_failures >= last_telemetry_counters_.socket_send_failures
            ? telemetry_counters.socket_send_failures - last_telemetry_counters_.socket_send_failures
            : telemetry_counters.socket_send_failures;
    last_telemetry_counters_ = telemetry_counters;

    telemetry_diag_.packets_sent = telemetry_counters.packets_sent;
    telemetry_diag_.socket_send_failures = telemetry_counters.socket_send_failures;
    telemetry_diag_.last_successful_send_timestamp = telemetry_counters.last_successful_send_timestamp;

    if (sent_delta > 0) {
        maybeLogVisualizerRecovery();
        consecutive_failures_ = 0;
    }

    if (failures_delta > 0) {
        consecutive_failures_ += failures_delta;
        maybeLogVisualizerFailureWarning(now);
    }
}

void RuntimeDiagnosticsReporter::setRuntimeImuReadsEnabled(bool enabled) {
    imu_reads_enabled_ = enabled;
}

void RuntimeDiagnosticsReporter::recordControlOutputs(const JointTargets& targets,
                                                      const ControlStatus& status,
                                                      TimePointUs now,
                                                      const LegTargets* leg_targets) {
    const bool diagnostics_trackable =
        status.estimator_valid && status.active_fault == FaultCode::NONE &&
        (status.active_mode == RobotMode::STAND || status.active_mode == RobotMode::WALK);
    if (!diagnostics_trackable) {
        if (diagnostics_tracking_active_) {
            joint_oscillation_tracker_.reset();
        }
        diagnostics_tracking_active_ = false;
        has_previous_leg_targets_ = false;
        has_previous_status_ = false;
        last_control_output_timestamp_ = now;
        return;
    }

    if (!diagnostics_tracking_active_) {
        joint_oscillation_tracker_.reset();
        diagnostics_tracking_active_ = true;
        if (leg_targets != nullptr) {
            previous_leg_targets_ = *leg_targets;
            has_previous_leg_targets_ = true;
        }
        previous_status_ = status;
        has_previous_status_ = true;
        last_control_output_timestamp_ = now;
        return;
    }

    joint_oscillation_tracker_.observe(targets, now);

    if (!last_control_output_timestamp_.isZero() && now.value > last_control_output_timestamp_.value) {
        const double dt_s = static_cast<double>(now.value - last_control_output_timestamp_.value) / 1'000'000.0;

        if (has_previous_status_) {
            const double cadence_rate =
                std::abs(status.dynamic_gait.cadence_hz - previous_status_.dynamic_gait.cadence_hz) / dt_s;
            if (cadence_rate > gait_variability_diag_.peak_cadence_delta_hz_per_s) {
                gait_variability_diag_.peak_cadence_delta_hz_per_s = cadence_rate;
            }

            const double reach_rate = std::abs(status.dynamic_gait.reach_utilization -
                                               previous_status_.dynamic_gait.reach_utilization) /
                                      dt_s;
            if (reach_rate > gait_variability_diag_.peak_reach_utilization_delta_per_s) {
                gait_variability_diag_.peak_reach_utilization_delta_per_s = reach_rate;
            }

            for (std::size_t leg_idx = 0; leg_idx < static_cast<std::size_t>(kNumLegs); ++leg_idx) {
                const double phase_rate = std::abs(status.dynamic_gait.leg_phase[leg_idx] -
                                                   previous_status_.dynamic_gait.leg_phase[leg_idx]) /
                                          dt_s;
                if (phase_rate > gait_variability_diag_.peak_phase_delta_per_s_by_leg[leg_idx]) {
                    gait_variability_diag_.peak_phase_delta_per_s_by_leg[leg_idx] = phase_rate;
                }
                if (phase_rate > gait_variability_diag_.peak_phase_delta_per_s) {
                    gait_variability_diag_.peak_phase_delta_per_s = phase_rate;
                }
            }

            if (cadence_rate > 20.0 || reach_rate > 6.0) {
                ++gait_variability_diag_.rapid_change_events;
            }
        }
    }

    if (leg_targets != nullptr && has_previous_leg_targets_ && !last_control_output_timestamp_.isZero() &&
        now.value > last_control_output_timestamp_.value) {
        const double dt_s = static_cast<double>(now.value - last_control_output_timestamp_.value) / 1'000'000.0;
        for (std::size_t leg_idx = 0; leg_idx < static_cast<std::size_t>(kNumLegs); ++leg_idx) {
            const Vec3& current = leg_targets->feet[leg_idx].pos_body_m;
            const Vec3& previous = previous_leg_targets_.feet[leg_idx].pos_body_m;
            const double dx = current.x - previous.x;
            const double dy = current.y - previous.y;
            const double dz = current.z - previous.z;
            const double velocity_mps = std::sqrt((dx * dx + dy * dy + dz * dz)) / dt_s;
            if (velocity_mps > leg_target_variability_diag_.peak_foot_target_velocity_mps_by_leg[leg_idx]) {
                leg_target_variability_diag_.peak_foot_target_velocity_mps_by_leg[leg_idx] = velocity_mps;
            }
            if (velocity_mps > leg_target_variability_diag_.peak_foot_target_velocity_mps) {
                leg_target_variability_diag_.peak_foot_target_velocity_mps = velocity_mps;
            }
            if (velocity_mps > 0.75) {
                ++leg_target_variability_diag_.rapid_change_events;
            }
        }
    }

    if (leg_targets != nullptr) {
        previous_leg_targets_ = *leg_targets;
        has_previous_leg_targets_ = true;
    }
    previous_status_ = status;
    has_previous_status_ = true;
    last_control_output_timestamp_ = now;
}

void RuntimeDiagnosticsReporter::report(const ControlStatus& status,
                                        const std::optional<BridgeCommandResultMetadata>& bridge_result,
                                        uint64_t loops,
                                        uint64_t avg_control_dt_us,
                                        uint64_t max_control_jitter_us,
                                        const LoopTimingRollingMetrics& loop_timing_metrics,
                                        uint64_t stale_intent_events,
                                        uint64_t stale_estimator_events,
                                        uint64_t freshness_reject_consecutive,
                                        uint64_t freshness_reject_total,
                                        uint64_t freshness_reject_stale_age_total,
                                        uint64_t freshness_reject_invalid_sample_id_total,
                                        uint64_t freshness_reject_non_monotonic_id_total) {
    status_reporter::logStatus(logger_, status, bridge_result);
    if (!logger_) {
        return;
    }

    const auto& estimator_diag = freshness_policy_.estimatorDiagnostics();
    const auto& intent_diag = freshness_policy_.intentDiagnostics();
    const double freshness_stale_age_reject_rate = freshness_reject_total > 0
                                                       ? static_cast<double>(freshness_reject_stale_age_total) /
                                                             static_cast<double>(freshness_reject_total)
                                                       : 0.0;
    const double freshness_invalid_sample_reject_rate = freshness_reject_total > 0
                                                            ? static_cast<double>(freshness_reject_invalid_sample_id_total) /
                                                                  static_cast<double>(freshness_reject_total)
                                                            : 0.0;
    const double freshness_non_monotonic_reject_rate = freshness_reject_total > 0
                                                           ? static_cast<double>(freshness_reject_non_monotonic_id_total) /
                                                                 static_cast<double>(freshness_reject_total)
                                                           : 0.0;
    const JointOscillationMetrics joint_diag = joint_oscillation_tracker_.metrics();
    const std::size_t dropped_messages = logger_->DroppedMessageCount();
    const logging::AsyncLogger::QueueState log_queue_state = logger_->CurrentQueueState();
    std::size_t worst_reversal_joint = 0;
    std::size_t worst_velocity_joint = 0;
    std::size_t worst_leg_target_velocity_leg = 0;
    std::size_t worst_phase_leg = 0;
    for (std::size_t idx = 1; idx < joint_diag.kJointCount; ++idx) {
        if (joint_diag.direction_reversal_events_by_joint[idx] >
            joint_diag.direction_reversal_events_by_joint[worst_reversal_joint]) {
            worst_reversal_joint = idx;
        }
        if (joint_diag.peak_joint_velocity_radps_by_joint[idx] >
            joint_diag.peak_joint_velocity_radps_by_joint[worst_velocity_joint]) {
            worst_velocity_joint = idx;
        }
    }
    for (std::size_t leg_idx = 1; leg_idx < static_cast<std::size_t>(kNumLegs); ++leg_idx) {
        if (leg_target_variability_diag_.peak_foot_target_velocity_mps_by_leg[leg_idx] >
            leg_target_variability_diag_.peak_foot_target_velocity_mps_by_leg[worst_leg_target_velocity_leg]) {
            worst_leg_target_velocity_leg = leg_idx;
        }
        if (gait_variability_diag_.peak_phase_delta_per_s_by_leg[leg_idx] >
            gait_variability_diag_.peak_phase_delta_per_s_by_leg[worst_phase_leg]) {
            worst_phase_leg = leg_idx;
        }
    }
    const std::size_t reversal_leg = worst_reversal_joint / static_cast<std::size_t>(kJointsPerLeg);
    const std::size_t reversal_joint = worst_reversal_joint % static_cast<std::size_t>(kJointsPerLeg);
    const std::size_t velocity_leg = worst_velocity_joint / static_cast<std::size_t>(kJointsPerLeg);
    const std::size_t velocity_joint = worst_velocity_joint % static_cast<std::size_t>(kJointsPerLeg);
    LOG_INFO(logger_,
             "runtime.metrics loops=",
             loops,
             " imu_reads_enabled=",
             imu_reads_enabled_ ? 1 : 0,
             " avg_control_dt_us=",
             avg_control_dt_us,
             " max_control_jitter_us=",
             max_control_jitter_us,
             " loop_dt_window_samples=",
             loop_timing_metrics.sample_count,
             " loop_dt_p50_us=",
             loop_timing_metrics.p50_control_dt_us,
             " loop_dt_p95_us=",
             loop_timing_metrics.p95_control_dt_us,
             " loop_dt_p99_us=",
             loop_timing_metrics.p99_control_dt_us,
             " loop_overrun_events_total=",
             loop_timing_metrics.overrun_events_total,
             " loop_overrun_periods_total=",
             loop_timing_metrics.overrun_periods_total,
             " loop_consecutive_overruns=",
             loop_timing_metrics.consecutive_overruns,
             " loop_max_consecutive_overruns=",
             loop_timing_metrics.max_consecutive_overruns,
             " loop_hard_overrun_escalation_crossings=",
             loop_timing_metrics.hard_overrun_escalation_crossings,
             " stale_intent_events=",
             stale_intent_events,
             " stale_estimator_events=",
             stale_estimator_events,
             " freshness_rejects_consecutive=",
             freshness_reject_consecutive,
             " freshness_rejects_total=",
             freshness_reject_total,
             " freshness_rejects_stale_age_total=",
             freshness_reject_stale_age_total,
             " freshness_rejects_invalid_sample_id_total=",
             freshness_reject_invalid_sample_id_total,
             " freshness_rejects_non_monotonic_id_total=",
             freshness_reject_non_monotonic_id_total,
             " freshness_reject_rate_stale_age=",
             freshness_stale_age_reject_rate,
             " freshness_reject_rate_invalid_sample_id=",
             freshness_invalid_sample_reject_rate,
             " freshness_reject_rate_non_monotonic_id=",
             freshness_non_monotonic_reject_rate,
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
             "} logger_diag={dropped_messages:",
             dropped_messages,
             ",queue_depth:",
             log_queue_state.depth,
             ",queue_capacity:",
             log_queue_state.capacity,
             ",worker_busy:",
             log_queue_state.worker_busy ? 1 : 0,
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
             "} joint_cmd_diag={direction_reversals:",
             joint_diag.direction_reversal_events,
             ",worst_reversal_joint:",
             legName(reversal_leg),
             ".",
             jointName(reversal_joint),
             ",worst_reversal_count:",
             joint_diag.direction_reversal_events_by_joint[worst_reversal_joint],
             ",peak_velocity_radps:",
             joint_diag.peak_joint_velocity_radps,
             ",worst_velocity_joint:",
             legName(velocity_leg),
             ".",
             jointName(velocity_joint),
             ",worst_velocity_joint_peak_radps:",
             joint_diag.peak_joint_velocity_radps_by_joint[worst_velocity_joint],
             "} leg_target_diag={peak_foot_vel_mps:",
             leg_target_variability_diag_.peak_foot_target_velocity_mps,
             ",worst_leg:",
             legName(worst_leg_target_velocity_leg),
             ",worst_leg_peak_foot_vel_mps:",
             leg_target_variability_diag_.peak_foot_target_velocity_mps_by_leg[worst_leg_target_velocity_leg],
             ",rapid_change_events:",
             leg_target_variability_diag_.rapid_change_events,
             "} gait_variability_diag={peak_cadence_delta_hz_per_s:",
             gait_variability_diag_.peak_cadence_delta_hz_per_s,
             ",peak_reach_delta_per_s:",
             gait_variability_diag_.peak_reach_utilization_delta_per_s,
             ",peak_phase_delta_per_s:",
             gait_variability_diag_.peak_phase_delta_per_s,
             ",worst_phase_leg:",
             legName(worst_phase_leg),
             ",worst_phase_leg_delta_per_s:",
             gait_variability_diag_.peak_phase_delta_per_s_by_leg[worst_phase_leg],
             ",rapid_change_events:",
             gait_variability_diag_.rapid_change_events,
             "}");

    maybeLogLoopTimingWarning(loop_timing_metrics);
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

void RuntimeDiagnosticsReporter::maybeLogLoopTimingWarning(
    const LoopTimingRollingMetrics& loop_timing_metrics)
{
    if (!logger_) {
        return;
    }

    const uint64_t warn_threshold = loop_timing_metrics.warning_consecutive_overrun_threshold;
    const uint64_t hard_threshold = loop_timing_metrics.hard_consecutive_overrun_threshold;
    if (warn_threshold == 0 || hard_threshold == 0 || warn_threshold >= hard_threshold) {
        return;
    }

    const uint64_t consecutive = loop_timing_metrics.consecutive_overruns;
    if (consecutive < warn_threshold || consecutive >= hard_threshold) {
        if (consecutive < warn_threshold) {
            last_overrun_warning_consecutive_ = 0;
        }
        return;
    }

    if (last_overrun_warning_consecutive_ == consecutive) {
        return;
    }
    last_overrun_warning_consecutive_ = consecutive;

    LOG_WARN(logger_,
             "runtime.loop_timing pre_safety_escalation_warning consecutive_overruns=",
             consecutive,
             " warning_threshold=",
             warn_threshold,
             " hard_escalation_threshold=",
             hard_threshold,
             " p95_us=",
             loop_timing_metrics.p95_control_dt_us,
             " p99_us=",
             loop_timing_metrics.p99_control_dt_us,
             " overrun_events_total=",
             loop_timing_metrics.overrun_events_total,
             " policy=reduce_load_before_safety_fault");
}
