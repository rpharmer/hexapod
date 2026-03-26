#include "control_config.hpp"

#include <algorithm>
#include <array>

#include "hexapod-server.hpp"

namespace control_config {

namespace {

std::array<double, kNumLegs> toLegArray(const std::vector<double>& values,
                                        const std::array<double, kNumLegs>& defaults) {
    std::array<double, kNumLegs> out = defaults;
    if (values.size() != kNumLegs) {
        return out;
    }
    for (int i = 0; i < kNumLegs; ++i) {
        out[i] = std::clamp(values[static_cast<std::size_t>(i)], 0.0, 1.0);
    }
    return out;
}

} // namespace

ControlConfig fromParsedToml(const ParsedToml& config) {
    ControlConfig parsed{};
    parsed.loop_timing.bus_loop_period = std::chrono::microseconds{config.busLoopPeriodUs};
    parsed.loop_timing.estimator_loop_period = std::chrono::microseconds{config.estimatorLoopPeriodUs};
    parsed.loop_timing.control_loop_period = std::chrono::microseconds{config.controlLoopPeriodUs};
    parsed.loop_timing.safety_loop_period = std::chrono::microseconds{config.safetyLoopPeriodUs};
    parsed.loop_timing.diagnostics_period = std::chrono::milliseconds{config.diagnosticsPeriodMs};
    parsed.loop_timing.command_refresh_period = std::chrono::milliseconds{config.commandRefreshPeriodMs};
    parsed.loop_timing.stand_settling_delay = std::chrono::milliseconds{config.standSettlingDelayMs};

    parsed.safety.max_tilt_rad = AngleRad{config.maxTiltRad};
    parsed.safety.command_timeout_us = DurationUs{config.commandTimeoutUs};
    parsed.safety.min_bus_voltage_v = static_cast<float>(config.minBusVoltageV);
    parsed.safety.max_bus_current_a = static_cast<float>(config.maxBusCurrentA);
    parsed.safety.min_foot_contacts = config.minFootContacts;
    parsed.safety.max_foot_contacts = config.maxFootContacts;

    parsed.gait.fallback_speed_mag = LinearRateMps{config.fallbackSpeedMag};
    parsed.gait.frequency.min_hz = FrequencyHz{config.gaitFrequencyMinHz};
    parsed.gait.frequency.max_hz = FrequencyHz{config.gaitFrequencyMaxHz};
    parsed.gait.frequency.nominal_max_speed_mps = LinearRateMps{config.gaitNominalMaxSpeedMps};
    parsed.gait.frequency.reach_envelope_soft_limit = config.gaitReachEnvelopeSoftLimit;
    parsed.gait.frequency.reach_envelope_min_scale = config.gaitReachEnvelopeMinScale;
    parsed.gait.duty.tripod = config.gaitTripodDutyCycle;
    parsed.gait.duty.ripple = config.gaitRippleDutyCycle;
    parsed.gait.duty.wave = config.gaitWaveDutyCycle;
    parsed.gait.phase_offsets.tripod = toLegArray(config.gaitTripodPhaseOffsets, kDefaultTripodPhaseOffsets);
    parsed.gait.phase_offsets.ripple = toLegArray(config.gaitRipplePhaseOffsets, kDefaultRipplePhaseOffsets);
    parsed.gait.phase_offsets.wave = toLegArray(config.gaitWavePhaseOffsets, kDefaultWavePhaseOffsets);
    parsed.gait.swing.height_m = LengthM{config.gaitSwingHeightM};
    parsed.gait.foothold.step_length_m = LengthM{config.gaitFootholdStepLengthM};
    parsed.gait.stance_field.center_x_m = LengthM{config.gaitStanceFieldCenterXM};
    parsed.gait.stance_field.center_y_m = LengthM{config.gaitStanceFieldCenterYM};
    parsed.gait.stance_field.radius_x_m = LengthM{config.gaitStanceFieldRadiusXM};
    parsed.gait.stance_field.radius_y_m = LengthM{config.gaitStanceFieldRadiusYM};
    parsed.gait.priority_suppression.stability_priority = config.gaitStabilityPriority;
    parsed.gait.priority_suppression.reach_suppression_gain = config.gaitReachSuppressionGain;
    parsed.gait.priority_suppression.turn_suppression_gain = config.gaitTurnSuppressionGain;
    parsed.gait.turn_mode_thresholds.yaw_rate_enter_radps = AngularRateRadPerSec{config.gaitTurnYawRateEnterRadps};
    parsed.gait.turn_mode_thresholds.yaw_rate_exit_radps = AngularRateRadPerSec{config.gaitTurnYawRateExitRadps};
    parsed.gait.turn_mode_thresholds.speed_enter_mps = LinearRateMps{config.gaitTurnSpeedEnterMps};
    parsed.gait.turn_mode_thresholds.speed_exit_mps = LinearRateMps{config.gaitTurnSpeedExitMps};
    parsed.gait.acceptance_gate.feature_flag_enabled = config.gaitDynamicFeatureFlagEnabled;
    parsed.gait.acceptance_gate.simulator_first_required = config.gaitDynamicSimulatorFirstRequired;
    parsed.gait.acceptance_gate.simulator_validation_runs_required =
        config.gaitDynamicSimulatorValidationRunsRequired;
    parsed.gait.acceptance_gate.simulator_validation_runs_passed =
        config.gaitDynamicSimulatorValidationRunsPassed;
    parsed.gait.acceptance_gate.max_control_latency_p95_ms = config.gaitDynamicMaxControlLatencyP95Ms;
    parsed.gait.acceptance_gate.observed_control_latency_p95_ms = config.gaitDynamicObservedControlLatencyP95Ms;
    parsed.gait.acceptance_gate.max_safety_faults_per_hour = config.gaitDynamicMaxSafetyFaultsPerHour;
    parsed.gait.acceptance_gate.observed_safety_faults_per_hour = config.gaitDynamicObservedSafetyFaultsPerHour;
    parsed.gait.acceptance_gate.min_stability_margin_m = config.gaitDynamicMinStabilityMarginM;
    parsed.gait.acceptance_gate.observed_min_stability_margin_m = config.gaitDynamicObservedMinStabilityMarginM;

    parsed.freshness.estimator.max_allowed_age_us = DurationUs{config.estimatorMaxAgeUs};
    parsed.freshness.estimator.require_timestamp = config.estimatorRequireTimestamp;
    parsed.freshness.estimator.require_nonzero_sample_id = config.estimatorRequireSampleId;
    parsed.freshness.estimator.require_monotonic_sample_id = config.estimatorRequireMonotonicSampleId;

    parsed.freshness.intent.max_allowed_age_us = DurationUs{config.intentMaxAgeUs};
    parsed.freshness.intent.require_timestamp = config.intentRequireTimestamp;
    parsed.freshness.intent.require_nonzero_sample_id = config.intentRequireSampleId;
    parsed.freshness.intent.require_monotonic_sample_id = config.intentRequireMonotonicSampleId;

    parsed.telemetry.enabled = config.telemetryEnabled;
    parsed.telemetry.host = config.telemetryHost;
    parsed.telemetry.port = config.telemetryPort;
    parsed.telemetry.publish_rate_hz = config.telemetryPublishRateHz;
    parsed.telemetry.geometry_resend_interval_sec = config.telemetryGeometryResendIntervalSec;
    parsed.telemetry.udp_host = config.telemetryUdpHost;
    parsed.telemetry.udp_port = config.telemetryUdpPort;
    parsed.telemetry.publish_period = std::chrono::milliseconds{config.telemetryPublishPeriodMs};
    parsed.telemetry.geometry_refresh_period = std::chrono::milliseconds{config.telemetryGeometryRefreshPeriodMs};
    parsed.runtime_imu.enable_reads = config.imuEnableReads;

    return parsed;
}

} // namespace control_config
