#include "control_config.hpp"

#include <algorithm>

#include "hexapod-server.hpp"

namespace control_config {

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
    parsed.gait.transition_blend_s = config.gaitTransitionBlendS;
    parsed.gait.nominal_planar_speed_mps = config.gaitNominalPlanarSpeedMps;
    parsed.gait.nominal_yaw_rate_radps = config.gaitNominalYawRateRadps;
    parsed.gait.turn_nominal_radius_m = config.gaitTurnNominalRadiusM;
    parsed.gait.foot_estimator_blend = config.footEstimatorBlend;

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

    return parsed;
}

} // namespace control_config
