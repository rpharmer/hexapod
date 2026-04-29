#include "control_config.hpp"

#include <algorithm>
#include <cmath>

#include "hexapod-server.hpp"
#include "types.hpp"

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
    parsed.safety.rapid_body_rate_radps = config.rapidBodyRateRadps;
    parsed.safety.rapid_body_rate_max_contacts = config.rapidBodyRateMaxContacts;
    parsed.safety.command_timeout_us = DurationUs{config.commandTimeoutUs};
    parsed.safety.min_bus_voltage_v = static_cast<float>(config.minBusVoltageV);
    parsed.safety.max_bus_current_a = static_cast<float>(config.maxBusCurrentA);
    parsed.safety.min_foot_contacts = config.minFootContacts;
    parsed.safety.max_foot_contacts = config.maxFootContacts;
    parsed.safety.body_height_collapse_margin_m = config.bodyHeightCollapseMarginM;
    parsed.safety.body_height_collapse_min_safe_m = config.bodyHeightCollapseMinSafeM;
    parsed.safety.body_height_collapse_max_contacts = config.bodyHeightCollapseMaxContacts;

    parsed.gait.fallback_speed_mag = LinearRateMps{config.fallbackSpeedMag};
    parsed.gait.transition_blend_s = config.gaitTransitionBlendS;
    parsed.gait.nominal_planar_speed_mps = config.gaitNominalPlanarSpeedMps;
    parsed.gait.nominal_yaw_rate_radps = config.gaitNominalYawRateRadps;
    parsed.gait.turn_nominal_radius_m = config.gaitTurnNominalRadiusM;
    parsed.gait.foot_estimator_blend = config.footEstimatorBlend;
    parsed.gait.swing_height_scale = config.swingHeightScale;
    parsed.gait.swing_ease_min = config.swingEaseMin;
    parsed.gait.swing_ease_max = config.swingEaseMax;

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
    parsed.telemetry.udp_host = config.telemetryHost;
    parsed.telemetry.udp_port = config.telemetryPort;
    parsed.telemetry.publish_period = std::chrono::milliseconds{
        std::max(1, static_cast<int>(std::lround(1000.0 / std::max(config.telemetryPublishRateHz, 0.1))))};
    parsed.telemetry.geometry_refresh_period = std::chrono::milliseconds{
        std::max(1, static_cast<int>(std::lround(config.telemetryGeometryResendIntervalSec * 1000.0)))};
    parsed.replay_log.enabled = config.replayLogToFile;
    parsed.replay_log.file_path = config.replayLogFilePath;
    parsed.foot_terrain.enable_stance_plane_bias = !config.investigationDisableTerrainStanceBias;
    parsed.foot_terrain.enable_swing_clearance = !config.investigationDisableTerrainSwingClearance;
    parsed.foot_terrain.enable_swing_xy_nudge = !config.investigationDisableTerrainSwingXYNudge;
    parsed.foot_terrain.enable_stance_tilt_leveling = !config.investigationDisableStanceTiltLeveling;

    parsed.nav_bridge.body_frame_integral_ki_fwd_per_s = config.navBodyFrameIntegralKiFwdPerS;
    parsed.nav_bridge.body_frame_integral_ki_lat_per_s = config.navBodyFrameIntegralKiLatPerS;
    parsed.nav_bridge.body_frame_integral_abs_cap_m_s = config.navBodyFrameIntegralAbsCapMetersSeconds;
    parsed.command_governor.low_speed_planar_cutoff_mps = config.governorLowSpeedPlanarCutoffMps;
    parsed.command_governor.low_speed_yaw_cutoff_radps = config.governorLowSpeedYawCutoffRadps;
    parsed.command_governor.startup_support_margin_m = config.governorStartupSupportMarginM;
    parsed.command_governor.support_margin_soft_m = config.governorSupportMarginSoftM;
    parsed.command_governor.support_margin_hard_m = config.governorSupportMarginHardM;
    parsed.command_governor.tilt_soft_rad = config.governorTiltSoftRad;
    parsed.command_governor.tilt_hard_rad = config.governorTiltHardRad;
    parsed.command_governor.body_rate_soft_radps = config.governorBodyRateSoftRadps;
    parsed.command_governor.body_rate_hard_radps = config.governorBodyRateHardRadps;
    parsed.command_governor.fusion_trust_soft = config.governorFusionTrustSoft;
    parsed.command_governor.fusion_trust_hard = config.governorFusionTrustHard;
    parsed.command_governor.contact_mismatch_soft = config.governorContactMismatchSoft;
    parsed.command_governor.contact_mismatch_hard = config.governorContactMismatchHard;
    parsed.command_governor.command_accel_soft_mps2 = config.governorCommandAccelSoftMps2;
    parsed.command_governor.command_accel_hard_mps2 = config.governorCommandAccelHardMps2;
    parsed.command_governor.low_speed_min_scale = config.governorLowSpeedMinScale;
    parsed.command_governor.active_min_scale = config.governorActiveMinScale;
    parsed.command_governor.low_speed_cadence_min_scale = config.governorLowSpeedCadenceMinScale;
    parsed.command_governor.active_cadence_min_scale = config.governorActiveCadenceMinScale;
    parsed.command_governor.body_height_squat_max_m = config.governorBodyHeightSquatMaxM;
    parsed.command_governor.body_height_squat_severity_threshold =
        config.governorBodyHeightSquatSeverityThreshold;
    parsed.command_governor.swing_floor_boost_m = config.governorSwingFloorBoostM;
    parsed.local_map.width_cells = config.localMapWidthCells;
    parsed.local_map.height_cells = config.localMapHeightCells;
    parsed.local_map.resolution_m = config.localMapResolutionM;
    parsed.local_map.obstacle_inflation_radius_m = config.localMapObstacleInflationRadiusM;
    parsed.local_map.safety_margin_m = config.localMapSafetyMarginM;
    parsed.local_map.observation_timeout_s = config.localMapObservationTimeoutS;
    parsed.local_map.observation_decay_s = config.localMapObservationDecayS;
    parsed.local_planner.replan_period_s = config.localPlannerReplanPeriodS;
    parsed.local_planner.search_horizon_m = config.localPlannerSearchHorizonM;
    parsed.local_planner.search_node_budget = config.localPlannerSearchNodeBudget;
    parsed.local_planner.max_output_waypoints = config.localPlannerMaxOutputWaypoints;
    parsed.local_planner.segment_cell_horizon = config.localPlannerSegmentCellHorizon;
    parsed.local_planner.blocked_timeout_s = config.localPlannerBlockedTimeoutS;

    parsed.foot_terrain.swing_margin_m = config.footTerrainSwingMarginM;
    parsed.foot_terrain.swing_max_lift_m = config.footTerrainSwingMaxLiftM;
    parsed.foot_terrain.swing_blend = config.footTerrainSwingBlend;
    parsed.foot_terrain.stance_plane_blend = config.footTerrainStancePlaneBlend;
    parsed.foot_terrain.stance_plane_dz_max_m = config.footTerrainStancePlaneDzMaxM;
    parsed.foot_terrain.swing_xy_nudge_max_m = config.footTerrainSwingXYNudgeMaxM;
    parsed.foot_terrain.swing_xy_nudge_tau_min = config.footTerrainSwingXYNudgeTauMin;
    parsed.foot_terrain.swing_xy_nudge_blend = config.footTerrainSwingXYNudgeBlend;
    parsed.foot_terrain.stance_ground_min_samples =
        std::clamp(config.footTerrainStanceGroundMinSamples, 1, kNumLegs);
    parsed.foot_terrain.swing_xy_nudge_window_cells =
        std::clamp(config.footTerrainSwingXYNudgeWindowCells, 0, 8);

    parsed.fusion.contact_debounce_samples =
        std::clamp(config.fusionContactDebounceSamples, 1, 16);
    parsed.fusion.touchdown_window = std::chrono::milliseconds{config.fusionTouchdownWindowMs};
    parsed.fusion.contact_hold_window = std::chrono::milliseconds{config.fusionContactHoldWindowMs};
    parsed.fusion.trust_decay_per_mismatch = config.fusionTrustDecayPerMismatch;
    parsed.fusion.predictive_trust_bias = config.fusionPredictiveTrustBias;
    parsed.fusion.soft_pose_resync_m = config.fusionSoftPoseResyncM;
    parsed.fusion.hard_pose_resync_m = config.fusionHardPoseResyncM;
    parsed.fusion.soft_orientation_resync_rad = config.fusionSoftOrientationResyncRad;
    parsed.fusion.hard_orientation_resync_rad = config.fusionHardOrientationResyncRad;
    parsed.fusion.soft_contact_mismatch_ratio = config.fusionSoftContactMismatchRatio;
    parsed.fusion.hard_contact_mismatch_ratio = config.fusionHardContactMismatchRatio;
    parsed.fusion.correction_mode_hold_samples = config.fusionCorrectionHoldSamples;
    parsed.fusion.correction_mode_strong_release_factor = config.fusionCorrectionStrongReleaseFactor;
    parsed.fusion.correction_mode_soft_release_factor = config.fusionCorrectionSoftReleaseFactor;
    parsed.fusion.emit_physics_sim_corrections = !config.investigationSuppressFusionCorrections;
    parsed.fusion.suppress_fusion_resets = config.investigationSuppressFusionResets;

    return parsed;
}

} // namespace control_config
