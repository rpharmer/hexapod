#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "local_map.hpp"
#include "local_planner.hpp"
#include "types.hpp"

struct ParsedToml;

namespace control_config {

inline constexpr int kDefaultBusLoopPeriodUs = 2000;
inline constexpr int kDefaultEstimatorLoopPeriodUs = 2000;
inline constexpr int kDefaultControlLoopPeriodUs = 4000;
inline constexpr int kDefaultSafetyLoopPeriodUs = 2000;
inline constexpr int kDefaultDiagnosticsPeriodMs = 500;
inline constexpr int kDefaultCommandRefreshPeriodMs = 100;
inline constexpr int kDefaultStandSettlingDelayMs = 2000;
inline constexpr int kDefaultTelemetryPublishPeriodMs = 50;
inline constexpr int kDefaultTelemetryGeometryRefreshPeriodMs = 2000;
inline constexpr int kDefaultTelemetryUdpPort = 9870;
inline constexpr AngleRad kDefaultMaxTiltRad{0.70};
inline constexpr DurationUs kDefaultCommandTimeoutUs{300000};
inline constexpr DurationUs kDefaultEstimatorMaxAgeUs{300000};
inline constexpr DurationUs kDefaultIntentMaxAgeUs{300000};
inline constexpr LinearRateMps kDefaultFallbackSpeedMag{0.01};
inline constexpr double kDefaultGaitTransitionBlendS{0.35};
inline constexpr double kDefaultGaitNominalPlanarSpeedMps{0.25};
inline constexpr double kDefaultGaitNominalYawRateRadps{0.5};
inline constexpr double kDefaultGaitTurnNominalRadiusM{0.11};
/** Default fraction of estimator body twist blended into foot placement / support velocity (0 = intent only). */
inline constexpr double kDefaultFootEstimatorBlend{0.35};
inline constexpr double kDefaultSwingHeightScale{1.0};
inline constexpr double kDefaultSwingEaseMin{0.40};
inline constexpr double kDefaultSwingEaseMax{1.0};
inline constexpr float kDefaultMinBusVoltageV{10.5f};
inline constexpr float kDefaultMaxBusCurrentA{25.0f};
inline constexpr int kDefaultMinFootContacts{0};
inline constexpr int kDefaultMaxFootContacts{kNumLegs};
inline constexpr double kDefaultNavBodyFrameIntegralKiFwdPerS{0.0};
inline constexpr double kDefaultNavBodyFrameIntegralKiLatPerS{0.0};
inline constexpr double kDefaultNavBodyFrameIntegralAbsCapMetersSeconds{0.0};

inline constexpr double kDefaultFootTerrainSwingMarginM{0.018};
inline constexpr double kDefaultFootTerrainSwingMaxLiftM{0.040};
inline constexpr double kDefaultFootTerrainSwingBlend{0.55};
inline constexpr double kDefaultFootTerrainStancePlaneBlend{0.35};
inline constexpr double kDefaultFootTerrainStancePlaneDzMaxM{0.012};
inline constexpr int kDefaultFootTerrainStanceGroundMinSamples{2};
inline constexpr double kDefaultFootTerrainSwingXYNudgeMaxM{0.018};
inline constexpr int kDefaultFootTerrainSwingXYNudgeWindowCells{2};
inline constexpr double kDefaultFootTerrainSwingXYNudgeTauMin{0.55};
inline constexpr double kDefaultFootTerrainSwingXYNudgeBlend{0.60};

struct LoopTimingConfig {
    std::chrono::microseconds bus_loop_period{std::chrono::microseconds{kDefaultBusLoopPeriodUs}};
    std::chrono::microseconds estimator_loop_period{std::chrono::microseconds{kDefaultEstimatorLoopPeriodUs}};
    std::chrono::microseconds control_loop_period{std::chrono::microseconds{kDefaultControlLoopPeriodUs}};
    std::chrono::microseconds safety_loop_period{std::chrono::microseconds{kDefaultSafetyLoopPeriodUs}};
    std::chrono::milliseconds diagnostics_period{std::chrono::milliseconds{kDefaultDiagnosticsPeriodMs}};
    std::chrono::milliseconds command_refresh_period{std::chrono::milliseconds{kDefaultCommandRefreshPeriodMs}};
    std::chrono::milliseconds stand_settling_delay{std::chrono::milliseconds{kDefaultStandSettlingDelayMs}};
};

struct SafetyConfig {
    AngleRad max_tilt_rad{kDefaultMaxTiltRad};
    DurationUs command_timeout_us{kDefaultCommandTimeoutUs};
    float min_bus_voltage_v{kDefaultMinBusVoltageV};
    float max_bus_current_a{kDefaultMaxBusCurrentA};
    int min_foot_contacts{kDefaultMinFootContacts};
    int max_foot_contacts{kDefaultMaxFootContacts};
};

struct GaitConfig {
    LinearRateMps fallback_speed_mag{kDefaultFallbackSpeedMag};
    /** Seconds to blend gait timing, duty, offsets, and stride shape when `GaitType` changes. */
    double transition_blend_s{kDefaultGaitTransitionBlendS};
    /** Planar speed used to normalize `vx` / `vy` for automatic stride and cadence scaling. */
    double nominal_planar_speed_mps{kDefaultGaitNominalPlanarSpeedMps};
    /** Yaw rate used to normalize commanded turn rate for scaling. */
    double nominal_yaw_rate_radps{kDefaultGaitNominalYawRateRadps};
    /** Effective radius (m) for converting yaw rate into a tangential speed scale. */
    double turn_nominal_radius_m{kDefaultGaitTurnNominalRadiusM};
    /** Blend factor [0,1] for measured body twist vs motion intent in `bodyVelocityForFootPlanning`. */
    double foot_estimator_blend{kDefaultFootEstimatorBlend};
    /** Multiplier on preset swing height after velocity scaling (clamped to a safe range). */
    double swing_height_scale{kDefaultSwingHeightScale};
    /** Clamp lower bound for `swing_time_ease` from `buildTargetUnifiedGait` (S-curve on swing phase). */
    double swing_ease_min{kDefaultSwingEaseMin};
    /** Clamp upper bound for `swing_time_ease`. */
    double swing_ease_max{kDefaultSwingEaseMax};
};

/** Limits and optional low-pass on the unified body-frame locomotion twist (Stage 1 command layer). */
struct LocomotionCommandConfig {
    bool enable_first_order_filter{true};
    /** Time constant (s) for exponential smoothing while walking; larger = slower/smoother. */
    double filter_time_constant_s{0.10};
    /** Fallback integration step (s) when intent timestamps do not advance. */
    double nominal_dt_s{0.004};
    double max_abs_linear_x_mps{0.50};
    double max_abs_linear_y_mps{0.50};
    double max_abs_linear_z_mps{0.35};
    double max_abs_angular_x_radps{0.85};
    double max_abs_angular_y_radps{0.85};
    double max_abs_angular_z_radps{1.10};
};

struct StreamFreshnessConfig {
    DurationUs max_allowed_age_us{kDefaultCommandTimeoutUs};
    bool require_timestamp{true};
    bool require_nonzero_sample_id{true};
    bool require_monotonic_sample_id{true};
};

struct FreshnessConfig {
    StreamFreshnessConfig estimator{
        kDefaultEstimatorMaxAgeUs,
        true,
        true,
        true};
    StreamFreshnessConfig intent{
        kDefaultIntentMaxAgeUs,
        true,
        true,
        true};
};

struct TelemetryConfig {
    bool enabled{false};
    std::string host{"127.0.0.1"};
    int port{9870};
    double publish_rate_hz{30.0};
    double geometry_resend_interval_sec{1.0};
    std::string udp_host{"127.0.0.1"};
    int udp_port{kDefaultTelemetryUdpPort};
    std::chrono::milliseconds publish_period{std::chrono::milliseconds{kDefaultTelemetryPublishPeriodMs}};
    std::chrono::milliseconds geometry_refresh_period{std::chrono::milliseconds{kDefaultTelemetryGeometryRefreshPeriodMs}};
};

/** Tuning for `NavLocomotionBridge` when used by scenarios / callers (not applied automatically in core loop yet). */
struct NavBridgeConfig {
    double body_frame_integral_ki_fwd_per_s{kDefaultNavBodyFrameIntegralKiFwdPerS};
    double body_frame_integral_ki_lat_per_s{kDefaultNavBodyFrameIntegralKiLatPerS};
    double body_frame_integral_abs_cap_m_s{kDefaultNavBodyFrameIntegralAbsCapMetersSeconds};
};

/** Tuning for LiDAR map–driven swing clearance, stance height bias, and late-swing XY nudges. */
struct FootTerrainConfig {
    double swing_margin_m{kDefaultFootTerrainSwingMarginM};
    double swing_max_lift_m{kDefaultFootTerrainSwingMaxLiftM};
    double swing_blend{kDefaultFootTerrainSwingBlend};
    double stance_plane_blend{kDefaultFootTerrainStancePlaneBlend};
    double stance_plane_dz_max_m{kDefaultFootTerrainStancePlaneDzMaxM};
    int stance_ground_min_samples{kDefaultFootTerrainStanceGroundMinSamples};
    double swing_xy_nudge_max_m{kDefaultFootTerrainSwingXYNudgeMaxM};
    int swing_xy_nudge_window_cells{kDefaultFootTerrainSwingXYNudgeWindowCells};
    double swing_xy_nudge_tau_min{kDefaultFootTerrainSwingXYNudgeTauMin};
    double swing_xy_nudge_blend{kDefaultFootTerrainSwingXYNudgeBlend};
};

struct ControlConfig {
    LoopTimingConfig loop_timing{};
    SafetyConfig safety{};
    GaitConfig gait{};
    LocomotionCommandConfig locomotion_cmd{};
    FreshnessConfig freshness{};
    TelemetryConfig telemetry{};
    NavBridgeConfig nav_bridge{};
    FootTerrainConfig foot_terrain{};
    LocalMapConfig local_map{};
    LocalPlannerConfig local_planner{};
};

ControlConfig fromParsedToml(const ParsedToml& config);

} // namespace control_config
