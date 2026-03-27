#include "gait_policy_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "leg_kinematics_utils.hpp"
#include "reach_envelope.hpp"

namespace {
constexpr double kRegionArcEnterSpeed = 0.20;
constexpr double kRegionArcEnterYaw = 0.55;
constexpr double kRegionPivotEnterSpeed = 0.25;
constexpr double kRegionPivotEnterYaw = 0.60;
constexpr double kRegionArcExitSpeed = 0.16;
constexpr double kRegionArcExitYaw = 0.60;
constexpr double kRegionPivotExitSpeed = 0.18;
constexpr double kRegionPivotExitYaw = 0.52;
constexpr double kRegionReorientToArcSpeed = 0.22;
constexpr double kRegionReorientToArcYaw = 0.52;
constexpr double kRegionReorientToPivotSpeed = 0.22;
constexpr double kRegionReorientToPivotYaw = 0.60;
constexpr double kSmoothCommandAlpha = 0.25;
constexpr double kGaitSwitchSpeedHysteresis = 0.03;
constexpr double kGaitSwitchYawHysteresis = 0.04;

double normalizedSpeed(const MotionIntent& intent, const control_config::GaitConfig& config)
{
    const double max_speed = std::max(config.frequency.nominal_max_speed_mps.value, 1e-6);
    return clamp01(std::abs(intent.speed_mps.value) / max_speed);
}

double normalizedYaw(const MotionIntent& intent, const control_config::GaitConfig& config)
{
    const double yaw_enter = std::max(config.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6);
    return clamp01(std::abs(intent.body_pose_setpoint.angular_velocity_radps.z) / yaw_enter);
}
constexpr double kArcMaxRollPitchRad = 8.0 * kPi / 180.0;
constexpr double kPivotMaxRollPitchRad = 10.0 * kPi / 180.0;
constexpr double kReorientationMaxRollPitchRad = 9.0 * kPi / 180.0;
constexpr double kHighRiskRollPitchRad = 11.0 * kPi / 180.0;
constexpr double kFaultHoldReachThreshold = 1.12;
constexpr double kDegradedReachThreshold = 1.06;

const std::array<double, kNumLegs>& gaitPhaseOffsets(const control_config::GaitPhaseOffsetsConfig& config,
                                                      const GaitType gait)
{
    switch (gait) {
        case GaitType::TRIPOD:
            return config.tripod;
        case GaitType::RIPPLE:
            return config.ripple;
        case GaitType::WAVE:
            return config.wave;
    }

    return config.tripod;
}

double gaitDutyCycle(const control_config::GaitDutyConfig& config, const GaitType gait)
{
    switch (gait) {
        case GaitType::TRIPOD:
            return config.tripod;
        case GaitType::RIPPLE:
            return config.ripple;
        case GaitType::WAVE:
            return config.wave;
    }

    return config.tripod;
}

struct RegionClassificationState {
    DynamicGaitRegion region{DynamicGaitRegion::ARC};
    double filtered_speed_norm{0.0};
    double filtered_yaw_norm{0.0};
};

struct RegionClassifier {
    static RegionClassificationState classify(const MotionIntent& intent,
                                              const control_config::GaitConfig& config,
                                              const RegionClassificationState& prior)
    {
        RegionClassificationState next = prior;
        const double speed = normalizedSpeed(intent, config);
        const double yaw = normalizedYaw(intent, config);
        next.filtered_speed_norm = std::lerp(prior.filtered_speed_norm, speed, kSmoothCommandAlpha);
        next.filtered_yaw_norm = std::lerp(prior.filtered_yaw_norm, yaw, kSmoothCommandAlpha);
        next.region = applyHysteresis(prior.region, next.filtered_speed_norm, next.filtered_yaw_norm);
        return next;
    }

private:
    static DynamicGaitRegion applyHysteresis(const DynamicGaitRegion prior, const double speed, const double yaw)
    {
        if (prior == DynamicGaitRegion::ARC) {
            if (yaw > kRegionArcExitYaw || speed < kRegionArcExitSpeed) {
                return DynamicGaitRegion::REORIENTATION;
            }
            return prior;
        }

        if (prior == DynamicGaitRegion::PIVOT) {
            if (yaw < kRegionPivotExitYaw && speed > kRegionPivotExitSpeed) {
                return DynamicGaitRegion::REORIENTATION;
            }
            return prior;
        }

        if (speed >= kRegionReorientToArcSpeed && yaw <= kRegionReorientToArcYaw) {
            return DynamicGaitRegion::ARC;
        }
        if (speed <= kRegionReorientToPivotSpeed && yaw >= kRegionReorientToPivotYaw) {
            return DynamicGaitRegion::PIVOT;
        }
        if (speed >= kRegionArcEnterSpeed && yaw <= kRegionArcEnterYaw) {
            return DynamicGaitRegion::ARC;
        }
        if (speed <= kRegionPivotEnterSpeed && yaw >= kRegionPivotEnterYaw) {
            return DynamicGaitRegion::PIVOT;
        }
        return DynamicGaitRegion::REORIENTATION;
    }
};

struct GaitFamilySelector {
    static GaitType select(const DynamicGaitRegion region,
                           const double speed,
                           const double yaw,
                           const GaitType prior_family)
    {
        const GaitType candidate = candidateFor(region, speed, yaw);
        return applyHysteresis(candidate, prior_family, speed, yaw);
    }

private:
    static GaitType candidateFor(const DynamicGaitRegion region, const double speed, const double yaw)
    {
        if (region == DynamicGaitRegion::PIVOT) {
            return (speed >= 0.15 && yaw <= 0.85) ? GaitType::RIPPLE : GaitType::WAVE;
        }
        if (region == DynamicGaitRegion::REORIENTATION) {
            return (speed < 0.30 || yaw > 0.75) ? GaitType::WAVE : GaitType::RIPPLE;
        }
        if (speed < 0.35) {
            return GaitType::WAVE;
        }
        if (speed < 0.70) {
            return GaitType::RIPPLE;
        }
        if (yaw <= 0.35) {
            return GaitType::TRIPOD;
        }
        return GaitType::RIPPLE;
    }

    static GaitType applyHysteresis(const GaitType candidate,
                                    const GaitType prior,
                                    const double speed,
                                    const double yaw)
    {
        if (candidate == prior) {
            return candidate;
        }

        if (candidate == GaitType::TRIPOD && prior == GaitType::RIPPLE &&
            yaw > (0.35 - kGaitSwitchYawHysteresis)) {
            return prior;
        }
        if (prior == GaitType::WAVE && candidate == GaitType::RIPPLE &&
            speed < (0.35 + kGaitSwitchSpeedHysteresis)) {
            return prior;
        }
        if (prior == GaitType::RIPPLE && candidate == GaitType::TRIPOD &&
            speed < (0.70 + kGaitSwitchSpeedHysteresis)) {
            return prior;
        }
        return candidate;
    }
};

} // namespace

GaitPolicyPlanner::GaitPolicyPlanner(control_config::GaitConfig config)
    : config_(config) {}

RuntimeGaitPolicy GaitPolicyPlanner::plan(const RobotState& est,
                                          const MotionIntent& intent,
                                          const SafetyState& safety)
{
    if (!dynamicPolicyEnabled()) {
        return legacyPolicy(est, intent, safety);
    }

    RuntimeGaitPolicy policy{};
    policy.dynamic_enabled = true;
    policy.region = selectRegion(intent);
    policy.gait_family = selectGaitFamily(intent, policy.region);
    policy.turn_mode = selectTurnMode(policy.region);
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);
    policy.reach_utilization = maxReachUtilization(est);
    const double normalized_command = std::clamp(
        std::abs(intent.speed_mps.value) / config_.frequency.nominal_max_speed_mps.value,
        0.0,
        1.0);

    applyEnvelopeAndFallback(policy, est, intent, safety);

    const double speed_mag = std::max(normalized_command, config_.fallback_speed_mag.value);
    const double envelope_speed_scale = std::clamp(
        (1.0 - policy.reach_utilization) / config_.frequency.reach_envelope_soft_limit,
        config_.frequency.reach_envelope_min_scale,
        1.0);
    policy.cadence_hz = FrequencyHz{std::clamp(
        config_.frequency.min_hz.value +
            (config_.frequency.max_hz.value - config_.frequency.min_hz.value) * speed_mag * envelope_speed_scale,
        config_.frequency.min_hz.value,
        config_.frequency.max_hz.value)};
    applyFallback(policy);
    applyServoVelocityConstraint(policy, est);
    return policy;
}

bool GaitPolicyPlanner::dynamicPolicyEnabled() const
{
    const auto& gate = config_.acceptance_gate;
    if (!gate.feature_flag_enabled) {
        return false;
    }

    if (gate.simulator_first_required &&
        gate.simulator_validation_runs_passed < gate.simulator_validation_runs_required) {
        return false;
    }

    return gate.observed_control_latency_p95_ms <= gate.max_control_latency_p95_ms &&
           gate.observed_safety_faults_per_hour <= gate.max_safety_faults_per_hour &&
           gate.observed_min_stability_margin_m >= gate.min_stability_margin_m;
}

RuntimeGaitPolicy GaitPolicyPlanner::legacyPolicy(const RobotState& est,
                                                  const MotionIntent& intent,
                                                  const SafetyState& safety) const
{
    RuntimeGaitPolicy policy{};
    policy.dynamic_enabled = false;
    policy.region = DynamicGaitRegion::ARC;
    policy.gait_family = intent.gait;
    policy.turn_mode = (std::abs(intent.body_pose_setpoint.angular_velocity_radps.z) >=
                        config_.turn_mode_thresholds.yaw_rate_enter_radps.value)
                           ? TurnMode::IN_PLACE
                           : TurnMode::CRAB;
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);
    policy.reach_utilization = maxReachUtilization(est);

    const double command_norm = std::clamp(
        std::abs(intent.speed_mps.value) / std::max(config_.frequency.nominal_max_speed_mps.value, 1e-6),
        0.0,
        1.0);
    const double speed_mag = std::max(command_norm, config_.fallback_speed_mag.value);
    const double envelope_speed_scale = std::clamp(
        (1.0 - policy.reach_utilization) / std::max(config_.frequency.reach_envelope_soft_limit, 1e-6),
        config_.frequency.reach_envelope_min_scale,
        1.0);
    policy.cadence_hz = FrequencyHz{std::clamp(
        config_.frequency.min_hz.value +
            (config_.frequency.max_hz.value - config_.frequency.min_hz.value) * speed_mag * envelope_speed_scale,
        config_.frequency.min_hz.value,
        config_.frequency.max_hz.value)};

    policy.envelope = DynamicSafetyEnvelope{1.0, 1.0, true, kArcMaxRollPitchRad};
    const double yaw_normalized = std::clamp(
        std::abs(intent.body_pose_setpoint.angular_velocity_radps.z) /
            std::max(config_.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6),
        0.0,
        1.0);
    const double roll_pitch_abs_rad = std::max(
        std::abs(est.body_pose_state.orientation_rad.x),
        std::abs(est.body_pose_state.orientation_rad.y));
    policy.fallback_stage = selectFallbackStage(
        safety,
        policy.envelope,
        roll_pitch_abs_rad,
        command_norm,
        yaw_normalized,
        policy.reach_utilization);
    applyFallback(policy);
    applyServoVelocityConstraint(policy, est);
    return policy;
}

void GaitPolicyPlanner::applyEnvelopeAndFallback(RuntimeGaitPolicy& policy,
                                                 const RobotState& est,
                                                 const MotionIntent& intent,
                                                 const SafetyState& safety) const
{
    const double normalized_command = std::clamp(
        std::abs(intent.speed_mps.value) / std::max(config_.frequency.nominal_max_speed_mps.value, 1e-6),
        0.0,
        1.0);
    const double yaw_normalized = std::clamp(
        std::abs(intent.body_pose_setpoint.angular_velocity_radps.z) /
            std::max(config_.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6),
        0.0,
        1.0);
    const double roll_pitch_abs_rad = std::max(
        std::abs(est.body_pose_state.orientation_rad.x),
        std::abs(est.body_pose_state.orientation_rad.y));

    policy.envelope = envelopeForRegion(policy.region);
    if (!policy.envelope.allow_tripod && policy.gait_family == GaitType::TRIPOD) {
        policy.gait_family = GaitType::RIPPLE;
    }
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);
    policy.fallback_stage = selectFallbackStage(
        safety,
        policy.envelope,
        roll_pitch_abs_rad,
        normalized_command,
        yaw_normalized,
        policy.reach_utilization);
}

DynamicGaitRegion GaitPolicyPlanner::selectRegion(const MotionIntent& intent)
{
    const RegionClassificationState next_state = RegionClassifier::classify(
        intent,
        config_,
        RegionClassificationState{last_region_, filtered_speed_norm_, filtered_yaw_norm_});
    filtered_speed_norm_ = next_state.filtered_speed_norm;
    filtered_yaw_norm_ = next_state.filtered_yaw_norm;
    last_region_ = next_state.region;
    return last_region_;
}

GaitType GaitPolicyPlanner::selectGaitFamily(const MotionIntent& intent, const DynamicGaitRegion region)
{
    (void)intent;
    last_gait_family_ = GaitFamilySelector::select(
        region,
        filtered_speed_norm_,
        filtered_yaw_norm_,
        last_gait_family_);
    return last_gait_family_;
}

TurnMode GaitPolicyPlanner::selectTurnMode(const DynamicGaitRegion region) const
{
    if (region == DynamicGaitRegion::PIVOT) {
        return TurnMode::IN_PLACE;
    }
    return TurnMode::CRAB;
}

std::array<LegDynamicGaitParams, kNumLegs> GaitPolicyPlanner::computePerLegDynamicParameters(GaitType family) const
{
    std::array<LegDynamicGaitParams, kNumLegs> params{};
    const std::array<double, kNumLegs>& offsets = gaitPhaseOffsets(config_.phase_offsets, family);
    const double duty = gaitDutyCycle(config_.duty, family);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        params[leg].phase_offset = offsets[leg];
        params[leg].duty_cycle = duty;
        params[leg].swing_height_m = config_.swing.height_m;
        params[leg].step_length_m = config_.foothold.step_length_m;
    }

    return params;
}

GaitSuppressionFlags GaitPolicyPlanner::computeSuppressionFlags(const RobotState& est,
                                                                const MotionIntent& intent,
                                                                const SafetyState& safety,
                                                                TurnMode turn_mode) const
{
    GaitSuppressionFlags flags{};
    const bool walking_requested = intent.requested_mode == RobotMode::WALK;
    const bool safe_to_walk = walking_requested && !safety.inhibit_motion && !safety.torque_cut && safety.stable;
    const double reach = maxReachUtilization(est);

    flags.suppress_stride_progression = !safe_to_walk || (reach > 1.02);
    flags.suppress_turning = (turn_mode == TurnMode::CRAB &&
                              std::abs(intent.body_pose_setpoint.angular_velocity_radps.z) <
                                  config_.turn_mode_thresholds.yaw_rate_exit_radps.value);
    flags.prioritize_stability = config_.priority_suppression.stability_priority >= 1.0;
    return flags;
}

DynamicSafetyEnvelope GaitPolicyPlanner::envelopeForRegion(const DynamicGaitRegion region) const
{
    switch (region) {
        case DynamicGaitRegion::ARC:
            return DynamicSafetyEnvelope{1.0, 0.35, true, kArcMaxRollPitchRad};
        case DynamicGaitRegion::PIVOT:
            return DynamicSafetyEnvelope{0.30, 0.90, false, kPivotMaxRollPitchRad};
        case DynamicGaitRegion::REORIENTATION:
            return DynamicSafetyEnvelope{0.60, 0.85, false, kReorientationMaxRollPitchRad};
    }

    return DynamicSafetyEnvelope{};
}

GaitFallbackStage GaitPolicyPlanner::selectFallbackStage(const SafetyState& safety,
                                                         const DynamicSafetyEnvelope& envelope,
                                                         const double roll_pitch_abs_rad,
                                                         const double speed_normalized,
                                                         const double yaw_normalized,
                                                         const double reach_utilization) const
{
    if (safety.active_fault != FaultCode::NONE) {
        return GaitFallbackStage::FAULT_HOLD;
    }

    if (safety.inhibit_motion || safety.torque_cut) {
        return GaitFallbackStage::SAFE_STOP;
    }

    if (roll_pitch_abs_rad > kHighRiskRollPitchRad ||
        reach_utilization > kFaultHoldReachThreshold ||
        !safety.stable) {
        return GaitFallbackStage::DEGRADED_LOCOMOTION;
    }

    if (roll_pitch_abs_rad > envelope.max_roll_pitch_rad ||
        speed_normalized > envelope.max_speed_normalized ||
        yaw_normalized > envelope.max_yaw_normalized ||
        reach_utilization > kDegradedReachThreshold) {
        return GaitFallbackStage::STABILITY;
    }

    return GaitFallbackStage::NONE;
}

void GaitPolicyPlanner::applyFallback(RuntimeGaitPolicy& policy) const
{
    switch (policy.fallback_stage) {
        case GaitFallbackStage::NONE:
            return;
        case GaitFallbackStage::STABILITY:
            policy.gait_family = GaitType::WAVE;
            for (auto& leg : policy.per_leg) {
                leg.duty_cycle = std::max(leg.duty_cycle, 0.76);
                leg.step_length_m = LengthM{leg.step_length_m.value * 0.70};
            }
            break;
        case GaitFallbackStage::DEGRADED_LOCOMOTION:
            policy.gait_family = GaitType::WAVE;
            policy.cadence_hz = FrequencyHz{std::min(policy.cadence_hz.value, 1.0)};
            for (auto& leg : policy.per_leg) {
                leg.duty_cycle = std::max(leg.duty_cycle, 0.84);
                leg.step_length_m = LengthM{leg.step_length_m.value * 0.45};
                leg.swing_height_m = LengthM{leg.swing_height_m.value * 0.85};
            }
            break;
        case GaitFallbackStage::SAFE_STOP:
            policy.gait_family = GaitType::WAVE;
            policy.cadence_hz = FrequencyHz{0.0};
            policy.suppression.suppress_stride_progression = true;
            policy.suppression.suppress_turning = true;
            for (auto& leg : policy.per_leg) {
                leg.duty_cycle = 0.95;
                leg.step_length_m = LengthM{0.0};
            }
            break;
        case GaitFallbackStage::FAULT_HOLD:
            policy.gait_family = GaitType::WAVE;
            policy.cadence_hz = FrequencyHz{0.0};
            policy.suppression.suppress_stride_progression = true;
            policy.suppression.suppress_turning = true;
            policy.suppression.prioritize_stability = true;
            for (auto& leg : policy.per_leg) {
                leg.duty_cycle = 0.95;
                leg.step_length_m = LengthM{0.0};
                leg.swing_height_m = LengthM{0.0};
            }
            break;
    }
}

void GaitPolicyPlanner::applyServoVelocityConstraint(RuntimeGaitPolicy& policy, const RobotState& est) const
{
    if (policy.cadence_hz.value <= 0.0 ||
        policy.fallback_stage == GaitFallbackStage::SAFE_STOP ||
        policy.fallback_stage == GaitFallbackStage::FAULT_HOLD) {
        return;
    }

    double observed_peak_velocity_radps = 0.0;
    for (const auto& leg : est.leg_states) {
        for (const auto& joint : leg.joint_state) {
            observed_peak_velocity_radps =
                std::max(observed_peak_velocity_radps, std::abs(joint.vel_radps.value));
        }
    }
    if (observed_peak_velocity_radps <= 0.0) {
        return;
    }

    double min_servo_vmax_radps = std::numeric_limits<double>::infinity();
    for (const auto& leg : geometry_.legGeometry) {
        for (const auto& dynamics : leg.servoDynamics) {
            min_servo_vmax_radps = std::min(min_servo_vmax_radps, dynamics.positive_direction.vmax_radps);
            min_servo_vmax_radps = std::min(min_servo_vmax_radps, dynamics.negative_direction.vmax_radps);
        }
    }
    if (!std::isfinite(min_servo_vmax_radps) || min_servo_vmax_radps <= 1e-9) {
        return;
    }

    const double allowed_peak_velocity_radps = 2.0 * min_servo_vmax_radps;
    if (observed_peak_velocity_radps <= allowed_peak_velocity_radps) {
        return;
    }

    const double gait_scale =
        std::clamp(allowed_peak_velocity_radps / observed_peak_velocity_radps, 0.05, 1.0);
    policy.adaptation_scale_cadence = gait_scale;
    policy.adaptation_scale_step = gait_scale;
    policy.hard_clamp_cadence = gait_scale < 0.999;
    policy.cadence_hz = FrequencyHz{policy.cadence_hz.value * gait_scale};
    for (auto& leg : policy.per_leg) {
        leg.step_length_m = LengthM{leg.step_length_m.value * gait_scale};
        leg.swing_height_m = LengthM{leg.swing_height_m.value * std::sqrt(gait_scale)};
        leg.duty_cycle = std::clamp(leg.duty_cycle + ((1.0 - gait_scale) * 0.15), 0.05, 0.95);
    }

    if (gait_scale < 0.75 && policy.gait_family == GaitType::TRIPOD) {
        policy.gait_family = GaitType::RIPPLE;
    } else if (gait_scale < 0.50) {
        policy.gait_family = GaitType::WAVE;
    }
}

double GaitPolicyPlanner::maxReachUtilization(const RobotState& est) const
{
    double max_utilization = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const LegState joint_frame = leg_geo.servo.toJointAngles(est.leg_states[leg]);
        const Vec3 foot_leg = kinematics::footInLegFrame(joint_frame, leg_geo);

        max_utilization = std::max(max_utilization, kinematics::legReachUtilization(foot_leg, leg_geo));
    }

    return max_utilization;
}
