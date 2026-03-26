#include "gait_policy_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

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

double clamp01(const double value)
{
    return std::clamp(value, 0.0, 1.0);
}

double normalizedSpeed(const MotionIntent& intent, const control_config::GaitConfig& config)
{
    const double max_speed = std::max(config.frequency.nominal_max_speed_mps.value, 1e-6);
    return clamp01(std::abs(intent.speed_mps.value) / max_speed);
}

double normalizedYaw(const MotionIntent& intent, const control_config::GaitConfig& config)
{
    const double yaw_enter = std::max(config.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6);
    return clamp01(std::abs(intent.twist.twist_vel_radps.z) / yaw_enter);
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

} // namespace

GaitPolicyPlanner::GaitPolicyPlanner(control_config::GaitConfig config)
    : config_(config) {}

RuntimeGaitPolicy GaitPolicyPlanner::plan(const RobotState& est,
                                          const MotionIntent& intent,
                                          const SafetyState& safety)
{
    RuntimeGaitPolicy policy{};
    policy.region = selectRegion(intent);
    policy.gait_family = selectGaitFamily(intent, policy.region);
    policy.turn_mode = selectTurnMode(policy.region);
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);
    policy.reach_utilization = maxReachUtilization(est);
    const double commanded_speed = std::abs(intent.speed_mps.value);
    const double normalized_command =
        std::clamp(commanded_speed / config_.frequency.nominal_max_speed_mps.value, 0.0, 1.0);
    const double yaw_normalized = std::clamp(
        std::abs(intent.twist.twist_vel_radps.z) /
            std::max(config_.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6),
        0.0,
        1.0);

    policy.envelope = envelopeForRegion(policy.region);
    if (!policy.envelope.allow_tripod && policy.gait_family == GaitType::TRIPOD) {
        policy.gait_family = GaitType::RIPPLE;
    }
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);

    const double roll_pitch_abs_rad = std::max(
        std::abs(est.body_twist_state.twist_pos_rad.x),
        std::abs(est.body_twist_state.twist_pos_rad.y));
    policy.fallback_stage = selectFallbackStage(
        safety,
        policy.envelope,
        roll_pitch_abs_rad,
        normalized_command,
        yaw_normalized,
        policy.reach_utilization);

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
    return policy;
}

DynamicGaitRegion GaitPolicyPlanner::selectRegion(const MotionIntent& intent)
{
    const double speed = normalizedSpeed(intent, config_);
    const double yaw = normalizedYaw(intent, config_);
    filtered_speed_norm_ = std::lerp(filtered_speed_norm_, speed, kSmoothCommandAlpha);
    filtered_yaw_norm_ = std::lerp(filtered_yaw_norm_, yaw, kSmoothCommandAlpha);

    if (last_region_ == DynamicGaitRegion::ARC) {
        if (filtered_yaw_norm_ > kRegionArcExitYaw || filtered_speed_norm_ < kRegionArcExitSpeed) {
            last_region_ = DynamicGaitRegion::REORIENTATION;
        }
        return last_region_;
    }

    if (last_region_ == DynamicGaitRegion::PIVOT) {
        if (filtered_yaw_norm_ < kRegionPivotExitYaw && filtered_speed_norm_ > kRegionPivotExitSpeed) {
            last_region_ = DynamicGaitRegion::REORIENTATION;
        }
        return last_region_;
    }

    if (filtered_speed_norm_ >= kRegionReorientToArcSpeed && filtered_yaw_norm_ <= kRegionReorientToArcYaw) {
        last_region_ = DynamicGaitRegion::ARC;
    } else if (filtered_speed_norm_ <= kRegionReorientToPivotSpeed &&
               filtered_yaw_norm_ >= kRegionReorientToPivotYaw) {
        last_region_ = DynamicGaitRegion::PIVOT;
    } else if (filtered_speed_norm_ >= kRegionArcEnterSpeed && filtered_yaw_norm_ <= kRegionArcEnterYaw) {
        last_region_ = DynamicGaitRegion::ARC;
    } else if (filtered_speed_norm_ <= kRegionPivotEnterSpeed && filtered_yaw_norm_ >= kRegionPivotEnterYaw) {
        last_region_ = DynamicGaitRegion::PIVOT;
    } else {
        last_region_ = DynamicGaitRegion::REORIENTATION;
    }

    return last_region_;
}

GaitType GaitPolicyPlanner::selectGaitFamily(const MotionIntent& intent, const DynamicGaitRegion region)
{
    (void)intent;
    const double speed = filtered_speed_norm_;
    const double yaw = filtered_yaw_norm_;

    auto keepStableTransitions = [&](const GaitType candidate) {
        if (candidate == last_gait_family_) {
            return candidate;
        }

        if (candidate == GaitType::TRIPOD && last_gait_family_ == GaitType::RIPPLE &&
            yaw > (0.35 - kGaitSwitchYawHysteresis)) {
            return last_gait_family_;
        }
        if (last_gait_family_ == GaitType::WAVE && candidate == GaitType::RIPPLE &&
            speed < (0.35 + kGaitSwitchSpeedHysteresis)) {
            return last_gait_family_;
        }
        if (last_gait_family_ == GaitType::RIPPLE && candidate == GaitType::TRIPOD &&
            speed < (0.70 + kGaitSwitchSpeedHysteresis)) {
            return last_gait_family_;
        }
        return candidate;
    };

    if (region == DynamicGaitRegion::PIVOT) {
        const GaitType candidate = (speed >= 0.15 && yaw <= 0.85) ? GaitType::RIPPLE : GaitType::WAVE;
        last_gait_family_ = keepStableTransitions(candidate);
        return last_gait_family_;
    }
    if (region == DynamicGaitRegion::REORIENTATION) {
        const GaitType candidate = (speed < 0.30 || yaw > 0.75) ? GaitType::WAVE : GaitType::RIPPLE;
        last_gait_family_ = keepStableTransitions(candidate);
        return last_gait_family_;
    }

    GaitType candidate = GaitType::WAVE;
    if (speed < 0.35) {
        candidate = GaitType::WAVE;
    } else if (speed < 0.70) {
        candidate = GaitType::RIPPLE;
    } else if (yaw <= 0.35) {
        candidate = GaitType::TRIPOD;
    } else {
        candidate = GaitType::RIPPLE;
    }

    last_gait_family_ = keepStableTransitions(candidate);
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
                              std::abs(intent.twist.twist_vel_radps.z) <
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

double GaitPolicyPlanner::maxReachUtilization(const RobotState& est) const
{
    double max_utilization = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const LegState joint_frame = leg_geo.servo.toJointAngles(est.leg_states[leg]);

        const double q1 = joint_frame.joint_state[0].pos_rad.value;
        const double q2 = joint_frame.joint_state[1].pos_rad.value;
        const double q3 = joint_frame.joint_state[2].pos_rad.value;

        const double rho =
            leg_geo.femurLength.value * std::cos(q2) +
            leg_geo.tibiaLength.value * std::cos(q2 + q3);
        const double z =
            leg_geo.femurLength.value * std::sin(q2) +
            leg_geo.tibiaLength.value * std::sin(q2 + q3);
        const double r = leg_geo.coxaLength.value + rho;
        const Vec3 foot_leg{r * std::cos(q1), r * std::sin(q1), z};

        max_utilization = std::max(max_utilization, kinematics::legReachUtilization(foot_leg, leg_geo));
    }

    return max_utilization;
}
