#include "gait_policy_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "reach_envelope.hpp"

namespace {
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
    policy.reach_utilization = maxReachUtilization(est);
    const double commanded_speed = std::abs(intent.speed_mps.value);
    const double normalized_command =
        std::clamp(commanded_speed / config_.frequency.nominal_max_speed_mps.value, 0.0, 1.0);
    const double yaw_normalized = std::clamp(
        std::abs(intent.twist.twist_vel_radps.z) /
            std::max(config_.turn_mode_thresholds.yaw_rate_enter_radps.value, 1e-6),
        0.0,
        1.0);

    policy.region = classifyRegion(normalized_command, yaw_normalized);
    policy.envelope = envelopeForRegion(policy.region);
    policy.gait_family = selectGaitFamily(intent);
    if (!policy.envelope.allow_tripod && policy.gait_family == GaitType::TRIPOD) {
        policy.gait_family = GaitType::RIPPLE;
    }
    policy.turn_mode = selectTurnMode(intent);
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

GaitType GaitPolicyPlanner::selectGaitFamily(const MotionIntent& intent) const
{
    return intent.gait;
}

TurnMode GaitPolicyPlanner::selectTurnMode(const MotionIntent& intent)
{
    const double speed = std::abs(intent.speed_mps.value);
    const double yaw_rate = std::abs(intent.twist.twist_vel_radps.z);

    if (last_turn_mode_ == TurnMode::IN_PLACE) {
        if (yaw_rate <= config_.turn_mode_thresholds.yaw_rate_exit_radps.value ||
            speed >= config_.turn_mode_thresholds.speed_exit_mps.value) {
            last_turn_mode_ = TurnMode::CRAB;
        }
        return last_turn_mode_;
    }

    if (yaw_rate >= config_.turn_mode_thresholds.yaw_rate_enter_radps.value &&
        speed <= config_.turn_mode_thresholds.speed_enter_mps.value) {
        last_turn_mode_ = TurnMode::IN_PLACE;
    }
    return last_turn_mode_;
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

DynamicGaitRegion GaitPolicyPlanner::classifyRegion(const double speed_normalized, const double yaw_normalized)
{
    if (last_region_ == DynamicGaitRegion::ARC) {
        if (yaw_normalized > 0.60 || speed_normalized < 0.16) {
            last_region_ = DynamicGaitRegion::REORIENTATION;
        }
        return last_region_;
    }

    if (last_region_ == DynamicGaitRegion::PIVOT) {
        if (yaw_normalized < 0.52 && speed_normalized > 0.18) {
            last_region_ = DynamicGaitRegion::REORIENTATION;
        }
        return last_region_;
    }

    if (speed_normalized >= 0.22 && yaw_normalized <= 0.52) {
        last_region_ = DynamicGaitRegion::ARC;
    } else if (speed_normalized <= 0.22 && yaw_normalized >= 0.60) {
        last_region_ = DynamicGaitRegion::PIVOT;
    } else {
        last_region_ = DynamicGaitRegion::REORIENTATION;
    }
    return last_region_;
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
