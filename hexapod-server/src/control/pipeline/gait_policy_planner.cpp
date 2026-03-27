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
constexpr double kFiniteDifferenceStepRad = 1e-4;
constexpr double kMinMotionLimiterScale = 0.30;

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

double minJointVmaxRadps(const LegGeometry& leg_geo, const int joint_idx)
{
    const ServoJointDynamics& dynamics = leg_geo.servoDynamics[joint_idx];
    return std::max(0.0, std::min(dynamics.positive_direction.vmax_radps, dynamics.negative_direction.vmax_radps));
}

double minJointAmaxRadps2(const LegGeometry& leg_geo, const int joint_idx)
{
    const ServoJointDynamics& dynamics = leg_geo.servoDynamics[joint_idx];
    const double vmax = minJointVmaxRadps(leg_geo, joint_idx);
    const double tau = std::max(1e-3, std::min(dynamics.positive_direction.tau_s, dynamics.negative_direction.tau_s));
    return vmax / tau;
}

double predictedFootSpeedUpperBound(const RuntimeGaitPolicy& policy)
{
    if (policy.cadence_hz.value <= 1e-9) {
        return 0.0;
    }

    double peak = 0.0;
    for (const auto& leg : policy.per_leg) {
        const double duty = std::clamp(leg.duty_cycle, 0.05, 0.95);
        const double phase_rate = std::max(0.0, policy.cadence_hz.value);
        const double step = std::max(0.0, leg.step_length_m.value);
        const double swing = std::max(0.0, leg.swing_height_m.value);

        const double stance_xy = kPi * step * phase_rate;
        const double swing_xy = 3.0 * step * phase_rate / std::max(1.0 - duty, 1e-3);
        const double swing_z = 2.0 * kPi * swing * phase_rate / std::max(1.0 - duty, 1e-3);
        peak = std::max(peak, stance_xy + swing_xy + swing_z);
    }
    return peak;
}

double predictedFootAccelUpperBound(const RuntimeGaitPolicy& policy)
{
    if (policy.cadence_hz.value <= 1e-9) {
        return 0.0;
    }

    double peak = 0.0;
    for (const auto& leg : policy.per_leg) {
        const double duty = std::clamp(leg.duty_cycle, 0.05, 0.95);
        const double phase_rate = std::max(0.0, policy.cadence_hz.value);
        const double step = std::max(0.0, leg.step_length_m.value);
        const double swing = std::max(0.0, leg.swing_height_m.value);
        const double rate_sq = phase_rate * phase_rate;

        const double stance_xy = 2.0 * kPi * kPi * step * rate_sq;
        const double swing_xy = 18.0 * step * rate_sq /
                                std::max((1.0 - duty) * (1.0 - duty), 1e-4);
        const double swing_z = 4.0 * kPi * kPi * swing * rate_sq /
                               std::max((1.0 - duty) * (1.0 - duty), 1e-4);
        peak = std::max(peak, stance_xy + swing_xy + swing_z);
    }
    return peak;
}

bool buildLocalJacobian(const LegGeometry& leg_geo, const LegState& leg_state, double jac[3][3])
{
    const LegState joint_frame = leg_geo.servo.toJointAngles(leg_state);
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        LegState plus = joint_frame;
        LegState minus = joint_frame;
        plus.joint_state[joint].pos_rad.value += kFiniteDifferenceStepRad;
        minus.joint_state[joint].pos_rad.value -= kFiniteDifferenceStepRad;

        const Vec3 foot_plus = kinematics::footInLegFrame(plus, leg_geo);
        const Vec3 foot_minus = kinematics::footInLegFrame(minus, leg_geo);
        if (!isFinite(foot_plus) || !isFinite(foot_minus)) {
            return false;
        }

        const Vec3 column = (foot_plus - foot_minus) * (0.5 / kFiniteDifferenceStepRad);
        jac[0][joint] = column.x;
        jac[1][joint] = column.y;
        jac[2][joint] = column.z;
    }
    return true;
}

double estimateJointDemandRatio(const RuntimeGaitPolicy& policy,
                                const RobotState& est,
                                const HexapodGeometry& geometry)
{
    const double predicted_speed = predictedFootSpeedUpperBound(policy);
    const double predicted_accel = predictedFootAccelUpperBound(policy);
    if (predicted_speed <= 1e-9 && predicted_accel <= 1e-9) {
        return 1.0;
    }

    const std::array<Vec3, 6> probing_dirs{
        Vec3{1.0, 0.0, 0.0},
        Vec3{0.0, 1.0, 0.0},
        Vec3{0.0, 0.0, 1.0},
        Vec3{-1.0, 0.0, 0.0},
        Vec3{0.0, -1.0, 0.0},
        Vec3{0.0, 0.0, -1.0},
    };

    double min_ratio = 1.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        double jac[3][3]{};
        if (!buildLocalJacobian(leg_geo, est.leg_states[leg], jac)) {
            continue;
        }

        for (const Vec3& dir : probing_dirs) {
            double qd0 = 0.0;
            double qd1 = 0.0;
            double qd2 = 0.0;
            if (!solve3x3Cramers(jac[0][0], jac[0][1], jac[0][2],
                                 jac[1][0], jac[1][1], jac[1][2],
                                 jac[2][0], jac[2][1], jac[2][2],
                                 dir.x * predicted_speed, dir.y * predicted_speed, dir.z * predicted_speed,
                                 qd0, qd1, qd2, 1e-9)) {
                min_ratio = std::min(min_ratio, 0.4);
                continue;
            }

            double qdd0 = 0.0;
            double qdd1 = 0.0;
            double qdd2 = 0.0;
            const bool accel_ok = solve3x3Cramers(jac[0][0], jac[0][1], jac[0][2],
                                                  jac[1][0], jac[1][1], jac[1][2],
                                                  jac[2][0], jac[2][1], jac[2][2],
                                                  dir.x * predicted_accel, dir.y * predicted_accel, dir.z * predicted_accel,
                                                  qdd0, qdd1, qdd2, 1e-9);

            const std::array<double, kJointsPerLeg> qd{qd0, qd1, qd2};
            const std::array<double, kJointsPerLeg> qdd{qdd0, qdd1, qdd2};
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double vmax = minJointVmaxRadps(leg_geo, joint);
                if (vmax > 1e-9) {
                    const double current = std::abs(est.leg_states[leg].joint_state[joint].vel_radps.value);
                    const double remaining = std::max(vmax - current, 0.0);
                    const double occupancy_margin =
                        std::clamp(remaining / std::max(0.25 * vmax, 1e-6), 0.0, 1.0);
                    min_ratio = std::min(min_ratio, occupancy_margin);
                    const double demand = std::abs(qd[joint]);
                    if (demand > 1e-9) {
                        min_ratio = std::min(min_ratio, remaining / demand);
                    }
                }

                if (accel_ok) {
                    const double amax = minJointAmaxRadps2(leg_geo, joint);
                    const double demand_accel = std::abs(qdd[joint]);
                    if (amax > 1e-9 && demand_accel > 1e-9) {
                        min_ratio = std::min(min_ratio, amax / demand_accel);
                    }
                }
            }
        }
    }

    return std::clamp(min_ratio, 0.0, 1.0);
}

void applyGaitScale(RuntimeGaitPolicy& policy, const double gait_scale)
{
    const double scale = std::clamp(gait_scale, kMinMotionLimiterScale, 1.0);
    policy.cadence_hz = FrequencyHz{policy.cadence_hz.value * scale};
    for (auto& leg : policy.per_leg) {
        leg.step_length_m = LengthM{leg.step_length_m.value * scale};
        leg.swing_height_m = LengthM{leg.swing_height_m.value * std::sqrt(scale)};
        leg.duty_cycle = std::clamp(leg.duty_cycle + (1.0 - scale) * 0.20, 0.05, 0.95);
    }

    if (scale < 0.72 && policy.gait_family == GaitType::TRIPOD) {
        policy.gait_family = GaitType::RIPPLE;
    }
    if (scale < 0.52 && policy.gait_family != GaitType::WAVE) {
        policy.gait_family = GaitType::WAVE;
    }
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
    applyMotionLimiter(policy, est);
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
    applyMotionLimiter(policy, est);
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

void GaitPolicyPlanner::applyMotionLimiter(RuntimeGaitPolicy& policy, const RobotState& est) const
{
    if (policy.cadence_hz.value <= 0.0 ||
        policy.fallback_stage == GaitFallbackStage::SAFE_STOP ||
        policy.fallback_stage == GaitFallbackStage::FAULT_HOLD) {
        return;
    }

    const double margin_scale = estimateJointDemandRatio(policy, est, geometry_);
    if (margin_scale >= 0.999) {
        return;
    }

    applyGaitScale(policy, margin_scale);

    if (margin_scale < kMinMotionLimiterScale) {
        policy.cadence_hz = FrequencyHz{std::min(policy.cadence_hz.value, 0.8)};
        policy.gait_family = GaitType::WAVE;
        for (auto& leg : policy.per_leg) {
            leg.step_length_m = LengthM{leg.step_length_m.value * 0.60};
            leg.swing_height_m = LengthM{leg.swing_height_m.value * 0.85};
            leg.duty_cycle = std::max(leg.duty_cycle, 0.88);
        }
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
