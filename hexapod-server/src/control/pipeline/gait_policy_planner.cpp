#include "gait_policy_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "reach_envelope.hpp"

namespace {

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
    policy.gait_family = selectGaitFamily(intent);
    policy.turn_mode = selectTurnMode(intent);
    policy.per_leg = computePerLegDynamicParameters(policy.gait_family);
    policy.suppression = computeSuppressionFlags(est, intent, safety, policy.turn_mode);
    policy.reach_utilization = maxReachUtilization(est);
    const double commanded_speed = std::abs(intent.speed_mps.value);
    const double normalized_command =
        std::clamp(commanded_speed / config_.frequency.nominal_max_speed_mps.value, 0.0, 1.0);
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
