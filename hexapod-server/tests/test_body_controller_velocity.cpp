#include "body_controller.hpp"
#include "leg_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "reach_envelope.hpp"
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

bool finiteJointTargets(const JointTargets& targets) {
    for (const auto& leg : targets.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

double legVelocityMagnitude(const FootTarget& target) {
    const Vec3 v = target.vel_body_mps;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double totalLegPositionDelta(const LegTargets& lhs, const LegTargets& rhs) {
    double total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = lhs.feet[leg].pos_body_m - rhs.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    return total_delta;
}
constexpr double kLevelHoldRollPitchKp = 0.6;
constexpr double kLevelHoldMaxCorrectionRad = 0.18;
constexpr double kLevelHoldMaxCommandRad = 0.30;
constexpr double kLevelHoldEstimateLpfAlpha = 0.25;
double wrappedAngleDistance(double a_rad, double b_rad) {
    const double wrapped =
        std::remainder(a_rad - b_rad, 2.0 * kPi);
    return std::abs(wrapped);
}

bool allFiniteTargets(const LegTargets& targets) {
    for (const auto& foot : targets.feet) {
        if (!std::isfinite(foot.pos_body_m.x) ||
            !std::isfinite(foot.pos_body_m.y) ||
            !std::isfinite(foot.pos_body_m.z) ||
            !std::isfinite(foot.vel_body_mps.x) ||
            !std::isfinite(foot.vel_body_mps.y) ||
            !std::isfinite(foot.vel_body_mps.z)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    BodyController controller{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent stand_intent{};
    stand_intent.requested_mode = RobotMode::STAND;
    stand_intent.body_pose_setpoint.body_trans_mps = Vec3{0.10, -0.20, 0.05};
    stand_intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.3};
    stand_intent.body_pose_setpoint.body_trans_m.z = 0.20;

    GaitState gait{};
    const LegTargets stand_targets = controller.update(est, stand_intent, gait, safety);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 expected = (Vec3{-stand_intent.body_pose_setpoint.body_trans_mps.x,
                                    -stand_intent.body_pose_setpoint.body_trans_mps.y,
                                    -stand_intent.body_pose_setpoint.body_trans_mps.z}) +
                              cross(stand_intent.body_pose_setpoint.angular_velocity_radps, stand_targets.feet[leg].pos_body_m);
        const Vec3 actual = stand_targets.feet[leg].vel_body_mps;
        if (!expect(nearlyEqual(actual.x, expected.x) &&
                        nearlyEqual(actual.y, expected.y) &&
                        nearlyEqual(actual.z, expected.z),
                    "stand mode velocity should include body translation and angular feed-forward")) {
            return EXIT_FAILURE;
        }
    }

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.speed_mps = LinearRateMps{0.2};
    walk_intent.heading_rad = AngleRad{0.0};
    walk_intent.body_pose_setpoint.body_trans_m.z = 0.20;

    GaitState walk_gait{};
    walk_gait.phase[0] = 0.25;
    walk_gait.in_stance[0] = true;
    walk_gait.stride_phase_rate_hz = FrequencyHz{1.0};
    const LegTargets walk_targets = controller.update(est, walk_intent, walk_gait, safety);
    const double expected_walk_x_vel =
        -kPi * 0.06 * walk_gait.stride_phase_rate_hz.value * std::sin(2.0 * kPi * walk_gait.phase[0]);

    if (!expect(nearlyEqual(walk_targets.feet[0].vel_body_mps.x, expected_walk_x_vel, 1e-6),
                "walking stance velocity should include analytic step derivative")) {
        return EXIT_FAILURE;
    }

    MotionIntent aggressive_intent{};
    aggressive_intent.requested_mode = RobotMode::WALK;
    aggressive_intent.speed_mps = LinearRateMps{0.30};
    aggressive_intent.heading_rad = AngleRad{0.0};
    aggressive_intent.body_pose_setpoint.body_trans_m = Vec3{0.38, 0.14, 0.20};
    aggressive_intent.body_pose_setpoint.body_trans_mps = Vec3{0.40, -0.30, 0.00};
    aggressive_intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 1.2};

    GaitState aggressive_gait{};
    aggressive_gait.in_stance.fill(true);
    aggressive_gait.phase.fill(0.20);
    aggressive_gait.stride_phase_rate_hz = FrequencyHz{1.40};

    const LegTargets aggressive_targets = controller.update(est, aggressive_intent, aggressive_gait, safety);
    if (!expect(controller.lastMotionLimiterTelemetry().hard_clamp_reach,
                "aggressive body commands should activate hard reach clamp telemetry")) {
        return EXIT_FAILURE;
    }

    const HexapodGeometry geometry = defaultHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const Vec3 relative = aggressive_targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
        const Vec3 foot_leg = Mat3::rotZ(-leg_geo.mountAngle.value) * relative;
        if (!expect(kinematics::legReachUtilization(foot_leg, leg_geo) <= 1.0 + 1e-6,
                    "hard-clamped aggressive commands must remain in reachable envelope")) {
            return EXIT_FAILURE;
        }
    }

    LegIK ik{geometry};
    RobotState ik_est{};
    for (auto& leg : ik_est.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.9};
    }

    double max_joint_step = 0.0;
    for (int step = 0; step < 12; ++step) {
        MotionIntent iter_intent = aggressive_intent;
        const double step_fraction = static_cast<double>(step) / 11.0;
        iter_intent.body_pose_setpoint.body_trans_m.x += 0.06 * std::sin(step_fraction * 2.0 * kPi);
        iter_intent.body_pose_setpoint.body_trans_m.y += 0.03 * std::cos(step_fraction * 2.0 * kPi);
        iter_intent.body_pose_setpoint.angular_velocity_radps.z += 0.4 * std::sin(step_fraction * 2.0 * kPi);

        const LegTargets iter_targets = controller.update(ik_est, iter_intent, aggressive_gait, safety);
        const JointTargets iter_joints = ik.solve(ik_est, iter_targets, safety);
        if (!expect(finiteJointTargets(iter_joints),
                    "clamped aggressive commands should produce finite IK joint outputs")) {
            return EXIT_FAILURE;
        }

        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double delta = std::abs(
                    wrappedAngleDistance(
                        iter_joints.leg_states[leg].joint_state[joint].pos_rad.value,
                        ik_est.leg_states[leg].joint_state[joint].pos_rad.value));
                max_joint_step = std::max(max_joint_step, delta);
            }
        }
        ik_est.leg_states = iter_joints.leg_states;
    }

    if (!expect(max_joint_step < 3.0,
                "aggressive clamped commands should not cause extreme inter-step joint jumps")) {
        return EXIT_FAILURE;
    }
    if (!expect(max_joint_step > 1e-3,
                "IK output should respond to aggressive commands (no persistent fallback behavior)")) {
        return EXIT_FAILURE;
    }

    MotionIntent offset_intent{};
    offset_intent.requested_mode = RobotMode::STAND;
    offset_intent.body_pose_setpoint.body_trans_m = Vec3{0.25, 0.0, 0.20};

    const LegTargets offset_targets = controller.update(est, offset_intent, gait, safety);
    if (!expect(controller.lastMotionLimiterTelemetry().hard_clamp_reach,
                "large body offset should report hard reach clamp telemetry")) {
        return EXIT_FAILURE;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const Vec3 relative = offset_targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
        const Vec3 foot_leg = Mat3::rotZ(-leg_geo.mountAngle.value) * relative;
        if (!expect(kinematics::legReachUtilization(foot_leg, leg_geo) <= 1.0 + 1e-6,
                    "body controller should clamp target positions to leg reach envelope")) {
            return EXIT_FAILURE;
        }
    }

    MotionIntent turn_intent{};
    turn_intent.requested_mode = RobotMode::WALK;
    turn_intent.body_pose_setpoint.angular_velocity_radps.z = 0.8;
    turn_intent.body_pose_setpoint.body_trans_m.z = 0.20;

    GaitState turn_gait{};
    turn_gait.in_stance.fill(true);
    turn_gait.phase.fill(0.25);
    turn_gait.stride_phase_rate_hz = FrequencyHz{1.0};

    RuntimeGaitPolicy turn_policy{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        turn_policy.per_leg[leg].step_length_m = LengthM{0.06};
        turn_policy.per_leg[leg].swing_height_m = LengthM{0.03};
        turn_policy.per_leg[leg].duty_cycle = 0.5;
    }
    turn_policy.turn_mode = TurnMode::IN_PLACE;

    MotionIntent neutral_turn_intent = turn_intent;
    neutral_turn_intent.body_pose_setpoint.angular_velocity_radps.z = 0.0;
    const LegTargets neutral_turn_targets = controller.update(est, neutral_turn_intent, turn_gait, turn_policy, safety);
    const LegTargets turn_targets = controller.update(est, turn_intent, turn_gait, turn_policy, safety);

    double outer_sum = 0.0;
    double inner_sum = 0.0;
    int outer_count = 0;
    int inner_count = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta_vel = turn_targets.feet[leg].vel_body_mps - neutral_turn_targets.feet[leg].vel_body_mps;
        const double delta_mag = std::sqrt(delta_vel.x * delta_vel.x + delta_vel.y * delta_vel.y);
        if (geometry.legGeometry[leg].bodyCoxaOffset.y >= 0.0) {
            inner_sum += delta_mag;
            ++inner_count;
        } else {
            outer_sum += delta_mag;
            ++outer_count;
        }
    }

    const double outer_mag = outer_sum / std::max(outer_count, 1);
    const double inner_mag = inner_sum / std::max(inner_count, 1);
    if (!expect(std::abs(outer_mag - inner_mag) > 1e-4,
                "in-place turn should apply different scaling to inner/outer legs")) {
        return EXIT_FAILURE;
    }

    MotionIntent swing_intent{};
    swing_intent.requested_mode = RobotMode::WALK;
    swing_intent.body_pose_setpoint.body_trans_m.z = 0.20;
    swing_intent.body_pose_setpoint.body_trans_mps = Vec3{0.10, 0.0, 0.0};

    GaitState swing_gait{};
    swing_gait.in_stance.fill(false);
    swing_gait.phase.fill(0.75);
    swing_gait.stride_phase_rate_hz = FrequencyHz{1.0};

    MotionIntent swing_static_intent = swing_intent;
    swing_static_intent.body_pose_setpoint.body_trans_mps = Vec3{0.0, 0.0, 0.0};

    const LegTargets swing_static_targets = controller.update(est, swing_static_intent, swing_gait, safety);
    const LegTargets swing_targets = controller.update(est, swing_intent, swing_gait, safety);
    if (!expect(swing_targets.feet[0].pos_body_m.x < swing_static_targets.feet[0].pos_body_m.x,
                "swing touchdown prediction should account for twist-field stance velocity")) {
        return EXIT_FAILURE;
    }

    MotionIntent level_hold_intent{};
    level_hold_intent.requested_mode = RobotMode::STAND;
    level_hold_intent.body_pose_setpoint.body_trans_m.z = 0.20;
    level_hold_intent.body_pose_setpoint.orientation_rad = Vec3{0.0, 0.0, 0.0};

    SafetyState poor_support_safety = safety;
    poor_support_safety.support_contact_count = 2;
    poor_support_safety.stability_margin_m = 0.002;

    SafetyState good_support_safety = safety;
    good_support_safety.support_contact_count = 4;
    good_support_safety.stability_margin_m = 0.03;

    RobotState level_est{};
    level_est.body_pose_state.orientation_rad = Vec3{0.0, 0.0, 0.0};
    level_est.has_inferred_body_pose_state = true;
    level_est.has_body_pose_state = true;
    const LegTargets level_targets = controller.update(level_est, level_hold_intent, gait, good_support_safety);

    RobotState tilted_est{};
    tilted_est.body_pose_state.orientation_rad = Vec3{0.12, -0.10, 0.0};
    tilted_est.has_inferred_body_pose_state = true;
    tilted_est.has_body_pose_state = true;
    const LegTargets poor_support_targets = controller.update(tilted_est, level_hold_intent, gait, poor_support_safety);

    double total_delta = totalLegPositionDelta(poor_support_targets, level_targets);
    if (!expect(total_delta < 1e-6,
                "level hold should be disabled with inferred pose but poor support quality")) {
        return EXIT_FAILURE;
    }

    const LegTargets corrected_targets = controller.update(tilted_est, level_hold_intent, gait, good_support_safety);
    total_delta = totalLegPositionDelta(corrected_targets, level_targets);
    if (!expect(total_delta > 1e-4,
                "level hold should engage with inferred pose when support quality is sufficient")) {
        return EXIT_FAILURE;
    }

    RobotState tilted_measured_only = tilted_est;
    tilted_measured_only.has_inferred_body_pose_state = false;
    tilted_measured_only.has_measured_body_pose_state = true;
    tilted_measured_only.has_body_pose_state = true;
    const LegTargets measured_level_hold_targets = controller.update(tilted_measured_only, level_hold_intent, gait, safety);
    total_delta = totalLegPositionDelta(measured_level_hold_targets, level_targets);
    if (!expect(total_delta > 1e-4,
                "level hold should work when measured body pose provenance is available")) {
        return EXIT_FAILURE;
    }

    MotionIntent tilted_setpoint_intent = level_hold_intent;
    tilted_setpoint_intent.body_pose_setpoint.orientation_rad = Vec3{0.12, -0.10, 0.0};
    const LegTargets respected_setpoint_poor_support =
        controller.update(tilted_est, tilted_setpoint_intent, gait, poor_support_safety);
    total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = respected_setpoint_poor_support.feet[leg].pos_body_m - poor_support_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    if (!expect(total_delta > 1e-4,
                "non-zero setpoints should remain effective when level hold is disabled")) {
        return EXIT_FAILURE;
    }

    const LegTargets respected_setpoint_targets =
        controller.update(tilted_est, tilted_setpoint_intent, gait, good_support_safety);
    total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = respected_setpoint_targets.feet[leg].pos_body_m - corrected_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    if (!expect(total_delta > 1e-4,
                "non-zero roll/pitch setpoints should remain effective under level hold")) {
        return EXIT_FAILURE;
    }

    BodyController trajectory_controller{};
    MotionIntent trajectory_intent = level_hold_intent;
    trajectory_intent.body_pose_setpoint.orientation_rad = Vec3{0.08, -0.06, 0.0};

    RobotState no_pose_est{};
    no_pose_est.body_pose_state.orientation_rad = trajectory_intent.body_pose_setpoint.orientation_rad;
    no_pose_est.has_inferred_body_pose_state = false;
    no_pose_est.has_measured_body_pose_state = false;
    no_pose_est.has_body_pose_state = false;

    RobotState matching_est{};
    matching_est.body_pose_state.orientation_rad = trajectory_intent.body_pose_setpoint.orientation_rad;
    matching_est.has_inferred_body_pose_state = true;
    matching_est.has_body_pose_state = true;
    const LegTargets trajectory_equal =
        trajectory_controller.update(matching_est, trajectory_intent, gait, good_support_safety);
    const LegTargets trajectory_baseline =
        trajectory_controller.update(no_pose_est, trajectory_intent, gait, good_support_safety);
    if (!expect(totalLegPositionDelta(trajectory_equal, trajectory_baseline) < 1e-6,
                "zero level-hold error should add no extra correction")) {
        return EXIT_FAILURE;
    }

    RobotState roll_high_est = matching_est;
    roll_high_est.body_pose_state.orientation_rad = Vec3{
        trajectory_intent.body_pose_setpoint.orientation_rad.x + 0.10,
        trajectory_intent.body_pose_setpoint.orientation_rad.y,
        0.0};
    const LegTargets trajectory_roll_high =
        trajectory_controller.update(roll_high_est, trajectory_intent, gait, good_support_safety);
    MotionIntent roll_negative_reference = trajectory_intent;
    roll_negative_reference.body_pose_setpoint.orientation_rad.x -= 0.04;
    MotionIntent roll_positive_reference = trajectory_intent;
    roll_positive_reference.body_pose_setpoint.orientation_rad.x += 0.04;
    const LegTargets roll_negative_targets =
        trajectory_controller.update(no_pose_est, roll_negative_reference, gait, good_support_safety);
    const LegTargets roll_positive_targets =
        trajectory_controller.update(no_pose_est, roll_positive_reference, gait, good_support_safety);
    if (!expect(totalLegPositionDelta(trajectory_roll_high, roll_negative_targets) <
                    totalLegPositionDelta(trajectory_roll_high, roll_positive_targets),
                "roll correction direction should oppose roll error sign")) {
        return EXIT_FAILURE;
    }

    RobotState pitch_low_est = matching_est;
    pitch_low_est.body_pose_state.orientation_rad = Vec3{
        trajectory_intent.body_pose_setpoint.orientation_rad.x,
        trajectory_intent.body_pose_setpoint.orientation_rad.y - 0.10,
        0.0};
    const LegTargets trajectory_pitch_low =
        trajectory_controller.update(pitch_low_est, trajectory_intent, gait, good_support_safety);
    MotionIntent pitch_positive_reference = trajectory_intent;
    pitch_positive_reference.body_pose_setpoint.orientation_rad.y += 0.04;
    MotionIntent pitch_negative_reference = trajectory_intent;
    pitch_negative_reference.body_pose_setpoint.orientation_rad.y -= 0.04;
    const LegTargets pitch_positive_targets =
        trajectory_controller.update(no_pose_est, pitch_positive_reference, gait, good_support_safety);
    const LegTargets pitch_negative_targets =
        trajectory_controller.update(no_pose_est, pitch_negative_reference, gait, good_support_safety);
    if (!expect(totalLegPositionDelta(trajectory_pitch_low, pitch_positive_targets) <
                    totalLegPositionDelta(trajectory_pitch_low, pitch_negative_targets),
                "pitch correction direction should oppose pitch error sign")) {
        return EXIT_FAILURE;
    }

    LegTargets trajectory_return =
        trajectory_controller.update(matching_est, trajectory_intent, gait, good_support_safety);
    const double initial_return_delta = totalLegPositionDelta(trajectory_return, trajectory_baseline);
    double return_delta = initial_return_delta;
    for (int i = 0; i < 30; ++i) {
        trajectory_return = trajectory_controller.update(matching_est, trajectory_intent, gait, good_support_safety);
        return_delta = totalLegPositionDelta(trajectory_return, trajectory_baseline);
    }
    if (!expect(return_delta < initial_return_delta * 0.3,
                "setpoint trajectory should return near commanded path after transient error")) {
        return EXIT_FAILURE;
    }

    MotionIntent trajectory_next_intent = trajectory_intent;
    trajectory_next_intent.body_pose_setpoint.orientation_rad = Vec3{0.11, -0.02, 0.0};
    BodyController trajectory_update_controller{};
    RobotState next_matching_est{};
    next_matching_est.body_pose_state.orientation_rad = trajectory_next_intent.body_pose_setpoint.orientation_rad;
    next_matching_est.has_inferred_body_pose_state = true;
    next_matching_est.has_body_pose_state = true;
    const LegTargets next_trajectory_with_hold =
        trajectory_update_controller.update(next_matching_est, trajectory_next_intent, gait, good_support_safety);
    RobotState no_pose_next_est = no_pose_est;
    no_pose_next_est.body_pose_state.orientation_rad = trajectory_next_intent.body_pose_setpoint.orientation_rad;
    const LegTargets next_trajectory_baseline =
        trajectory_update_controller.update(no_pose_next_est, trajectory_next_intent, gait, good_support_safety);
    if (!expect(totalLegPositionDelta(next_trajectory_with_hold, next_trajectory_baseline) < 1e-6,
                "setpoint trajectory updates should be preserved under zero-error level hold")) {
        return EXIT_FAILURE;
    }

    RobotState tilted_without_pose = tilted_est;
    tilted_without_pose.has_inferred_body_pose_state = false;
    tilted_without_pose.has_measured_body_pose_state = false;
    tilted_without_pose.has_body_pose_state = false;
    const LegTargets disabled_level_hold_targets =
        controller.update(tilted_without_pose, level_hold_intent, gait, good_support_safety);
    total_delta = totalLegPositionDelta(disabled_level_hold_targets, level_targets);
    if (!expect(total_delta < 1e-6,
                "level hold should be disabled when body pose state is unavailable")) {
        return EXIT_FAILURE;
    }

    MotionIntent noisy_level_hold_intent = level_hold_intent;
    noisy_level_hold_intent.body_pose_setpoint.orientation_rad = Vec3{0.0, 0.0, 0.0};
    noisy_level_hold_intent.body_pose_setpoint.body_trans_mps = Vec3{0.0, 0.0, 0.0};
    noisy_level_hold_intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.0};

    BodyController noisy_controller{};
    SafetyState reliable_support = safety;
    reliable_support.support_contact_count = 6;
    reliable_support.stability_margin_m = 0.05;

    double filtered_roll = 0.0;
    double filtered_pitch = 0.0;
    bool filter_initialized = false;
    double previous_roll_cmd = 0.0;
    double previous_pitch_cmd = 0.0;
    constexpr double kNoiseAmplitudeRad = 0.20;
    constexpr double kVelocityBoundMps = 1e-6;
    constexpr double kSignFlipAmplificationToleranceRad = 1e-3;
    const double expected_flip_command_limit_rad =
        std::min(kLevelHoldMaxCorrectionRad, kLevelHoldRollPitchKp * kNoiseAmplitudeRad);
    constexpr int kNoisyUpdates = 12;

    for (int step = 0; step < kNoisyUpdates; ++step) {
        const double sign = (step % 2 == 0) ? 1.0 : -1.0;
        const double roll_noise = sign * kNoiseAmplitudeRad;
        const double pitch_noise = -sign * kNoiseAmplitudeRad;

        RobotState noisy_est{};
        noisy_est.body_pose_state.orientation_rad = Vec3{roll_noise, pitch_noise, 0.0};
        noisy_est.has_inferred_body_pose_state = true;
        noisy_est.has_body_pose_state = true;

        if (!filter_initialized) {
            filtered_roll = roll_noise;
            filtered_pitch = pitch_noise;
            filter_initialized = true;
        } else {
            filtered_roll += kLevelHoldEstimateLpfAlpha * (roll_noise - filtered_roll);
            filtered_pitch += kLevelHoldEstimateLpfAlpha * (pitch_noise - filtered_pitch);
        }

        const double roll_correction = std::clamp(
            kLevelHoldRollPitchKp * (0.0 - filtered_roll),
            -kLevelHoldMaxCorrectionRad,
            kLevelHoldMaxCorrectionRad);
        const double pitch_correction = std::clamp(
            kLevelHoldRollPitchKp * (0.0 - filtered_pitch),
            -kLevelHoldMaxCorrectionRad,
            kLevelHoldMaxCorrectionRad);
        const double roll_cmd = std::clamp(roll_correction, -kLevelHoldMaxCommandRad, kLevelHoldMaxCommandRad);
        const double pitch_cmd = std::clamp(pitch_correction, -kLevelHoldMaxCommandRad, kLevelHoldMaxCommandRad);

        if (!expect(std::abs(roll_correction) <= kLevelHoldMaxCorrectionRad + 1e-9 &&
                        std::abs(pitch_correction) <= kLevelHoldMaxCorrectionRad + 1e-9,
                    "level hold correction should remain within clamp limits under alternating estimate noise")) {
            return EXIT_FAILURE;
        }
        if (!expect(std::abs(roll_cmd) <= kLevelHoldMaxCommandRad + 1e-9 &&
                        std::abs(pitch_cmd) <= kLevelHoldMaxCommandRad + 1e-9,
                    "level hold commanded roll/pitch should remain within command clamp limits")) {
            return EXIT_FAILURE;
        }

        const LegTargets noisy_targets = noisy_controller.update(
            noisy_est, noisy_level_hold_intent, gait, reliable_support);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!expect(legVelocityMagnitude(noisy_targets.feet[leg]) <= kVelocityBoundMps,
                        "alternating level-hold estimate noise should keep foot target velocity bounded")) {
                return EXIT_FAILURE;
            }
        }

        if (step > 0) {
            const bool roll_sign_flip = (roll_cmd * previous_roll_cmd) < 0.0;
            const bool pitch_sign_flip = (pitch_cmd * previous_pitch_cmd) < 0.0;
            if (roll_sign_flip &&
                !expect(std::abs(roll_cmd) <= expected_flip_command_limit_rad + kSignFlipAmplificationToleranceRad,
                        "roll command sign flips should not amplify beyond tolerance")) {
                return EXIT_FAILURE;
            }
            if (pitch_sign_flip &&
                !expect(std::abs(pitch_cmd) <= expected_flip_command_limit_rad + kSignFlipAmplificationToleranceRad,
                        "pitch command sign flips should not amplify beyond tolerance")) {
                return EXIT_FAILURE;
            }
        }

        previous_roll_cmd = roll_cmd;
        previous_pitch_cmd = pitch_cmd;
    }

    double max_velocity_spike = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double poor_mag = legVelocityMagnitude(poor_support_targets.feet[leg]);
        const double good_mag = legVelocityMagnitude(corrected_targets.feet[leg]);
        max_velocity_spike = std::max(max_velocity_spike, std::abs(good_mag - poor_mag));
    }
    if (!expect(max_velocity_spike < 0.5,
                "switching confidence-aware level hold should not create large velocity spikes")) {
        return EXIT_FAILURE;
    }

    MotionIntent non_finite_intent{};
    non_finite_intent.requested_mode = RobotMode::WALK;
    non_finite_intent.body_pose_setpoint.body_trans_m = Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    non_finite_intent.body_pose_setpoint.body_trans_mps = Vec3{
        std::numeric_limits<double>::infinity(), 0.0, std::numeric_limits<double>::quiet_NaN()};
    non_finite_intent.body_pose_setpoint.angular_velocity_radps = Vec3{
        0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
    non_finite_intent.body_pose_setpoint.orientation_rad = Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};

    RobotState non_finite_state = est;
    non_finite_state.has_measured_body_pose_state = true;
    non_finite_state.has_body_pose_state = true;
    non_finite_state.body_pose_state.orientation_rad = Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        0.0};

    const LegTargets safeguarded_targets =
        controller.update(non_finite_state, non_finite_intent, gait, safety);
    if (!expect(allFiniteTargets(safeguarded_targets),
                "body controller should clamp or reject non-finite state/intent fields to finite targets")) {
        return EXIT_FAILURE;
    }

    MotionIntent wrap_heading_intent{};
    wrap_heading_intent.requested_mode = RobotMode::WALK;
    wrap_heading_intent.speed_mps = LinearRateMps{0.2};
    wrap_heading_intent.body_pose_setpoint.body_trans_m.z = 0.20;

    GaitState wrap_heading_gait{};
    wrap_heading_gait.in_stance.fill(true);
    wrap_heading_gait.phase.fill(0.25);
    wrap_heading_gait.stride_phase_rate_hz = FrequencyHz{1.0};

    constexpr std::array<double, 4> kBoundaryHeadings{
        kPi - 1e-4,
        -kPi + 1e-4,
        kPi - 2e-4,
        -kPi + 2e-4,
    };

    std::array<Vec3, kBoundaryHeadings.size()> boundary_step_velocities{};
    for (std::size_t i = 0; i < kBoundaryHeadings.size(); ++i) {
        wrap_heading_intent.heading_rad = AngleRad{kBoundaryHeadings[i]};
        const LegTargets boundary_targets =
            controller.update(est, wrap_heading_intent, wrap_heading_gait, safety);
        boundary_step_velocities[i] = boundary_targets.feet[0].vel_body_mps;
    }

    for (std::size_t i = 1; i < boundary_step_velocities.size(); ++i) {
        const Vec3& previous = boundary_step_velocities[i - 1];
        const Vec3& current = boundary_step_velocities[i];
        const double planar_dot = previous.x * current.x + previous.y * current.y;
        const double previous_norm = std::sqrt(previous.x * previous.x + previous.y * previous.y);
        const double current_norm = std::sqrt(current.x * current.x + current.y * current.y);
        const double continuity =
            planar_dot / std::max(previous_norm * current_norm, 1e-9);

        if (!expect(continuity > 0.995,
                    "heading wrap across -pi/+pi should keep consecutive step directions continuous")) {
            return EXIT_FAILURE;
        }

        if (!expect(planar_dot > 0.0,
                    "heading wrap should not inject opposite-direction stride artifacts")) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
