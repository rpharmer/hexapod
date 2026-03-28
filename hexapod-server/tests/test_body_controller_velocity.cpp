#include "body_controller.hpp"
#include "leg_ik.hpp"

#include <algorithm>
#include <cmath>

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

double wrappedAngleDistance(double a_rad, double b_rad) {
    const double wrapped =
        std::remainder(a_rad - b_rad, 2.0 * kPi);
    return std::abs(wrapped);
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

    double total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = poor_support_targets.feet[leg].pos_body_m - level_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    if (!expect(total_delta < 1e-6,
                "level hold should be disabled with inferred pose but poor support quality")) {
        return EXIT_FAILURE;
    }

    const LegTargets corrected_targets = controller.update(tilted_est, level_hold_intent, gait, good_support_safety);
    total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = corrected_targets.feet[leg].pos_body_m - level_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    if (!expect(total_delta > 1e-4,
                "level hold should engage with inferred pose when support quality is sufficient")) {
        return EXIT_FAILURE;
    }

    RobotState tilted_measured_only = tilted_est;
    tilted_measured_only.has_inferred_body_pose_state = false;
    tilted_measured_only.has_measured_body_pose_state = true;
    tilted_measured_only.has_body_pose_state = true;
    const LegTargets measured_level_hold_targets = controller.update(tilted_measured_only, level_hold_intent, gait, safety);
    total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = measured_level_hold_targets.feet[leg].pos_body_m - level_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
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

    RobotState tilted_without_pose = tilted_est;
    tilted_without_pose.has_inferred_body_pose_state = false;
    tilted_without_pose.has_measured_body_pose_state = false;
    tilted_without_pose.has_body_pose_state = false;
    const LegTargets disabled_level_hold_targets =
        controller.update(tilted_without_pose, level_hold_intent, gait, good_support_safety);
    total_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = disabled_level_hold_targets.feet[leg].pos_body_m - level_targets.feet[leg].pos_body_m;
        total_delta += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    if (!expect(total_delta < 1e-6,
                "level hold should be disabled when body pose state is unavailable")) {
        return EXIT_FAILURE;
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

    return EXIT_SUCCESS;
}
