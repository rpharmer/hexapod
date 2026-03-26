#include "body_controller.hpp"

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

} // namespace

int main() {
    BodyController controller{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent stand_intent{};
    stand_intent.requested_mode = RobotMode::STAND;
    stand_intent.twist.body_trans_mps = Vec3{0.10, -0.20, 0.05};
    stand_intent.twist.twist_vel_radps = Vec3{0.0, 0.0, 0.3};
    stand_intent.twist.body_trans_m.z = 0.20;

    GaitState gait{};
    const LegTargets stand_targets = controller.update(est, stand_intent, gait, safety);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 expected = (Vec3{-stand_intent.twist.body_trans_mps.x,
                                    -stand_intent.twist.body_trans_mps.y,
                                    -stand_intent.twist.body_trans_mps.z}) +
                              cross(stand_intent.twist.twist_vel_radps, stand_targets.feet[leg].pos_body_m);
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
    walk_intent.twist.body_trans_m.z = 0.20;

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

    MotionIntent offset_intent{};
    offset_intent.requested_mode = RobotMode::STAND;
    offset_intent.twist.body_trans_m = Vec3{0.25, 0.0, 0.20};

    const LegTargets offset_targets = controller.update(est, offset_intent, gait, safety);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = defaultHexapodGeometry().legGeometry[leg];
        const Vec3 relative = offset_targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
        const Vec3 foot_leg = Mat3::rotZ(-leg_geo.mountAngle.value) * relative;
        if (!expect(kinematics::legReachUtilization(foot_leg, leg_geo) <= 1.0 + 1e-6,
                    "body controller should clamp target positions to leg reach envelope")) {
            return EXIT_FAILURE;
        }
    }

    MotionIntent turn_intent{};
    turn_intent.requested_mode = RobotMode::WALK;
    turn_intent.twist.twist_vel_radps.z = 0.8;
    turn_intent.twist.body_trans_m.z = 0.20;

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
    neutral_turn_intent.twist.twist_vel_radps.z = 0.0;
    const LegTargets neutral_turn_targets = controller.update(est, neutral_turn_intent, turn_gait, turn_policy, safety);
    const LegTargets turn_targets = controller.update(est, turn_intent, turn_gait, turn_policy, safety);

    const HexapodGeometry geometry = defaultHexapodGeometry();
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
    swing_intent.twist.body_trans_m.z = 0.20;
    swing_intent.twist.body_trans_mps = Vec3{0.10, 0.0, 0.0};

    GaitState swing_gait{};
    swing_gait.in_stance.fill(false);
    swing_gait.phase.fill(0.75);
    swing_gait.stride_phase_rate_hz = FrequencyHz{1.0};

    MotionIntent swing_static_intent = swing_intent;
    swing_static_intent.twist.body_trans_mps = Vec3{0.0, 0.0, 0.0};

    const LegTargets swing_static_targets = controller.update(est, swing_static_intent, swing_gait, safety);
    const LegTargets swing_targets = controller.update(est, swing_intent, swing_gait, safety);
    if (!expect(swing_targets.feet[0].pos_body_m.x < swing_static_targets.feet[0].pos_body_m.x,
                "swing touchdown prediction should account for twist-field stance velocity")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
