#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
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

bool standTargetsRemainReachable(const LegTargets& targets) {
    const HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    constexpr double kReachEps = 1e-9;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const Vec3 rel_body = targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
        const Mat3 leg_from_body = Mat3::rotZ(-leg_geo.mountAngle.value);
        const Vec3 rel_leg = leg_from_body * rel_body;
        const double r = std::hypot(rel_leg.x, rel_leg.y);
        const double rho = r - leg_geo.coxaLength.value;
        const double d = std::hypot(rho, rel_leg.z);
        const double max_reach = leg_geo.femurLength.value + leg_geo.tibiaLength.value;
        if (d > max_reach + kReachEps) {
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
    stand_intent.twist.body_trans_mps = Vec3{0.10, -0.20, 0.05};
    stand_intent.twist.twist_vel_radps = Vec3{0.0, 0.0, 0.3};
    stand_intent.twist.body_trans_m.z = 0.12;

    GaitState gait{};
    const BodyTwist stand_twist =
        rawLocomotionTwistFromIntent(stand_intent, planarMotionCommand(stand_intent));
    const LegTargets stand_targets = controller.update(est, stand_intent, gait, safety, stand_twist);

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
    if (!expect(standTargetsRemainReachable(stand_targets),
                "stand mode nominal targets should stay within leg reach")) {
        return EXIT_FAILURE;
    }

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.speed_mps = LinearRateMps{0.2};
    walk_intent.heading_rad = AngleRad{0.0};
    walk_intent.cmd_vx_mps = LinearRateMps{0.2};
    walk_intent.cmd_vy_mps = LinearRateMps{0.0};
    walk_intent.twist.body_trans_m.z = 0.12;

    GaitState walk_gait{};
    walk_gait.phase[0] = 0.25;
    walk_gait.in_stance[0] = true;
    walk_gait.duty_factor = 0.5;
    walk_gait.stride_phase_rate_hz = FrequencyHz{1.0};
    walk_gait.step_length_m = 0.06;
    walk_gait.swing_height_m = 0.03;
    const BodyTwist walk_twist =
        rawLocomotionTwistFromIntent(walk_intent, planarMotionCommand(walk_intent));
    const LegTargets walk_targets = controller.update(est, walk_intent, walk_gait, safety, walk_twist);

    if (!expect(nearlyEqual(walk_targets.feet[0].vel_body_mps.x, -0.2, 2e-3),
                "walking stance foot velocity should match commanded body planar motion")) {
        return EXIT_FAILURE;
    }

    RobotState low_body_est = est;
    low_body_est.has_body_twist_state = true;
    low_body_est.has_fusion_diagnostics = true;
    low_body_est.fusion.model_trust = 1.0;
    low_body_est.body_twist_state.body_trans_m.z = 0.03;
    const LegTargets held_height_targets =
        controller.update(low_body_est, walk_intent, walk_gait, safety, walk_twist);
    if (!expect(held_height_targets.feet[0].pos_body_m.z < walk_targets.feet[0].pos_body_m.z - 0.02,
                "walking height hold should materially lower stance feet when the body sags")) {
        return EXIT_FAILURE;
    }

    MotionIntent idle_intent = stand_intent;
    idle_intent.requested_mode = RobotMode::SAFE_IDLE;
    const LegTargets idle_targets = controller.update(low_body_est, idle_intent, gait, safety, stand_twist);
    if (!expect(idle_targets.feet[0].pos_body_m.z < stand_targets.feet[0].pos_body_m.z - 0.02,
                "idle height hold should also materially lower stance feet when the body sags")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
