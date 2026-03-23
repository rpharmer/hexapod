#include "body_controller.hpp"

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

} // namespace

int main() {
    BodyController controller{};
    EstimatedState est{};
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

    if (!expect(nearlyEqual(walk_targets.feet[0].vel_body_mps.x, -0.12, 1e-6),
                "walking stance velocity should include analytic step derivative")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
