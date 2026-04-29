#include "foot_planners.hpp"

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

bool nearlyEqual(double lhs, double rhs, double eps = 1e-9) {
    return std::abs(lhs - rhs) <= eps;
}

} // namespace

int main() {
    const BodyTwist cmd_twist{
        Vec3{0.08, 0.015, 0.01},
        Vec3{0.02, -0.03, 0.25},
    };

    RobotState est{};
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_mps = VelocityMps3{0.08, 0.005, -0.02};
    est.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.04, -0.01, 0.3};

    const BodyVelocityCommand blended = bodyVelocityForFootPlanning(est, cmd_twist, 0.35);
    const double expected_wz = cmd_twist.angular_radps.z * 0.65 + 0.3 * 0.35;
    if (!expect(nearlyEqual(blended.linear_mps.x, cmd_twist.linear_mps.x) &&
                    nearlyEqual(blended.linear_mps.y, cmd_twist.linear_mps.y) &&
                    nearlyEqual(blended.linear_mps.z, cmd_twist.linear_mps.z) &&
                    nearlyEqual(blended.angular_radps.z, expected_wz),
                "foot planning should keep linear motion intent-driven while blending estimator yaw")) {
        return EXIT_FAILURE;
    }

    const BodyVelocityCommand estimator_only = bodyVelocityForFootPlanning(est, cmd_twist, 1.0);
    if (!expect(nearlyEqual(estimator_only.linear_mps.x, cmd_twist.linear_mps.x) &&
                    nearlyEqual(estimator_only.linear_mps.y, cmd_twist.linear_mps.y) &&
                    nearlyEqual(estimator_only.linear_mps.z, cmd_twist.linear_mps.z),
                "full blend should not feed estimator linear velocity back into the foot planner")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqual(estimator_only.angular_radps.x, cmd_twist.angular_radps.x) &&
                    nearlyEqual(estimator_only.angular_radps.y, cmd_twist.angular_radps.y) &&
                    nearlyEqual(estimator_only.angular_radps.z, 0.3),
                "full blend should preserve commanded roll/pitch while blending estimator yaw")) {
        return EXIT_FAILURE;
    }

    RobotState missing_twist = est;
    missing_twist.has_body_twist_state = false;
    const BodyVelocityCommand missing_result = bodyVelocityForFootPlanning(missing_twist, cmd_twist, 0.35);
    if (!expect(nearlyEqual(missing_result.linear_mps.x, cmd_twist.linear_mps.x) &&
                    nearlyEqual(missing_result.linear_mps.y, cmd_twist.linear_mps.y) &&
                    nearlyEqual(missing_result.linear_mps.z, cmd_twist.linear_mps.z) &&
                    nearlyEqual(missing_result.angular_radps.z, cmd_twist.angular_radps.z),
                "missing body twist state should fall back to intent-only command")) {
        return EXIT_FAILURE;
    }

    RobotState invalid_est = est;
    invalid_est.valid = false;
    const BodyVelocityCommand invalid_result = bodyVelocityForFootPlanning(invalid_est, cmd_twist, 1.0);
    if (!expect(nearlyEqual(invalid_result.linear_mps.x, cmd_twist.linear_mps.x) &&
                    nearlyEqual(invalid_result.angular_radps.x, cmd_twist.angular_radps.x),
                "invalid estimator state should also fall back to intent-only command")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
