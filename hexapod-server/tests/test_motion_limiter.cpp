#include "motion_limiter.hpp"

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
    MotionLimiterConfig cfg{};
    cfg.enabled = true;
    cfg.linear_accel_max_mps2 = Vec3{1.0, 2.0, 3.0};
    cfg.angular_accel_max_radps2 = Vec3{4.0, 5.0, 6.0};

    MotionLimiter limiter{cfg};

    MotionIntent first{};
    first.requested_mode = RobotMode::WALK;
    first.sample_id = 7;
    first.body_pose_setpoint.body_trans_mps = Vec3{0.0, 0.0, 0.0};
    first.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.0};

    const TimePointUs t0{1'000'000};
    const MotionIntent first_limited = limiter.limit(first, t0);
    if (!expect(first_limited.sample_id == first.sample_id,
                "limiter should preserve untouched intent semantics")) {
        return EXIT_FAILURE;
    }

    MotionIntent second = first;
    second.body_pose_setpoint.body_trans_mps = Vec3{5.0, -5.0, 5.0};
    second.body_pose_setpoint.angular_velocity_radps = Vec3{-5.0, 5.0, -5.0};

    const TimePointUs t1{1'100'000}; // dt = 0.1 s
    const MotionIntent second_limited = limiter.limit(second, t1);

    if (!expect(nearlyEqual(second_limited.body_pose_setpoint.body_trans_mps.x, 0.1) &&
                    nearlyEqual(second_limited.body_pose_setpoint.body_trans_mps.y, -0.2) &&
                    nearlyEqual(second_limited.body_pose_setpoint.body_trans_mps.z, 0.3),
                "linear velocity deltas should be clamped per-axis by a_max * dt")) {
        return EXIT_FAILURE;
    }

    if (!expect(nearlyEqual(second_limited.body_pose_setpoint.angular_velocity_radps.x, -0.4) &&
                    nearlyEqual(second_limited.body_pose_setpoint.angular_velocity_radps.y, 0.5) &&
                    nearlyEqual(second_limited.body_pose_setpoint.angular_velocity_radps.z, -0.6),
                "angular velocity deltas should be clamped per-axis by a_max * dt")) {
        return EXIT_FAILURE;
    }

    MotionLimiter disabled{MotionLimiterConfig{.enabled = false}};
    const MotionIntent disabled_output = disabled.limit(second, t1);
    if (!expect(nearlyEqual(disabled_output.body_pose_setpoint.body_trans_mps.x, second.body_pose_setpoint.body_trans_mps.x) &&
                    nearlyEqual(disabled_output.body_pose_setpoint.angular_velocity_radps.z, second.body_pose_setpoint.angular_velocity_radps.z),
                "disabled limiter should pass velocities through")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
