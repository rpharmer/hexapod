#include "motion_limiter.hpp"

#include <algorithm>
#include <cmath>

MotionLimiter::MotionLimiter(MotionLimiterConfig config)
    : config_(config) {}

MotionIntent MotionLimiter::limit(const MotionIntent& intent, const TimePointUs& now) {
    MotionIntent limited = intent;

    if (!config_.enabled) {
        has_previous_command_ = false;
        previous_update_timestamp_ = now;
        previous_body_trans_mps_ = intent.body_pose_setpoint.body_trans_mps;
        previous_angular_velocity_radps_ = intent.body_pose_setpoint.angular_velocity_radps;
        return limited;
    }

    const TimePointUs previous_timestamp = previous_update_timestamp_;
    const bool has_positive_dt = has_previous_command_ &&
                                 !previous_timestamp.isZero() &&
                                 now.value > previous_timestamp.value;
    const double dt_s = has_positive_dt
                            ? static_cast<double>(now.value - previous_timestamp.value) / 1'000'000.0
                            : 0.0;

    if (!has_previous_command_ || !has_positive_dt) {
        limited.body_pose_setpoint.body_trans_mps = intent.body_pose_setpoint.body_trans_mps;
        limited.body_pose_setpoint.angular_velocity_radps = intent.body_pose_setpoint.angular_velocity_radps;
    } else {
        limited.body_pose_setpoint.body_trans_mps.x = clampAxisDelta(
            previous_body_trans_mps_.x,
            intent.body_pose_setpoint.body_trans_mps.x,
            config_.linear_accel_max_mps2.x,
            dt_s);
        limited.body_pose_setpoint.body_trans_mps.y = clampAxisDelta(
            previous_body_trans_mps_.y,
            intent.body_pose_setpoint.body_trans_mps.y,
            config_.linear_accel_max_mps2.y,
            dt_s);
        limited.body_pose_setpoint.body_trans_mps.z = clampAxisDelta(
            previous_body_trans_mps_.z,
            intent.body_pose_setpoint.body_trans_mps.z,
            config_.linear_accel_max_mps2.z,
            dt_s);

        limited.body_pose_setpoint.angular_velocity_radps.x = clampAxisDelta(
            previous_angular_velocity_radps_.x,
            intent.body_pose_setpoint.angular_velocity_radps.x,
            config_.angular_accel_max_radps2.x,
            dt_s);
        limited.body_pose_setpoint.angular_velocity_radps.y = clampAxisDelta(
            previous_angular_velocity_radps_.y,
            intent.body_pose_setpoint.angular_velocity_radps.y,
            config_.angular_accel_max_radps2.y,
            dt_s);
        limited.body_pose_setpoint.angular_velocity_radps.z = clampAxisDelta(
            previous_angular_velocity_radps_.z,
            intent.body_pose_setpoint.angular_velocity_radps.z,
            config_.angular_accel_max_radps2.z,
            dt_s);
    }

    previous_body_trans_mps_ = limited.body_pose_setpoint.body_trans_mps;
    previous_angular_velocity_radps_ = limited.body_pose_setpoint.angular_velocity_radps;
    previous_update_timestamp_ = now;
    has_previous_command_ = true;

    return limited;
}

bool MotionLimiter::enabled() const {
    return config_.enabled;
}

double MotionLimiter::clampAxisDelta(double previous, double commanded, double accel_limit, double dt_s) {
    const double bounded_accel = std::max(0.0, accel_limit);
    const double max_delta = bounded_accel * dt_s;
    const double delta = commanded - previous;
    return previous + std::clamp(delta, -max_delta, max_delta);
}
