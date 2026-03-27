#include "motion_limiter.hpp"

#include "control_config.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPlanarSpeedEnterMps = 0.04;
constexpr double kPlanarSpeedExitMps = 0.02;
constexpr double kYawRateEnterRadps = 0.20;
constexpr double kYawRateExitRadps = 0.12;

double elapsedMs(const TimePointUs& from, const TimePointUs& to) {
    if (from.isZero() || to.value <= from.value) {
        return 0.0;
    }
    return static_cast<double>(to.value - from.value) * 1e-3;
}

bool locomotionRequestedByCommand(const MotionIntent& raw) {
    const bool walk_mode = raw.requested_mode == RobotMode::WALK;
    const double requested_speed = std::abs(raw.speed_mps.value);
    const double requested_yaw = std::abs(raw.body_pose_setpoint.angular_velocity_radps.z);
    return walk_mode && (requested_speed >= kPlanarSpeedEnterMps ||
                         requested_yaw >= kYawRateEnterRadps);
}

bool locomotionClearedByCommand(const MotionIntent& raw) {
    if (raw.requested_mode != RobotMode::WALK) {
        return true;
    }
    const double requested_speed = std::abs(raw.speed_mps.value);
    const double requested_yaw = std::abs(raw.body_pose_setpoint.angular_velocity_radps.z);
    return requested_speed <= kPlanarSpeedExitMps &&
           requested_yaw <= kYawRateExitRadps;
}

Vec3 planarVelocityFromIntent(const MotionIntent& raw) {
    const double speed = raw.speed_mps.value;
    const double heading = raw.heading_rad.value;
    return Vec3{speed * std::cos(heading), speed * std::sin(heading), 0.0};
}

} // namespace

double MotionLimiter::updateAxis(double target,
                                 double dt_s,
                                 double accel_limit,
                                 double vel_limit,
                                 double* value,
                                 double* velocity) {
    if (dt_s <= 0.0) {
        *value = target;
        *velocity = 0.0;
        return *value;
    }

    const double delta = target - *value;
    const double desired_vel = std::clamp(delta / dt_s, -vel_limit, vel_limit);
    const double accel_step = accel_limit * dt_s;
    const double vel_delta = desired_vel - *velocity;
    *velocity += std::clamp(vel_delta, -accel_step, accel_step);
    *velocity = std::clamp(*velocity, -vel_limit, vel_limit);
    *value += (*velocity * dt_s);

    const bool reached_and_not_moving = std::abs(delta) < 1e-9 && std::abs(*velocity) < 1e-9;
    if (reached_and_not_moving) {
        *value = target;
        *velocity = 0.0;
    }
    return *value;
}

bool MotionLimiter::bodyPoseNear(const BodyPoseKinematics& lhs, const BodyPoseKinematics& rhs) {
    const Vec3 trans_delta = static_cast<Vec3>(lhs.body_trans_m) - static_cast<Vec3>(rhs.body_trans_m);
    const Vec3 orient_delta = static_cast<Vec3>(lhs.orientation_rad) - static_cast<Vec3>(rhs.orientation_rad);
    return vecNorm(trans_delta) <= kBodyTranslationSettleEpsM &&
           vecNorm(orient_delta) <= kBodyOrientationSettleEpsRad;
}

MotionLimiterOutput MotionLimiter::update(const MotionIntent& raw_intent, const SafetyState& safety) {
    MotionLimiterOutput out{};
    out.intent = raw_intent;

    const TimePointUs now = now_us();
    const double dt_s = (!last_update_us_.isZero() && now.value > last_update_us_.value)
                            ? static_cast<double>(now.value - last_update_us_.value) * 1e-6
                            : 0.0;
    last_update_us_ = now;

    const bool safety_blocks_locomotion = safety.inhibit_motion || safety.torque_cut ||
                                          safety.active_fault != FaultCode::NONE;
    const bool motion_requested = !safety_blocks_locomotion && locomotionRequestedByCommand(raw_intent);
    const bool motion_cleared = safety_blocks_locomotion || locomotionClearedByCommand(raw_intent);

    if (motion_requested) {
        if (motion_request_since_us_.isZero()) {
            motion_request_since_us_ = now;
        }
        motion_clear_since_us_ = TimePointUs{};
    } else if (motion_cleared) {
        if (motion_clear_since_us_.isZero()) {
            motion_clear_since_us_ = now;
        }
        motion_request_since_us_ = TimePointUs{};
    }

    const bool request_stable = elapsedMs(motion_request_since_us_, now) >= kCommandHysteresisMs;
    const bool clear_stable = elapsedMs(motion_clear_since_us_, now) >= kCommandHysteresisMs;

    switch (state_) {
        case MotionLimiterState::IDLE:
            if (request_stable) {
                state_ = MotionLimiterState::BODY_PRELOAD;
                state_enter_us_ = now;
            }
            break;
        case MotionLimiterState::BODY_PRELOAD: {
            const bool body_ready = bodyPoseNear(filtered_body_, raw_intent.body_pose_setpoint);
            const bool preload_timed_out = elapsedMs(state_enter_us_, now) >= kBodyPreloadTimeoutMs;
            if (clear_stable) {
                state_ = MotionLimiterState::IDLE;
                state_enter_us_ = now;
            } else if (request_stable && (body_ready || preload_timed_out)) {
                state_ = MotionLimiterState::LOCOMOTING;
                state_enter_us_ = now;
            }
            break;
        }
        case MotionLimiterState::LOCOMOTING:
            if (clear_stable) {
                state_ = MotionLimiterState::BODY_SETTLE;
                state_enter_us_ = now;
                settle_stride_halted_ = false;
                body_settle_pose_ = filtered_body_;
            }
            break;
        case MotionLimiterState::BODY_SETTLE:
            if (request_stable) {
                state_ = MotionLimiterState::LOCOMOTING;
                state_enter_us_ = now;
            }
            break;
    }

    const Vec3 desired_planar_vel =
        (state_ == MotionLimiterState::LOCOMOTING) ? planarVelocityFromIntent(raw_intent) : Vec3{};
    const double desired_yaw_rate =
        (state_ == MotionLimiterState::LOCOMOTING) ? raw_intent.body_pose_setpoint.angular_velocity_radps.z : 0.0;

    updateAxis(desired_planar_vel.x,
               dt_s,
               kPlanarVelocityAccelMps2,
               control_config::kDefaultGaitNominalMaxSpeedMps,
               &filtered_planar_velocity_mps_.x,
               &filtered_planar_velocity_dot_mps2_.x);
    updateAxis(desired_planar_vel.y,
               dt_s,
               kPlanarVelocityAccelMps2,
               control_config::kDefaultGaitNominalMaxSpeedMps,
               &filtered_planar_velocity_mps_.y,
               &filtered_planar_velocity_dot_mps2_.y);
    updateAxis(desired_yaw_rate,
               dt_s,
               kYawRateAccelRadps2,
               control_config::kDefaultTurnYawRateEnterRadps,
               &filtered_yaw_rate_radps_,
               &filtered_yaw_rate_dot_radps2_);

    BodyPoseKinematics desired_body = raw_intent.body_pose_setpoint;
    if (state_ == MotionLimiterState::BODY_SETTLE && !settle_stride_halted_) {
        desired_body = body_settle_pose_;
    }

    updateAxis(desired_body.body_trans_m.x,
               dt_s,
               kBodyTranslationAccelMps2,
               kBodyTranslationMaxVelMps,
               &filtered_body_.body_trans_m.x,
               &filtered_body_trans_vel_mps_.x);
    updateAxis(desired_body.body_trans_m.y,
               dt_s,
               kBodyTranslationAccelMps2,
               kBodyTranslationMaxVelMps,
               &filtered_body_.body_trans_m.y,
               &filtered_body_trans_vel_mps_.y);
    updateAxis(desired_body.body_trans_m.z,
               dt_s,
               kBodyTranslationAccelMps2,
               kBodyTranslationMaxVelMps,
               &filtered_body_.body_trans_m.z,
               &filtered_body_trans_vel_mps_.z);

    updateAxis(desired_body.orientation_rad.x,
               dt_s,
               kBodyOrientationAccelRadps2,
               kBodyOrientationMaxVelRadps,
               &filtered_body_.orientation_rad.x,
               &filtered_body_orientation_vel_radps_.x);
    updateAxis(desired_body.orientation_rad.y,
               dt_s,
               kBodyOrientationAccelRadps2,
               kBodyOrientationMaxVelRadps,
               &filtered_body_.orientation_rad.y,
               &filtered_body_orientation_vel_radps_.y);
    updateAxis(desired_body.orientation_rad.z,
               dt_s,
               kBodyOrientationAccelRadps2,
               kBodyOrientationMaxVelRadps,
               &filtered_body_.orientation_rad.z,
               &filtered_body_orientation_vel_radps_.z);

    out.intent.body_pose_setpoint.body_trans_m = filtered_body_.body_trans_m;
    out.intent.body_pose_setpoint.body_trans_mps = filtered_planar_velocity_mps_;
    out.intent.body_pose_setpoint.orientation_rad = filtered_body_.orientation_rad;

    const double planar_speed =
        std::hypot(filtered_planar_velocity_mps_.x, filtered_planar_velocity_mps_.y);
    out.intent.speed_mps = LinearRateMps{planar_speed};
    out.intent.heading_rad = AngleRad{(planar_speed > 1e-6)
                                          ? std::atan2(filtered_planar_velocity_mps_.y,
                                                       filtered_planar_velocity_mps_.x)
                                          : raw_intent.heading_rad.value};
    out.intent.body_pose_setpoint.angular_velocity_radps.z = filtered_yaw_rate_radps_;

    switch (state_) {
        case MotionLimiterState::IDLE:
            out.intent.requested_mode =
                raw_intent.requested_mode == RobotMode::FAULT ? RobotMode::FAULT : RobotMode::STAND;
            break;
        case MotionLimiterState::BODY_PRELOAD:
            out.intent.requested_mode = RobotMode::STAND;
            out.intent.speed_mps = LinearRateMps{0.0};
            out.intent.body_pose_setpoint.angular_velocity_radps.z = 0.0;
            break;
        case MotionLimiterState::LOCOMOTING:
            out.intent.requested_mode = RobotMode::WALK;
            break;
        case MotionLimiterState::BODY_SETTLE: {
            const bool zero_stride_cmd =
                out.intent.speed_mps.value <= kPlanarSpeedExitMps &&
                std::abs(out.intent.body_pose_setpoint.angular_velocity_radps.z) <= kYawRateExitRadps;
            const bool settle_hold_elapsed = elapsedMs(state_enter_us_, now) >= kBodySettleHoldMs;
            if (zero_stride_cmd && settle_hold_elapsed) {
                settle_stride_halted_ = true;
            }

            out.intent.requested_mode = settle_stride_halted_ ? RobotMode::STAND : RobotMode::WALK;

            const bool settle_delay_done = settle_hold_elapsed;
            const bool settle_timeout = elapsedMs(state_enter_us_, now) >= kBodySettleTimeoutMs;
            const bool body_done = bodyPoseNear(filtered_body_, raw_intent.body_pose_setpoint);
            if (settle_stride_halted_ && settle_delay_done && (body_done || settle_timeout)) {
                state_ = MotionLimiterState::IDLE;
                state_enter_us_ = now;
                out.intent.requested_mode = RobotMode::STAND;
            }
            break;
        }
    }

    out.state = state_;
    return out;
}
