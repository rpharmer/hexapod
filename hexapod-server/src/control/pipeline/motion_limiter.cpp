#include "motion_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr uint8_t kLimiterPhaseTracking = 0;
constexpr uint8_t kLimiterPhaseBodyLeadsOnStart = 1;
constexpr uint8_t kLimiterPhaseLegsLeadOnStop = 2;
constexpr uint8_t kConstraintReasonNone = 0;
constexpr uint8_t kConstraintReasonTransition = 1;
constexpr uint8_t kConstraintReasonSlewRate = 2;

double safePositive(double value, double fallback) {
    if (!std::isfinite(value) || value <= 0.0) {
        return fallback;
    }
    return value;
}

double clampDelta(double previous, double requested, double max_step) {
    const double delta = requested - previous;
    return previous + std::clamp(delta, -max_step, max_step);
}

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

Vec3 sanitizeVec3(const Vec3& requested, const Vec3& fallback, bool& modified) {
    const Vec3 safe{
        finiteOr(requested.x, fallback.x),
        finiteOr(requested.y, fallback.y),
        finiteOr(requested.z, fallback.z)};
    if (safe.x != requested.x || safe.y != requested.y || safe.z != requested.z) {
        modified = true;
    }
    return safe;
}

} // namespace

MotionLimiter::MotionLimiter(control_config::MotionLimiterConfig config)
    : config_(config) {}

MotionLimiterOutput MotionLimiter::update(const RobotState& estimated,
                                          const MotionIntent& intent,
                                          const RuntimeGaitPolicy& gait_policy,
                                          const SafetyState& safety_state,
                                          const DurationSec& loop_dt) {
    MotionLimiterOutput output{};
    output.limited_intent = intent;
    output.adapted_gait_policy = gait_policy;
    output.diagnostics.enabled = true;
    output.diagnostics.loop_dt = loop_dt;
    output.diagnostics.constraint_reason = kConstraintReasonNone;
    (void)estimated;

    const MotionIntent& fallback_intent = has_previous_limited_intent_ ? previous_limited_intent_ : MotionIntent{};
    output.limited_intent.body_pose_setpoint.body_trans_m = sanitizeVec3(
        output.limited_intent.body_pose_setpoint.body_trans_m,
        fallback_intent.body_pose_setpoint.body_trans_m,
        output.diagnostics.intent_modified);
    output.limited_intent.body_pose_setpoint.body_trans_mps = sanitizeVec3(
        output.limited_intent.body_pose_setpoint.body_trans_mps,
        fallback_intent.body_pose_setpoint.body_trans_mps,
        output.diagnostics.intent_modified);
    output.limited_intent.body_pose_setpoint.angular_velocity_radps = sanitizeVec3(
        output.limited_intent.body_pose_setpoint.angular_velocity_radps,
        fallback_intent.body_pose_setpoint.angular_velocity_radps,
        output.diagnostics.intent_modified);
    output.limited_intent.body_pose_setpoint.orientation_rad = sanitizeVec3(
        output.limited_intent.body_pose_setpoint.orientation_rad,
        fallback_intent.body_pose_setpoint.orientation_rad,
        output.diagnostics.intent_modified);

    const double dt_s = std::max(loop_dt.value, 0.0);
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety_state.inhibit_motion &&
        !safety_state.torque_cut;
    if (!previous_walking_ && walking) {
        startup_phase_elapsed_s_ = 0.0;
        shutdown_phase_elapsed_s_ = 0.0;
        output.diagnostics.phase = kLimiterPhaseBodyLeadsOnStart;
        output.diagnostics.constraint_reason = kConstraintReasonTransition;
    } else if (previous_walking_ && !walking) {
        startup_phase_elapsed_s_ = 0.0;
        shutdown_phase_elapsed_s_ = 0.0;
        output.diagnostics.phase = kLimiterPhaseLegsLeadOnStop;
        output.diagnostics.constraint_reason = kConstraintReasonTransition;
    } else if (walking && startup_phase_elapsed_s_ * 1000.0 < config_.startup_phase_threshold.count()) {
        startup_phase_elapsed_s_ += dt_s;
        output.diagnostics.phase = kLimiterPhaseBodyLeadsOnStart;
        output.diagnostics.constraint_reason = kConstraintReasonTransition;
    } else if (!walking && shutdown_phase_elapsed_s_ * 1000.0 < config_.shutdown_phase_threshold.count()) {
        shutdown_phase_elapsed_s_ += dt_s;
        output.diagnostics.phase = kLimiterPhaseLegsLeadOnStop;
        output.diagnostics.constraint_reason = kConstraintReasonTransition;
    } else {
        output.diagnostics.phase = kLimiterPhaseTracking;
    }
    previous_walking_ = walking;

    if (output.diagnostics.phase == kLimiterPhaseBodyLeadsOnStart) {
        output.adapted_gait_policy.cadence_hz = FrequencyHz{0.0};
        output.adapted_gait_policy.suppression.suppress_stride_progression = true;
        output.diagnostics.gait_policy_modified = true;
    } else if (output.diagnostics.phase == kLimiterPhaseLegsLeadOnStop) {
        output.adapted_gait_policy.cadence_hz = FrequencyHz{0.0};
        output.adapted_gait_policy.suppression.suppress_stride_progression = true;
        output.adapted_gait_policy.suppression.suppress_turning = true;
        output.diagnostics.gait_policy_modified = true;

        if (has_previous_limited_intent_) {
            output.limited_intent.body_pose_setpoint.body_trans_m =
                previous_limited_intent_.body_pose_setpoint.body_trans_m;
            output.limited_intent.body_pose_setpoint.orientation_rad =
                previous_limited_intent_.body_pose_setpoint.orientation_rad;
            output.limited_intent.body_pose_setpoint.body_trans_mps = Vec3{};
            output.limited_intent.body_pose_setpoint.angular_velocity_radps = Vec3{};
            output.diagnostics.intent_modified = true;
        }
    }

    if (has_previous_limited_intent_ && dt_s > 0.0 &&
        output.diagnostics.phase != kLimiterPhaseLegsLeadOnStop) {
        const double linear_limit_mps = safePositive(config_.foot_velocity_limit_mps, 1e-6);
        const double max_linear_step = linear_limit_mps * dt_s;
        const Vec3 previous_body = previous_limited_intent_.body_pose_setpoint.body_trans_m;
        const Vec3 requested_body = intent.body_pose_setpoint.body_trans_m;
        const Vec3 body_delta = requested_body - previous_body;
        const double delta_mag = vecNorm(body_delta);
        if (delta_mag > max_linear_step && delta_mag > 1e-9) {
            const double scale = max_linear_step / delta_mag;
            output.limited_intent.body_pose_setpoint.body_trans_m = previous_body + (body_delta * scale);
            output.diagnostics.intent_modified = true;
            output.diagnostics.hard_clamp_linear = true;
            output.diagnostics.adaptation_scale_linear = scale;
            output.diagnostics.constraint_reason = kConstraintReasonSlewRate;
        }

        const double yaw_rate_limit = safePositive(config_.body_yaw_rate_limit_radps, 1e-6);
        const double max_yaw_step = yaw_rate_limit * dt_s;
        const double previous_yaw = previous_limited_intent_.body_pose_setpoint.orientation_rad.z;
        const double requested_yaw = output.limited_intent.body_pose_setpoint.orientation_rad.z;
        const double limited_yaw = clampDelta(previous_yaw, requested_yaw, max_yaw_step);
        const double yaw_delta = requested_yaw - previous_yaw;
        const double limited_delta = limited_yaw - previous_yaw;
        if (std::abs(yaw_delta) > std::abs(limited_delta) + 1e-12) {
            output.limited_intent.body_pose_setpoint.orientation_rad.z = limited_yaw;
            output.diagnostics.intent_modified = true;
            output.diagnostics.hard_clamp_yaw = true;
            output.diagnostics.adaptation_scale_yaw =
                std::abs(yaw_delta) > 1e-9 ? std::abs(limited_delta) / std::abs(yaw_delta) : 1.0;
            output.diagnostics.constraint_reason = kConstraintReasonSlewRate;
        }
    }

    if (dt_s > 0.0) {
        const Vec3 req_vel = output.limited_intent.body_pose_setpoint.body_trans_mps;
        const Vec3 prev_vel = has_previous_limited_intent_
                                  ? static_cast<Vec3>(previous_limited_intent_.body_pose_setpoint.body_trans_mps)
                                  : req_vel;
        const double max_dxy = safePositive(config_.body_linear_accel_limit_xy_mps2, 1e-6) * dt_s;
        const double max_dz = safePositive(config_.body_linear_accel_limit_z_mps2, 1e-6) * dt_s;
        const double limited_vx = clampDelta(prev_vel.x, req_vel.x, max_dxy);
        const double limited_vy = clampDelta(prev_vel.y, req_vel.y, max_dxy);
        const double limited_vz = clampDelta(prev_vel.z, req_vel.z, max_dz);
        if (std::abs(limited_vx - req_vel.x) > 1e-12 ||
            std::abs(limited_vy - req_vel.y) > 1e-12 ||
            std::abs(limited_vz - req_vel.z) > 1e-12) {
            output.limited_intent.body_pose_setpoint.body_trans_mps = Vec3{limited_vx, limited_vy, limited_vz};
            output.diagnostics.intent_modified = true;
            output.diagnostics.hard_clamp_linear = true;
            output.diagnostics.constraint_reason = kConstraintReasonSlewRate;
        }

        const Vec3 req_ang = output.limited_intent.body_pose_setpoint.angular_velocity_radps;
        const Vec3 prev_ang = has_previous_limited_intent_
                                  ? static_cast<Vec3>(previous_limited_intent_.body_pose_setpoint.angular_velocity_radps)
                                  : req_ang;
        const Vec3 ang_lim = config_.body_angular_accel_limit_radps2;
        const double limited_wx = clampDelta(prev_ang.x, req_ang.x, safePositive(ang_lim.x, 1e-6) * dt_s);
        const double limited_wy = clampDelta(prev_ang.y, req_ang.y, safePositive(ang_lim.y, 1e-6) * dt_s);
        const double limited_wz = clampDelta(prev_ang.z, req_ang.z, safePositive(ang_lim.z, 1e-6) * dt_s);
        if (std::abs(limited_wx - req_ang.x) > 1e-12 ||
            std::abs(limited_wy - req_ang.y) > 1e-12 ||
            std::abs(limited_wz - req_ang.z) > 1e-12) {
            output.limited_intent.body_pose_setpoint.angular_velocity_radps = Vec3{limited_wx, limited_wy, limited_wz};
            output.diagnostics.intent_modified = true;
            output.diagnostics.hard_clamp_yaw = true;
            output.diagnostics.constraint_reason = kConstraintReasonSlewRate;
        }
    }

    previous_limited_intent_ = output.limited_intent;
    has_previous_limited_intent_ = true;
    return output;
}
