#include "locomotion_command.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

BodyTwist rawLocomotionTwistFromIntent(const MotionIntent& intent, const PlanarMotionCommand& planar) {
    BodyTwist w{};
    w.linear_mps = Vec3{planar.vx_mps + intent.twist.body_trans_mps.x,
                        planar.vy_mps + intent.twist.body_trans_mps.y,
                        intent.twist.body_trans_mps.z};
    w.angular_radps = Vec3{intent.twist.twist_vel_radps.x,
                           intent.twist.twist_vel_radps.y,
                           intent.twist.twist_vel_radps.z + planar.yaw_rate_radps};
    return w;
}

BodyTwist clampLocomotionTwist(const BodyTwist& in, const control_config::LocomotionCommandConfig& cfg) {
    BodyTwist o{};
    o.linear_mps.x = std::clamp(in.linear_mps.x, -cfg.max_abs_linear_x_mps, cfg.max_abs_linear_x_mps);
    o.linear_mps.y = std::clamp(in.linear_mps.y, -cfg.max_abs_linear_y_mps, cfg.max_abs_linear_y_mps);
    o.linear_mps.z = std::clamp(in.linear_mps.z, -cfg.max_abs_linear_z_mps, cfg.max_abs_linear_z_mps);
    o.angular_radps.x = std::clamp(in.angular_radps.x, -cfg.max_abs_angular_x_radps, cfg.max_abs_angular_x_radps);
    o.angular_radps.y = std::clamp(in.angular_radps.y, -cfg.max_abs_angular_y_radps, cfg.max_abs_angular_y_radps);
    o.angular_radps.z = std::clamp(in.angular_radps.z, -cfg.max_abs_angular_z_radps, cfg.max_abs_angular_z_radps);
    return o;
}

namespace {

double expSmoothAlpha(const double dt_s, const double tau_s) {
    if (tau_s <= 1e-9 || dt_s <= 0.0) {
        return 1.0;
    }
    const double a = 1.0 - std::exp(-dt_s / tau_s);
    return std::clamp(a, 0.0, 1.0);
}

Vec3 lerpVec3(const Vec3& from, const Vec3& to, const double t) {
    const double u = std::clamp(t, 0.0, 1.0);
    return from * (1.0 - u) + to * u;
}

BodyTwist slewBodyTwistToward(const BodyTwist& current,
                              const BodyTwist& target,
                              const double dt_s,
                              const control_config::LocomotionCommandConfig& cfg) {
    if (dt_s <= 0.0) {
        return current;
    }
    BodyTwist out = current;

    double dx = target.linear_mps.x - current.linear_mps.x;
    double dy = target.linear_mps.y - current.linear_mps.y;
    const double norm_xy = std::hypot(dx, dy);
    if (cfg.max_linear_accel_xy_mps2 > 0.0) {
        const double max_step = cfg.max_linear_accel_xy_mps2 * dt_s;
        if (norm_xy > max_step && norm_xy > 1e-18) {
            const double s = max_step / norm_xy;
            dx *= s;
            dy *= s;
        }
    }
    out.linear_mps.x = current.linear_mps.x + dx;
    out.linear_mps.y = current.linear_mps.y + dy;

    double dz = target.linear_mps.z - current.linear_mps.z;
    if (cfg.max_linear_accel_z_mps2 > 0.0) {
        const double max_step = cfg.max_linear_accel_z_mps2 * dt_s;
        dz = std::clamp(dz, -max_step, max_step);
    }
    out.linear_mps.z = current.linear_mps.z + dz;

    if (cfg.max_angular_accel_radps2 > 0.0) {
        const double max_step = cfg.max_angular_accel_radps2 * dt_s;
        double dwx = target.angular_radps.x - current.angular_radps.x;
        double dwy = target.angular_radps.y - current.angular_radps.y;
        double dwz = target.angular_radps.z - current.angular_radps.z;
        dwx = std::clamp(dwx, -max_step, max_step);
        dwy = std::clamp(dwy, -max_step, max_step);
        dwz = std::clamp(dwz, -max_step, max_step);
        out.angular_radps.x = current.angular_radps.x + dwx;
        out.angular_radps.y = current.angular_radps.y + dwy;
        out.angular_radps.z = current.angular_radps.z + dwz;
    } else {
        out.angular_radps = target.angular_radps;
    }

    return out;
}

} // namespace

LocomotionCommandProcessor::LocomotionCommandProcessor(control_config::LocomotionCommandConfig config)
    : config_(config) {}

void LocomotionCommandProcessor::reset() {
    filtered_ = BodyTwist{};
    last_clock_ = TimePointUs{};
    have_clock_ = false;
    prev_walking_ = false;
}

BodyTwist LocomotionCommandProcessor::update(const MotionIntent& intent,
                                             const PlanarMotionCommand& planar,
                                             const TimePointUs clock_tick_us) {
    const BodyTwist raw = rawLocomotionTwistFromIntent(intent, planar);
    const BodyTwist target = clampLocomotionTwist(raw, config_);

    double dt = config_.nominal_dt_s;
    if (have_clock_ && !clock_tick_us.isZero() && !last_clock_.isZero() && clock_tick_us.value > last_clock_.value) {
        dt = static_cast<double>(clock_tick_us.value - last_clock_.value) * 1e-6;
        dt = std::clamp(dt, 1e-5, 0.25);
    }

    const bool walking = (intent.requested_mode == RobotMode::WALK);

    if (config_.enable_chassis_accel_limit) {
        filtered_ = slewBodyTwistToward(filtered_, target, dt, config_);
        last_clock_ = clock_tick_us;
        have_clock_ = !clock_tick_us.isZero();
        prev_walking_ = walking;
        return filtered_;
    }

    if (!walking) {
        prev_walking_ = false;
        filtered_ = target;
        last_clock_ = clock_tick_us;
        have_clock_ = !clock_tick_us.isZero();
        return filtered_;
    }

    if (!prev_walking_) {
        prev_walking_ = true;
        filtered_ = target;
        last_clock_ = clock_tick_us;
        have_clock_ = !clock_tick_us.isZero();
        return filtered_;
    }

    last_clock_ = clock_tick_us;
    have_clock_ = true;

    if (!config_.enable_first_order_filter || config_.filter_time_constant_s <= 1e-9) {
        filtered_ = target;
        return filtered_;
    }

    const double a = expSmoothAlpha(dt, config_.filter_time_constant_s);
    filtered_.linear_mps = lerpVec3(filtered_.linear_mps, target.linear_mps, a);
    filtered_.angular_radps = lerpVec3(filtered_.angular_radps, target.angular_radps, a);
    return filtered_;
}
