#include "foot_planners.hpp"

#include "foothold_planner.hpp"
#include "swing_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kVRefMps = 0.18;
constexpr double kWRefRadps = 0.42;
constexpr double kRaibertCaptureGain = 0.55;
constexpr double kAccelCaptureGain = 0.42;
void rotate2d(const double x, const double y, const double c, const double s, double* ox, double* oy) {
    *ox = x * c - y * s;
    *oy = x * s + y * c;
}

// z_rel(tau) = swing_h * 64 * (tau*(1-tau))^3  => dz/dtau = 0 at tau in {0,1} (smooth liftoff/touchdown).
void swingVerticalShape(const double swing_h, const double tau01, double* z_rel, double* dz_dtau) {
    const double t = std::clamp(tau01, 0.0, 1.0);
    const double u = t * (1.0 - t);
    const double u3 = u * u * u;
    constexpr double k_norm = 64.0;
    *z_rel = swing_h * k_norm * u3;
    const double du_dt = 1.0 - 2.0 * t;
    *dz_dtau = swing_h * k_norm * 3.0 * u * u * du_dt;
}

} // namespace

BodyVelocityCommand bodyVelocityForFootPlanning(const RobotState& est,
                                                const BodyTwist& cmd_twist,
                                                const double foot_estimator_blend_01) {
    const double k = std::clamp(foot_estimator_blend_01, 0.0, 1.0);
    const BodyVelocityCommand intent_only = cmd_twist;

    if (!est.valid || !est.has_body_twist_state) {
        return intent_only;
    }

    // The estimator body twist and the command twist are both expressed in the same body frame:
    // +X forward, +Y left, +Z up. Blend them directly in that shared convention.
    // Angular velocity: simVecToServer preserves CCW-positive yaw (Y_sim→Z_svr) and maps
    // roll/pitch consistently, so no extra sign adjustment is needed here either.
    const Vec3 lin_est{est.body_twist_state.body_trans_mps.x,
                       est.body_twist_state.body_trans_mps.y,
                       est.body_twist_state.body_trans_mps.z};
    const Vec3 ang_est{est.body_twist_state.twist_vel_radps.x,
                       est.body_twist_state.twist_vel_radps.y,
                       est.body_twist_state.twist_vel_radps.z};

    BodyVelocityCommand out{};
    out.linear_mps = intent_only.linear_mps * (1.0 - k) + lin_est * k;
    out.angular_radps = intent_only.angular_radps * (1.0 - k) + ang_est * k;
    out.linear_mps.z = intent_only.linear_mps.z;
    return out;
}

Vec3 supportFootVelocityAt(const Vec3& r_b, const BodyVelocityCommand& body) {
    return TwistField::stanceFootVelocity(body, r_b);
}

void planStanceFoot(const StanceFootInputs& in, Vec3& pos_body, Vec3& vel_body) {
    // Integrate v_foot_body = twist-field stance velocity so the contact stays world-fixed (stage 4).
    const double f = std::max(in.f_hz, 1e-6);
    const double phi = clamp01(in.phase);
    pos_body = in.anchor + in.v_foot_body * (phi / f);
    vel_body = in.v_foot_body;
}

void planSwingFoot(const BodyVelocityCommand& body, const SwingFootInputs& in, Vec3& pos_body, Vec3& vel_body) {
    const double tau = clamp01(in.tau01);
    const double swing_span = std::max(in.swing_span, 1e-6);
    const double f_hz = std::max(in.f_hz, 1e-6);
    const double T_swing = swing_span / f_hz;
    const double chain = (1.0 / swing_span) * f_hz;

    const double wz = body.angular_radps.z;
    const double v_planar = std::hypot(body.linear_mps.x, body.linear_mps.y);
    const double w_norm = vecNorm(body.angular_radps);
    const double vel_scale =
        std::clamp(0.40 + 0.92 * (v_planar / kVRefMps) + 0.38 * (w_norm / kWRefRadps), 0.48, 1.55);
    const double step_len = std::max(in.step_length_m * vel_scale, 0.0);
    const double swing_h = std::max(in.swing_height_m * vel_scale, 0.0);

    const double p0x = in.stance_end.x;
    const double p0y = in.stance_end.y;

    double stride_x = 0.0;
    double stride_y = 0.0;
    rotate2d(in.stride_ux * step_len,
             in.stride_uy * step_len,
             std::cos(-wz * T_swing),
             std::sin(-wz * T_swing),
             &stride_x,
             &stride_y);

    const double capture_x = kRaibertCaptureGain * body.linear_mps.x * T_swing;
    const double capture_y = kRaibertCaptureGain * body.linear_mps.y * T_swing;
    const double accel_x = kAccelCaptureGain * in.cmd_accel_body_x_mps2 * T_swing * T_swing;
    const double accel_y = kAccelCaptureGain * in.cmd_accel_body_y_mps2 * T_swing * T_swing;

    const double horizon = T_swing + std::max(0.0, in.stance_lookahead_s);
    Vec3 twist_extra = twistIntegratedFootholdDeltaXY(body, in.stance_end, horizon);
    twist_extra = twist_extra + stabilityFootholdBiasXY(in.static_stability_margin_m, in.stance_end);
    twist_extra = clampFootholdExtraXY(twist_extra, 0.58 * step_len);

    const double p3x = p0x + stride_x + capture_x + accel_x + twist_extra.x;
    const double p3y = p0y + stride_y + capture_y + accel_y + twist_extra.y;

    const double m0x = in.v_liftoff_body.x * T_swing;
    const double m0y = in.v_liftoff_body.y * T_swing;

    const Vec3 v_touch = supportFootVelocityAt(Vec3{p3x, p3y, in.anchor.z}, body);
    double m1x = v_touch.x * T_swing;
    double m1y = v_touch.y * T_swing;
    const double stride_mag = std::hypot(p3x - p0x, p3y - p0y);
    constexpr double kM1VsStride = 2.5;
    const double m1_lim = std::max(0.02, kM1VsStride * stride_mag);
    const double m1_mag = std::hypot(m1x, m1y);
    if (m1_mag > m1_lim && m1_mag > 1e-12) {
        const double s = m1_lim / m1_mag;
        m1x *= s;
        m1y *= s;
    }

    const double time_ease = std::clamp(in.swing_time_ease_01, 0.0, 1.0);

    double sx = 0.0;
    double sy = 0.0;
    double dpx_dtau = 0.0;
    double dpy_dtau = 0.0;
    swing_trajectory::evalSwingPlanarBezier(
        tau, time_ease, p0x, p0y, p3x, p3y, m0x, m0y, m1x, m1y, &sx, &sy, &dpx_dtau, &dpy_dtau);

    const double vx_s = dpx_dtau * chain;
    const double vy_s = dpy_dtau * chain;

    const double s_vert = swing_trajectory::timeWarp(tau, time_ease);
    const double ds_vert_dtau = swing_trajectory::timeWarpDeriv(tau, time_ease);
    double z_rel = 0.0;
    double dz_ds = 0.0;
    swingVerticalShape(swing_h, s_vert, &z_rel, &dz_ds);
    const double dz_dtau = dz_ds * ds_vert_dtau;
    const double vz_s = dz_dtau * chain;

    pos_body = Vec3{sx, sy, in.anchor.z + z_rel};
    vel_body = Vec3{vx_s, vy_s, vz_s};
}
