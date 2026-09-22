#include "foot_planners.hpp"

#include "foothold_planner.hpp"
#include "swing_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kVRefMps = 0.18;
constexpr double kWRefRadps = 0.42;
constexpr double kCaptureLimitScale = 0.35;
constexpr double kCaptureLimitMinM = 0.008;
constexpr double kCaptureLimitMaxM = 0.032;

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
    // +X forward, +Y left, +Z up. Linear estimator feedback is only trusted when fusion/support
    // are credible; yaw blending remains the legacy behaviour for sparse diagnostics.
    const Vec3 ang_est{est.body_twist_state.twist_vel_radps.x,
                       est.body_twist_state.twist_vel_radps.y,
                       est.body_twist_state.twist_vel_radps.z};
    const Vec3 lin_est{est.body_twist_state.body_trans_mps.x,
                       est.body_twist_state.body_trans_mps.y,
                       est.body_twist_state.body_trans_mps.z};

    int support_count = 0;
    for (const bool contact : est.foot_contacts) {
        support_count += contact ? 1 : 0;
    }
    const bool support_sparse = support_count > 0 && support_count < 3;
    const double fusion_trust = est.has_fusion_diagnostics ? est.fusion.model_trust : 1.0;
    const bool trust_good = fusion_trust >= 0.55;
    constexpr bool kEnableLinearEstimatorBlend = false;
    const double linear_k = (kEnableLinearEstimatorBlend && !support_sparse && trust_good) ? k : 0.0;

    BodyVelocityCommand out{};
    out.linear_mps = intent_only.linear_mps;
    out.linear_mps.x = intent_only.linear_mps.x * (1.0 - linear_k) + lin_est.x * linear_k;
    out.linear_mps.y = intent_only.linear_mps.y * (1.0 - linear_k) + lin_est.y * linear_k;
    out.angular_radps = intent_only.angular_radps;
    out.angular_radps.z = intent_only.angular_radps.z * (1.0 - k) + ang_est.z * k;
    out.linear_mps.z = intent_only.linear_mps.z;
    return out;
}

Vec3 supportFootVelocityAt(const Vec3& r_b, const BodyVelocityCommand& body) {
    return TwistField::stanceFootVelocity(body, r_b);
}

SwingFootPlanDecomposition computeSwingFootPlacement(const RobotState& est,
                                                     const BodyTwist& nominal_body,
                                                     const SwingFootInputs& in) {
    SwingFootPlanDecomposition out{};

    const double swing_span = std::max(in.swing_span, 1e-6);
    const double f_hz = std::max(in.f_hz, 1e-6);
    const double T_swing = swing_span / f_hz;
    const double v_planar = std::hypot(nominal_body.linear_mps.x, nominal_body.linear_mps.y);
    const double w_norm = vecNorm(nominal_body.angular_radps);
    const double vel_scale =
        std::clamp(0.40 + 0.92 * (v_planar / kVRefMps) + 0.38 * (w_norm / kWRefRadps), 0.48, 1.55);
    const double step_len = std::max(in.step_length_m * vel_scale, 0.0);

    // Stance starts at `anchor` and integrates the world-fixed support velocity until
    // `stance_end`. Returning to that same anchor is therefore the only nominal touchdown
    // that is continuous with phase zero of the following stance. The old independent
    // `stance_end + step_length` construction left a sizeable target jump whenever cadence,
    // command speed, and the adaptive step-length table did not happen to agree.
    //
    // Capture/stability corrections remain explicit offsets from the continuous nominal
    // endpoint and are bounded below. `step_len` is retained as their scale.
    out.nominal_body = in.anchor;

    Vec3 measured_capture{};
    if (est.valid && est.has_body_twist_state) {
        // Use a single planar capture channel driven by measured body motion. Roll/pitch are
        // intentionally excluded here so the foothold correction does not double-count posture
        // compensation that is already handled in pose / stance shaping.
        BodyTwist measured{};
        measured.linear_mps = est.body_twist_state.body_trans_mps.raw();
        measured.angular_radps = est.body_twist_state.twist_vel_radps.raw();
        measured.linear_mps.z = 0.0;
        measured.angular_radps.x = 0.0;
        measured.angular_radps.y = 0.0;
        const double horizon = T_swing + std::max(0.0, in.stance_lookahead_s);
        measured_capture = twistIntegratedFootholdDeltaXY(measured, in.stance_end, horizon);
    }
    measured_capture = measured_capture + stabilityFootholdBiasXY(in.static_stability_margin_m, in.stance_end);
    out.capture_raw_body = measured_capture;
    out.capture_limit_m = std::clamp(kCaptureLimitScale * step_len, kCaptureLimitMinM, kCaptureLimitMaxM);
    out.capture_body = clampFootholdExtraXY(out.capture_raw_body, out.capture_limit_m);
    out.final_body = out.nominal_body + out.capture_body;
    return out;
}

void planStanceFoot(const StanceFootInputs& in, Vec3& pos_body, Vec3& vel_body) {
    // Integrate v_foot_body = twist-field stance velocity so the contact stays world-fixed (stage 4).
    const double f = std::max(in.f_hz, 1e-6);
    const double phi = clamp01(in.phase);
    pos_body = in.anchor + in.v_foot_body * (phi / f);
    vel_body = in.v_foot_body;
}

SwingPlanCommit resolveSwingPlan(const RobotState& est,
                                 const BodyTwist& nominal_body,
                                 const SwingFootInputs& in) {
    SwingPlanCommit plan{};
    plan.swing_span = std::max(in.swing_span, 1e-6);
    plan.f_hz = std::max(in.f_hz, 1e-6);
    const double T_swing = plan.swing_span / plan.f_hz;
    const double v_planar = std::hypot(nominal_body.linear_mps.x, nominal_body.linear_mps.y);
    const double w_norm = vecNorm(nominal_body.angular_radps);
    const double vel_scale =
        std::clamp(0.40 + 0.92 * (v_planar / kVRefMps) + 0.38 * (w_norm / kWRefRadps), 0.48, 1.55);

    const SwingFootPlanDecomposition foothold = computeSwingFootPlacement(est, nominal_body, in);
    // `swing_height_m` already includes the gait's physical clearance floor. Low-speed
    // velocity scaling must not reduce the realized trajectory below that floor; it may only
    // add clearance for faster motion.
    plan.swing_height_m = std::max(in.swing_height_m * std::max(vel_scale, 1.0), 0.0);
    plan.p0x = in.stance_end.x;
    plan.p0y = in.stance_end.y;
    plan.p3x = foothold.final_body.x;
    plan.p3y = foothold.final_body.y;

    plan.m0x = in.v_liftoff_body.x * T_swing;
    plan.m0y = in.v_liftoff_body.y * T_swing;

    const Vec3 v_touch = supportFootVelocityAt(Vec3{plan.p3x, plan.p3y, in.anchor.z}, nominal_body);
    plan.m1x = v_touch.x * T_swing;
    plan.m1y = v_touch.y * T_swing;
    const double stride_mag = std::hypot(plan.p3x - plan.p0x, plan.p3y - plan.p0y);
    constexpr double kM1VsStride = 2.5;
    const double m1_lim = std::max(0.02, kM1VsStride * stride_mag);
    const double m1_mag = std::hypot(plan.m1x, plan.m1y);
    if (m1_mag > m1_lim && m1_mag > 1e-12) {
        const double s = m1_lim / m1_mag;
        plan.m1x *= s;
        plan.m1y *= s;
    }

    plan.anchor_y = in.anchor.y;
    plan.anchor_z = in.anchor.z;
    plan.time_ease = std::clamp(in.swing_time_ease_01, 0.0, 1.0);
    plan.valid = true;
    return plan;
}

void evalSwingPlan(const SwingPlanCommit& plan, const double tau01, Vec3& pos_body, Vec3& vel_body) {
    const double tau = clamp01(tau01);
    const double chain = (1.0 / std::max(plan.swing_span, 1e-6)) * std::max(plan.f_hz, 1e-6);

    double sx = 0.0;
    double sy = 0.0;
    double dpx_dtau = 0.0;
    double dpy_dtau = 0.0;
    swing_trajectory::evalSwingPlanarBezier(
        tau, plan.time_ease, plan.p0x, plan.p0y, plan.p3x, plan.p3y,
        plan.m0x, plan.m0y, plan.m1x, plan.m1y, &sx, &sy, &dpx_dtau, &dpy_dtau);

    // Keep the cubic from tucking more than 40 mm inside the hip-signed anchor Y.
    // Capture/stability already bound the foothold; this floor only stops the realized
    // swing XY from running through the hip relative to that anchor.
    constexpr double kSwingMedialYMaxInsideM = 0.040;
    const double min_abs_y = std::max(0.0, std::abs(plan.anchor_y) - kSwingMedialYMaxInsideM);
    if (std::abs(plan.anchor_y) > 1e-9 && std::abs(sy) < min_abs_y) {
        const bool inward_vel = (plan.anchor_y >= 0.0) ? (dpy_dtau < 0.0) : (dpy_dtau > 0.0);
        sy = std::copysign(min_abs_y, plan.anchor_y);
        if (inward_vel) {
            dpy_dtau = 0.0;
        }
    }

    const double vx_s = dpx_dtau * chain;
    const double vy_s = dpy_dtau * chain;

    const double s_vert = swing_trajectory::timeWarp(tau, plan.time_ease);
    const double ds_vert_dtau = swing_trajectory::timeWarpDeriv(tau, plan.time_ease);
    double z_rel = 0.0;
    double dz_ds = 0.0;
    swingVerticalShape(plan.swing_height_m, s_vert, &z_rel, &dz_ds);
    const double dz_dtau = dz_ds * ds_vert_dtau;
    const double vz_s = dz_dtau * chain;

    pos_body = Vec3{sx, sy, plan.anchor_z + z_rel};
    vel_body = Vec3{vx_s, vy_s, vz_s};
}

void planSwingFoot(const RobotState& est, const BodyTwist& nominal_body, const SwingFootInputs& in, Vec3& pos_body, Vec3& vel_body) {
    evalSwingPlan(resolveSwingPlan(est, nominal_body, in), in.tau01, pos_body, vel_body);
}
