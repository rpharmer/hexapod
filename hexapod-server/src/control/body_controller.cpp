#include "body_controller.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kDefaultBodyHeightM = 0.05;
constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;

void strideDirectionXY(const double rx,
                       const double ry,
                       const GaitType gait,
                       const PlanarMotionCommand& cmd,
                       double* out_ux,
                       double* out_uy) {
    const double vx = cmd.vx_mps;
    const double vy = cmd.vy_mps;
    const double w = cmd.yaw_rate_radps;
    const double planar = std::hypot(vx, vy);
    double tx = -ry;
    double ty = rx;
    const double tn = std::hypot(tx, ty);
    if (tn > 1e-6) {
        tx /= tn;
        ty /= tn;
        if (w < 0.0) {
            tx = -tx;
            ty = -ty;
        }
    }

    if (gait == GaitType::TURN_IN_PLACE) {
        if (std::abs(w) > 1e-6 && tn > 1e-6) {
            *out_ux = tx;
            *out_uy = ty;
            return;
        }
    }

    if (planar > 1e-6) {
        *out_ux = vx / planar;
        *out_uy = vy / planar;
        return;
    }

    if (std::abs(w) > 1e-6 && tn > 1e-6) {
        *out_ux = tx;
        *out_uy = ty;
        return;
    }

    *out_ux = 1.0;
    *out_uy = 0.0;
}

} // namespace

std::array<Vec3, kNumLegs> BodyController::nominalStance(double body_height_m) const {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const double femur_tibia_reach =
            std::max(0.0, leg_geo.femurLength.value + leg_geo.tibiaLength.value - kReachMarginM);
        const double desired_rho = kNominalReachFraction * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);

        const double desired_foot_z_body = -body_height_m;
        double foot_z_in_leg_frame = desired_foot_z_body - leg_geo.bodyCoxaOffset.z;
        if (std::abs(foot_z_in_leg_frame) > femur_tibia_reach) {
            foot_z_in_leg_frame = std::copysign(femur_tibia_reach, foot_z_in_leg_frame);
        }

        const double max_rho =
            std::sqrt(std::max(0.0, femur_tibia_reach * femur_tibia_reach - foot_z_in_leg_frame * foot_z_in_leg_frame));
        const double rho = std::min(desired_rho, max_rho);
        const Vec3 neutral_leg_frame{leg_geo.coxaLength.value + rho, 0.0, foot_z_in_leg_frame};
        const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
        nominal[leg] = leg_geo.bodyCoxaOffset + (body_from_leg * neutral_leg_frame);
    }
    return nominal;
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();

    (void)est;

    double commanded_body_height_m = intent.twist.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    const std::array<Vec3, kNumLegs> nominal = nominalStance(commanded_body_height_m);
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    const double roll_cmd = intent.twist.twist_pos_rad.x;
    const double pitch_cmd = intent.twist.twist_pos_rad.y;
    const double yaw_cmd = intent.twist.twist_pos_rad.z;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const double duty = std::clamp(gait.duty_factor, 0.06, 0.94);
    const double f_hz = std::max(gait.stride_phase_rate_hz.value, 1e-6);
    const double swing_span = std::max(1.0 - duty, 1e-6);
    const double step_len = std::max(gait.step_length_m, 0.0);
    const double swing_h = std::max(gait.swing_height_m, 0.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];
        Vec3 target_vel = Vec3{};

        target = target - planar_body_offset;
        target_vel = target_vel - intent.twist.body_trans_mps;

        if (walking) {
            const double phase = clamp01(gait.phase[leg]);
            const Vec3 anchor = nominal[leg] - planar_body_offset;

            if (gait.in_stance[leg]) {
                const Vec3 v_foot(-cmd.vx_mps + cmd.yaw_rate_radps * anchor.y,
                                  -cmd.vy_mps - cmd.yaw_rate_radps * anchor.x,
                                  0.0);
                target = anchor + v_foot * (phase / f_hz);
                target_vel = target_vel + v_foot;
            } else {
                const Vec3 v_foot(-cmd.vx_mps + cmd.yaw_rate_radps * anchor.y,
                                  -cmd.vy_mps - cmd.yaw_rate_radps * anchor.x,
                                  0.0);
                const Vec3 stance_end = anchor + v_foot * (duty / f_hz);
                const double tau = clamp01((phase - duty) / swing_span);

                double ux = 0.0;
                double uy = 0.0;
                strideDirectionXY(anchor.x, anchor.y, intent.gait, cmd, &ux, &uy);

                const double p0x = stance_end.x;
                const double p0y = stance_end.y;
                const double p3x = p0x + ux * step_len;
                const double p3y = p0y + uy * step_len;

                const double t = tau;
                const double u = 1.0 - t;
                const double w0 = u * u * (1.0 + 2.0 * t);
                const double w3 = t * t * (3.0 - 2.0 * t);
                const double sx = p0x * w0 + p3x * w3;
                const double sy = p0y * w0 + p3y * w3;
                const double s_z_lift = std::sin(kPi * t);
                const double sz = anchor.z + swing_h * s_z_lift * s_z_lift;

                const double dP_dtau = 6.0 * t * (1.0 - t);
                const double chain = (1.0 / swing_span) * f_hz;
                const double vx_s = (p3x - p0x) * dP_dtau * chain;
                const double vy_s = (p3y - p0y) * dP_dtau * chain;
                const double dz_dtau = swing_h * kPi * std::sin(2.0 * kPi * t);
                const double vz_s = dz_dtau * chain;

                target = Vec3{sx, sy, sz};
                target_vel = target_vel + Vec3{vx_s, vy_s, vz_s};
            }
        }

        target = body_rotation * target;
        target_vel = body_rotation * target_vel;
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    return out;
}
