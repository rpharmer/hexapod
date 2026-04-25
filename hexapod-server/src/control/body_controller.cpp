#include "body_controller.hpp"

#include "body_pose_controller.hpp"
#include "foot_terrain.hpp"
#include "contact_foot_response.hpp"
#include "foot_planners.hpp"
#include "foot_reachability.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

BodyController::BodyController(control_config::GaitConfig gait_cfg,
                               control_config::FootTerrainConfig foot_terrain_cfg)
    : foot_estimator_blend_(std::clamp(gait_cfg.foot_estimator_blend, 0.0, 1.0)),
      foot_terrain_cfg_(foot_terrain_cfg) {}

namespace {

constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;
constexpr double kFootReachInsetM = 0.004;
constexpr double kBodyHeightHoldGain = 1.35;
constexpr double kBodyHeightHoldMaxAdjustM = 0.260;
constexpr double kTerrainBlendMinScale = 0.35;
constexpr double kTerrainBlendSagScale = 0.65;

double fusionTrustScale(const RobotState& est) {
    if (!est.has_fusion_diagnostics) {
        return 1.0;
    }
    return std::clamp(est.fusion.model_trust, 0.20, 1.0);
}

double bodyHeightHoldOffsetM(const RobotState& est, const double commanded_body_height_m) {
    if (!est.has_body_twist_state) {
        return 0.0;
    }
    const double measured_body_height_m = est.body_twist_state.body_trans_m.z;
    if (!std::isfinite(measured_body_height_m)) {
        return 0.0;
    }

    // Only correct sag, not overshoot. The goal is to keep the body from settling lower than the
    // requested stance without adding a new downward oscillation.
    const double sag_m = std::max(0.0, commanded_body_height_m - measured_body_height_m);
    if (sag_m <= 0.0) {
        return 0.0;
    }

    // Height hold should follow the measured body height directly. Fusion trust is useful for
    // resync decisions, but it should not dilute the stance correction that keeps the chassis up.
    return std::clamp(kBodyHeightHoldGain * sag_m, 0.0, kBodyHeightHoldMaxAdjustM);
}

double terrainBlendScaleForHeightHold(const double height_hold_m) {
    const double hold_ratio = std::clamp(height_hold_m / std::max(kBodyHeightHoldMaxAdjustM, 1e-6), 0.0, 1.0);
    return std::clamp(1.0 - kTerrainBlendSagScale * hold_ratio, kTerrainBlendMinScale, 1.0);
}

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
                                  const SafetyState& safety,
                                  const BodyTwist& cmd_twist,
                                  const LocalMapSnapshot* terrain_snapshot) {
    LegTargets out{};
    out.timestamp_us = now_us();

    const PlanarMotionCommand cmd = planarMotionFromCommandTwist(cmd_twist);
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, gait.static_stability_margin_m, gait.stride_phase_rate_hz.value);

    const double commanded_body_height_m = pose.body_height_m;
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;
    const double body_height_hold_m = bodyHeightHoldOffsetM(est, commanded_body_height_m);
    const bool height_hold_active = body_height_hold_m > 1e-6;
    const double terrain_blend_scale =
        height_hold_active ? 0.0 : terrainBlendScaleForHeightHold(body_height_hold_m);
    const double effective_body_height_m = commanded_body_height_m + body_height_hold_m;
    std::array<Vec3, kNumLegs> nominal = nominalStance(effective_body_height_m);

    if (walking && terrain_snapshot != nullptr && terrain_blend_scale > 0.0) {
        applyTerrainStanceZBias(*terrain_snapshot, est, intent, foot_terrain_cfg_, terrain_blend_scale, &nominal);
    }

    const Mat3 body_rotation =
        (Mat3::rotZ(pose.yaw_rad) * Mat3::rotY(pose.pitch_rad) * Mat3::rotX(pose.roll_rad)).transpose();
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    const double trust_scale = fusionTrustScale(est);
    const BodyVelocityCommand body_mot =
        bodyVelocityForFootPlanning(est, cmd_twist, foot_estimator_blend_ * trust_scale);
    const double duty = std::clamp(gait.duty_factor, 0.06, 0.94);
    const double f_hz = std::max(gait.stride_phase_rate_hz.value, 1e-6);
    const double swing_span = std::max(1.0 - duty, 1e-6);
    const double step_len = std::max(gait.step_length_m, 0.0);
    const double swing_h = std::max(gait.swing_height_m, 0.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg] - planar_body_offset;
        Vec3 target_vel = walking
                              ? Vec3{}
                              : Vec3{-intent.twist.body_trans_mps.x,
                                     -intent.twist.body_trans_mps.y,
                                     -intent.twist.body_trans_mps.z};
        bool apply_workspace_clamp = true;

        if (walking) {
            double ph = clamp01(gait.phase[leg]);
            const Vec3 anchor = nominal[leg] - planar_body_offset;
            const Vec3 v_foot = supportFootVelocityAt(anchor, body_mot);

            if (ph < duty) {
                apply_workspace_clamp = false;
                StanceFootInputs st{};
                st.anchor = anchor;
                st.v_foot_body = v_foot;
                st.phase = ph;
                st.f_hz = f_hz;
                Vec3 p{};
                Vec3 v{};
                planStanceFoot(st, p, v);
                target = p;
                target_vel = target_vel + v;
                if (terrain_blend_scale > 0.0 && foot_terrain_cfg_.enable_stance_tilt_leveling &&
                    est.foot_contacts[static_cast<std::size_t>(leg)]) {
                    target.z += terrain_blend_scale * trust_scale *
                                contact_foot_response::stanceTiltLevelingDeltaZ(est, intent, anchor.x, anchor.y);
                }
            } else {
                const Vec3 stance_end = anchor + v_foot * (duty / f_hz);
                const Vec3 v_liftoff = supportFootVelocityAt(stance_end, body_mot);
                const double tau = clamp01((ph - duty) / swing_span);
                const double tau_for_terrain_xy = tau;
                double tau_use = tau;
                double swing_extra_down_z = 0.0;
                contact_foot_response::adjustSwingTauAndVerticalExtension(
                    true,
                    est.foot_contacts[static_cast<std::size_t>(leg)],
                    est,
                    tau,
                    tau_use,
                    swing_extra_down_z,
                    &est.foot_contact_fusion[static_cast<std::size_t>(leg)]);

                double ux = 0.0;
                double uy = 0.0;
                if (intent.gait == GaitType::TURN_IN_PLACE) {
                    strideDirectionXY(anchor.x, anchor.y, intent.gait, cmd, &ux, &uy);
                } else {
                    const double vf = std::hypot(v_foot.x, v_foot.y);
                    if (vf > 1e-6) {
                        ux = -v_foot.x / vf;
                        uy = -v_foot.y / vf;
                    } else {
                        strideDirectionXY(anchor.x, anchor.y, intent.gait, cmd, &ux, &uy);
                    }
                }

                SwingFootInputs sw{};
                sw.anchor = anchor;
                sw.stance_end = stance_end;
                sw.v_liftoff_body = v_liftoff;
                sw.tau01 = tau_use;
                sw.swing_span = swing_span;
                sw.f_hz = f_hz;
                sw.step_length_m = step_len;
                sw.swing_height_m = swing_h;
                sw.stride_ux = ux;
                sw.stride_uy = uy;
                sw.cmd_accel_body_x_mps2 = gait.cmd_accel_body_x_mps2;
                sw.cmd_accel_body_y_mps2 = gait.cmd_accel_body_y_mps2;
                sw.stance_lookahead_s = (duty / f_hz) * 0.48;
                sw.static_stability_margin_m = gait.static_stability_margin_m;
                sw.swing_time_ease_01 = gait.swing_time_ease_01;
                Vec3 p{};
                Vec3 v{};
                planSwingFoot(body_mot, sw, p, v);
                target = p;
                if (terrain_snapshot != nullptr) {
                    applyTerrainSwingXYNudge(*terrain_snapshot, est, foot_terrain_cfg_, tau_for_terrain_xy, &target);
                    applyTerrainSwingClearance(*terrain_snapshot, est, foot_terrain_cfg_, &target);
                }
                target.z -= swing_extra_down_z;
                target_vel = target_vel + v;
            }
        } else if (intent.requested_mode == RobotMode::STAND &&
                   est.foot_contacts[static_cast<std::size_t>(leg)]) {
            if (terrain_blend_scale > 0.0 && foot_terrain_cfg_.enable_stance_tilt_leveling) {
                target.z += terrain_blend_scale * trust_scale *
                            contact_foot_response::stanceTiltLevelingDeltaZ(est, intent, target.x, target.y);
            }
        }

        target = body_rotation * target;
        target_vel = body_rotation * target_vel;
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);

        if (apply_workspace_clamp) {
            const Vec3 target_before_reach = target;
            target = foot_reachability::clampFootPositionBody(geometry_.legGeometry[leg], target, kFootReachInsetM);
            foot_reachability::clipVelocityForReachClamp(target_before_reach, target, &target_vel);
        }

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    return out;
}
