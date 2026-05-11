#include "body_controller.hpp"

#include "body_pose_controller.hpp"
#include "foot_terrain.hpp"
#include "contact_foot_response.hpp"
#include "foot_planners.hpp"
#include "foot_reachability.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

BodyController::BodyController(control_config::GaitConfig gait_cfg,
                               control_config::FootTerrainConfig foot_terrain_cfg)
    : foot_estimator_blend_(std::clamp(gait_cfg.foot_estimator_blend, 0.0, 1.0)),
      foot_terrain_cfg_(foot_terrain_cfg) {}

namespace {

constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;
constexpr double kFootReachInsetM = 0.004;
constexpr double kBodyHeightHoldGain = 1.0;
constexpr double kBodyHeightHoldMaxAdjustM = 0.120;
constexpr double kBodyHeightHoldIntegralGain = 0.02;  // leaky extra-support term for persistent compliance sag
constexpr double kBodyHeightHoldIntegralCapM = 0.020; // cap extra support so the chassis does not chase large transients
constexpr double kBodyHeightHoldIntegralDecay = 0.99;     // per-step decay when body is near commanded
constexpr double kBodyHeightHoldIntegralDecayFast = 0.93; // fast decay when body is above commanded (~250 ms to clear)
constexpr double kBodyHeightHoldIntegralFastUnwindGapM = 0.002;
constexpr double kBodyHeightHoldMinEffectiveMarginM = 0.008;
constexpr double kBodyHeightHoldMaxEffectiveMarginM = 0.012;
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

namespace body_controller_detail {

double updateBodyHeightHoldIntegralM(const double current_integral_m,
                                     const double commanded_body_height_m,
                                     const bool has_measured_body_height,
                                     const double measured_body_height_m) {
    double integral_m = std::clamp(current_integral_m, 0.0, kBodyHeightHoldIntegralCapM);
    if (!has_measured_body_height || !std::isfinite(measured_body_height_m)) {
        return integral_m * kBodyHeightHoldIntegralDecay;
    }

    const double height_error_m = commanded_body_height_m - measured_body_height_m; // positive = sagging
    if (height_error_m > 1e-4) {
        // Stale-integral fast-unwind: fires only when the body is already very close to the
        // commanded height (error < 2 mm) but the integral has accumulated far
        // more than the current error requires.  This drains leftover correction that built
        // up during a previous gait phase without fighting the integrator when the body is
        // still actively sagging.
        const bool stale_transition_hold =
            (integral_m - height_error_m) > kBodyHeightHoldIntegralFastUnwindGapM &&
            height_error_m < kBodyHeightHoldIntegralFastUnwindGapM;
        if (stale_transition_hold) {
            return integral_m * kBodyHeightHoldIntegralDecayFast;
        }
        return std::clamp(
            integral_m * kBodyHeightHoldIntegralDecay +
                kBodyHeightHoldIntegralGain * height_error_m,
            0.0,
            kBodyHeightHoldIntegralCapM);
    }
    if (height_error_m < -1e-4) {
        return integral_m * kBodyHeightHoldIntegralDecayFast;
    }
    return integral_m * kBodyHeightHoldIntegralDecay;
}

} // namespace body_controller_detail

std::array<Vec3, kNumLegs> computeNominalStance(const HexapodGeometry& geometry, double body_height_m) {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
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

std::array<Vec3, kNumLegs> BodyController::nominalStance(double body_height_m) const {
    return computeNominalStance(geometry_, body_height_m);
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety,
                                  const BodyTwist& cmd_twist,
                                  const LocalMapSnapshot* terrain_snapshot,
                                  const std::array<LegContactDecision, kNumLegs>* contact_modes) {
    LegTargets out{};
    out.timestamp_us = now_us();

    const PlanarMotionCommand cmd = planarMotionFromCommandTwist(cmd_twist);
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;
    const double trust_scale = fusionTrustScale(est);
    BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, gait.static_stability_margin_m, gait.stride_phase_rate_hz.value);
    if (walking && est.has_body_twist_state) {
        const double roll_meas = est.body_twist_state.twist_pos_rad.x;
        const double pitch_meas = est.body_twist_state.twist_pos_rad.y;
        if (std::isfinite(roll_meas) && std::isfinite(pitch_meas)) {
            constexpr double kTiltFeedbackGain = 0.24;
            constexpr double kTiltFeedbackMaxRad = 0.24;
            const double roll_error = roll_meas - pose.roll_rad;
            const double pitch_error = pitch_meas - pose.pitch_rad;
            const double roll_correction = std::clamp(
                -kTiltFeedbackGain * trust_scale * roll_error, -kTiltFeedbackMaxRad, kTiltFeedbackMaxRad);
            const double pitch_correction = std::clamp(
                -kTiltFeedbackGain * trust_scale * pitch_error, -kTiltFeedbackMaxRad, kTiltFeedbackMaxRad);
            pose.roll_rad += roll_correction;
            pose.pitch_rad += pitch_correction;
        }
    }

    const double commanded_body_height_m = pose.body_height_m;

    // Integrate persistent body height sag. Converges toward zero steady-state error
    // at a rate driven by actual servo compliance — no fixed compliance model assumed.
    // Decays when the body is at or above the commanded height (anti-windup).
    const bool has_measured_body_height = est.has_body_twist_state &&
                                          std::isfinite(est.body_twist_state.body_trans_m.z);
    const double measured_body_height_m =
        has_measured_body_height ? est.body_twist_state.body_trans_m.z : 0.0;
    height_hold_integral_m_ = body_controller_detail::updateBodyHeightHoldIntegralM(
        height_hold_integral_m_,
        commanded_body_height_m,
        has_measured_body_height,
        measured_body_height_m);

    const double body_height_hold_m = bodyHeightHoldOffsetM(est, commanded_body_height_m)
                                      + height_hold_integral_m_;
    double tilt_squat_m = 0.0;
    if (walking && est.has_body_twist_state) {
        const double roll_meas = est.body_twist_state.twist_pos_rad.x;
        const double pitch_meas = est.body_twist_state.twist_pos_rad.y;
        if (std::isfinite(roll_meas) && std::isfinite(pitch_meas)) {
            const double tilt_mag = std::hypot(roll_meas, pitch_meas);
            const double squat_over = std::max(0.0, tilt_mag - 0.08);
            tilt_squat_m = std::clamp(0.12 * squat_over, 0.0, 0.06);
        }
    }
    const double terrain_blend_scale = terrainBlendScaleForHeightHold(body_height_hold_m);
    const double effective_body_height_m = std::clamp(
        commanded_body_height_m + body_height_hold_m - tilt_squat_m,
        std::max(0.04, commanded_body_height_m - kBodyHeightHoldMinEffectiveMarginM),
        commanded_body_height_m + kBodyHeightHoldMaxEffectiveMarginM);
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
            const std::size_t leg_index = static_cast<std::size_t>(leg);
            const ContactPhase contact_phase = est.foot_contact_fusion[leg_index].phase;
            const LegContactDecision* contact_decision =
                contact_modes != nullptr ? &(*contact_modes)[leg_index] : nullptr;
            const bool planned_stance =
                contact_decision != nullptr ? contact_decision->planned_stance : ph < duty;
            const bool support_hold =
                contact_decision != nullptr
                    ? (contact_decision->mode == LegContactMode::HeldStance ||
                       contact_decision->mode == LegContactMode::LostCandidate)
                    : (gait.stability_hold_stance[leg_index] &&
                       (est.foot_contacts[leg_index] || contact_phase == ContactPhase::LostCandidate));
            const bool effective_stance =
                contact_decision != nullptr ? contact_decision->effective_stance : (planned_stance || support_hold);
            const bool recovery_touchdown =
                contact_decision != nullptr
                    ? contact_decision->mode == LegContactMode::RecoveryTouchdown
                    : (gait.stride_phase_rate_hz.value <= 1e-6 &&
                       !effective_stance &&
                       !est.foot_contacts[leg_index] &&
                       contact_phase != ContactPhase::LostCandidate);
            if (recovery_touchdown) {
                // During recovery hold the gait cadence is frozen. Unsupported swing legs still
                // need to finish their touchdown arc instead of hovering mid-swing, so bias them
                // to the late-swing segment while supported legs are held in stance.
                ph = std::max(ph, duty + 0.98 * swing_span);
            }
            // When transitioning to swing the foot is still touching ground while the servo
            // begins to lift. Raw contact during early swing with a ConfirmedStance fusion
            // phase is a liftoff artifact: sending stance targets creates a deadlock where
            // the server never commands lift. Allow swing kinematics during this grace window
            // so the servo can actually drive the foot off the ground.
            const double tau_swing_prelim =
                contact_decision != nullptr ? contact_decision->swing_tau
                                            : (effective_stance ? 0.0 : clamp01((ph - duty) / swing_span));
            // Grace fires on any contact during early swing regardless of fusion phase.
            // The ConfirmedStance check was too strict: fusion transitions away quickly,
            // leaving 50%+ of early-swing contacts incorrectly locked to stance targets.
            const bool liftoff_grace =
                contact_decision != nullptr
                    ? contact_decision->mode == LegContactMode::ContactGrace
                    : (!effective_stance && est.foot_contacts[leg_index] && tau_swing_prelim < 0.45);
            const bool lost_candidate_grace =
                contact_decision != nullptr
                    ? contact_decision->mode == LegContactMode::LostCandidate
                    : (effective_stance && contact_phase == ContactPhase::LostCandidate);
            const bool use_stance_kinematics =
                contact_decision != nullptr
                    ? contact_decision->use_stance_kinematics
                    : (effective_stance || (est.foot_contacts[leg_index] && !liftoff_grace) || lost_candidate_grace);

            // Diagnostic: log kinematics selection when in early-swing contact.
            // Enable with HEXAPOD_DIAG_LOG=1.
            if (est.foot_contacts[leg_index] && !planned_stance) {
                static bool s_diag_init = false;
                static bool s_diag_enabled = false;
                if (!s_diag_init) {
                    const char* v = std::getenv("HEXAPOD_DIAG_LOG");
                    s_diag_enabled = v && v[0] != '\0' && v[0] != '0';
                    s_diag_init = true;
                }
                if (s_diag_enabled) {
                    std::fprintf(stderr,
                        "[DIAG_SVR] leg=%d ph=%.3f tau_sw=%.3f grace=%d hold=%d use_stance=%d fusion=%d\n",
                        leg,
                        ph,
                        tau_swing_prelim,
                        static_cast<int>(liftoff_grace),
                        static_cast<int>(support_hold),
                        static_cast<int>(use_stance_kinematics),
                        static_cast<int>(contact_phase));
                    std::fflush(stderr);
                }
            }

            if (use_stance_kinematics) {
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
                    est.foot_contacts[leg_index]) {
                    target.z += terrain_blend_scale * trust_scale *
                                contact_foot_response::stanceTiltLevelingDeltaZ(est, intent, anchor.x, anchor.y);
                }
            } else {
                const double swing_f_hz = recovery_touchdown ? 1.0 : f_hz;
                const Vec3 stance_end = anchor + v_foot * (duty / swing_f_hz);
                const Vec3 v_liftoff = supportFootVelocityAt(stance_end, body_mot);
                const double tau = clamp01((ph - duty) / swing_span);
                const double tau_for_terrain_xy = tau;
                double tau_use = tau;
                double swing_extra_down_z = 0.0;
                contact_foot_response::adjustSwingTauAndVerticalExtension(
                    true,
                    est.foot_contacts[leg_index],
                    est,
                    tau,
                    tau_use,
                    swing_extra_down_z,
                    &est.foot_contact_fusion[leg_index]);

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
                sw.f_hz = swing_f_hz;
                sw.step_length_m = step_len;
                sw.swing_height_m = swing_h;
                sw.stride_ux = ux;
                sw.stride_uy = uy;
                sw.cmd_accel_body_x_mps2 = gait.cmd_accel_body_x_mps2;
                sw.cmd_accel_body_y_mps2 = gait.cmd_accel_body_y_mps2;
                sw.stance_lookahead_s = (duty / swing_f_hz) * 0.48;
                sw.static_stability_margin_m = gait.static_stability_margin_m;
                sw.swing_time_ease_01 = gait.swing_time_ease_01;
                Vec3 p{};
                Vec3 v{};
                planSwingFoot(est, cmd_twist, sw, p, v);
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
