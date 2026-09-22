#include "body_controller.hpp"

#include "body_pose_controller.hpp"
#include "foot_terrain.hpp"
#include "contact_foot_response.hpp"
#include "foot_planners.hpp"
#include "foot_reachability.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

BodyController::BodyController(control_config::GaitConfig gait_cfg,
                               control_config::FootTerrainConfig foot_terrain_cfg)
    : foot_estimator_blend_(std::clamp(gait_cfg.foot_estimator_blend, 0.0, 1.0)),
      foot_terrain_cfg_(foot_terrain_cfg) {}

void BodyController::reset() {
    height_hold_integral_m_ = 0.0;
    last_intent_timestamp_us_ = TimePointUs{};
    have_stance_pos_.fill(false);
    latched_stance_pos_.fill(Vec3{});
    latched_plant_pos_.fill(Vec3{});
    latched_stroke_l_m_.fill(0.0);
    last_planned_stance_.fill(false);
    last_stroke_clamp_hit_.fill(false);
    last_workspace_xy_hit_.fill(false);
    have_last_clamped_stance_.fill(false);
    last_clamped_stance_body_.fill(Vec3{});
    committed_swing_plan_.fill(SwingPlanCommit{});
    last_emitted_target_.fill(Vec3{});
    have_last_emitted_target_.fill(false);
    have_support_foot_world_z_.fill(false);
    last_r2_swing_decomp_ = {};
    stand_untilt_ticks_ = 0;
}

namespace {

/**
 * Opt-in screen, **default off and rejected** (leftover §3.15). Commits one swing
 * plan per swing instead of re-resolving it every control sample, and bounds the
 * emitted Cartesian step to the leg's actuator envelope.
 *
 * It does what it claims: swing commanded foot speed p99 falls 8.70 -> 0.735 m/s
 * and no sample steps more than 3.7 mm. It still regressed every screen (isolated
 * reverse 4/5 -> 1/5, sequential 3/5 -> 0/5), and drag stayed at 76-82%. Making
 * the commanded path feasible therefore does not free the dragging legs — with
 * `peak_solver_servo_torque_utilization` at 1.0 the binding constraint is servo
 * torque, not command quality.
 */
bool swingPlanCommitEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("HEXAPOD_SWING_PLAN_COMMIT");
        return value != nullptr && value[0] != '\0' && std::string{value} != "0";
    }();
    return enabled;
}

} // namespace

const SwingPlanCommit& BodyController::commitSwingPlan(const std::size_t leg_index,
                                                      const RobotState& est,
                                                      const BodyTwist& nominal_body,
                                                      const SwingFootInputs& in) {
    SwingPlanCommit& plan = committed_swing_plan_[leg_index];
    if (!plan.valid || !swingPlanCommitEnabled()) {
        plan = resolveSwingPlan(est, nominal_body, in);
    }
    return plan;
}

namespace {

/**
 * Cartesian output rate bound. A joint driven at `fraction · no-load speed` still
 * has torque left to accelerate the link, unlike the existing per-joint clamp at
 * 100% of no-load; `fraction · ω_noload · r` is the foot speed that buys. The
 * planner's own commanded speed wins when it is larger, so a legitimately fast
 * swing is never throttled — only steps that the trajectory itself did not ask
 * for are.
 */
constexpr double kFootEnvelopeNoLoadFraction = 0.5;
constexpr double kPlannerSpeedMargin = 1.5;
constexpr double kMinFootLeverArmM = 0.05;

constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;
constexpr double kFootReachInsetM = 0.004;
constexpr double kBodyHeightHoldGain = 1.0;
constexpr double kBodyHeightHoldMaxAdjustM = 0.120;
constexpr double kStaticBodyHeightPullDownMaxM = 0.020;
constexpr double kBodyHeightHoldIntegralGain = 0.02;  // leaky extra-support term for persistent compliance sag
constexpr double kBodyHeightHoldIntegralCapM = 0.020; // cap extra support so the chassis does not chase large transients
constexpr double kBodyHeightHoldIntegralDecay = 0.99;     // per-step decay when body is near commanded
constexpr double kBodyHeightHoldIntegralDecayFast = 0.93; // fast decay when body is above commanded (~250 ms to clear)
constexpr double kBodyHeightHoldIntegralFastUnwindGapM = 0.002;
// Walk cyclic sag at production 0.14 m requests more than 12 mm of hold
// (slow-fwd min ~20 mm, WAVE min ~35 mm) while the governor stays at command.
constexpr double kBodyHeightHoldMaxEffectiveMarginM = 0.040;
constexpr double kTerrainBlendMinScale = 0.35;
constexpr double kTerrainBlendSagScale = 0.65;
constexpr int kR2SwingDumpLeg = 2;
constexpr std::size_t kSwingPlannerDumpMaxSamples = 256;

void writeJsonVec3(std::ostream& out, const Vec3& v) {
    out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}

struct SwingPlannerDumpSample {
    double phase{0.0};
    double duty_factor{0.0};
    bool in_stance{false};
    SwingFootInputs sw{};
    Vec3 twist_linear_mps{};
    Vec3 twist_angular_radps{};
    bool est_valid{false};
    bool est_has_body_twist{false};
    Vec3 est_linear_mps{};
    Vec3 est_angular_radps{};
    Vec3 planned_pre_rot{};
    Vec3 target_clamped{};
};

struct SwingPlannerDumpState {
    bool decided{false};
    bool active{false};
    bool written{false};
    std::string path{};
    std::vector<SwingPlannerDumpSample> samples{};
};

SwingPlannerDumpState& swingPlannerDumpState() {
    static SwingPlannerDumpState state;
    return state;
}

void writeSwingPlannerDumpFile(SwingPlannerDumpState& state) {
    if (!state.active || state.path.empty() || state.samples.empty()) {
        return;
    }
    std::ofstream out(state.path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[swing-planner-dump] failed path=" << state.path << '\n';
        state.written = true;
        state.active = false;
        return;
    }
    out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"swing_planner_dump\",\"leg\":" << kR2SwingDumpLeg
        << ",\"samples\":[";
    for (std::size_t i = 0; i < state.samples.size(); ++i) {
        const SwingPlannerDumpSample& s = state.samples[i];
        if (i != 0) {
            out << ',';
        }
        out << "{\"phase\":" << s.phase
            << ",\"duty_factor\":" << s.duty_factor
            << ",\"in_stance\":" << (s.in_stance ? "true" : "false")
            << ",\"tau01\":" << s.sw.tau01
            << ",\"swing_span\":" << s.sw.swing_span
            << ",\"f_hz\":" << s.sw.f_hz
            << ",\"step_length_m\":" << s.sw.step_length_m
            << ",\"swing_height_m\":" << s.sw.swing_height_m
            << ",\"stance_lookahead_s\":" << s.sw.stance_lookahead_s
            << ",\"static_stability_margin_m\":" << s.sw.static_stability_margin_m
            << ",\"swing_time_ease_01\":" << s.sw.swing_time_ease_01
            << ",\"cmd_accel_body_x_mps2\":" << s.sw.cmd_accel_body_x_mps2
            << ",\"cmd_accel_body_y_mps2\":" << s.sw.cmd_accel_body_y_mps2
            << ",\"anchor\":";
        writeJsonVec3(out, s.sw.anchor);
        out << ",\"stance_end\":";
        writeJsonVec3(out, s.sw.stance_end);
        out << ",\"v_liftoff_body\":";
        writeJsonVec3(out, s.sw.v_liftoff_body);
        out << ",\"twist_linear_mps\":";
        writeJsonVec3(out, s.twist_linear_mps);
        out << ",\"twist_angular_radps\":";
        writeJsonVec3(out, s.twist_angular_radps);
        out << ",\"est_valid\":" << (s.est_valid ? "true" : "false")
            << ",\"est_has_body_twist\":" << (s.est_has_body_twist ? "true" : "false")
            << ",\"est_linear_mps\":";
        writeJsonVec3(out, s.est_linear_mps);
        out << ",\"est_angular_radps\":";
        writeJsonVec3(out, s.est_angular_radps);
        out << ",\"planned_pre_rot\":";
        writeJsonVec3(out, s.planned_pre_rot);
        out << ",\"target_clamped\":";
        writeJsonVec3(out, s.target_clamped);
        out << '}';
    }
    out << "]}\n";
    if (state.samples.size() == 1) {
        std::cerr << "[swing-planner-dump] path=" << state.path << '\n';
    }
}

void maybeRecordR2SwingPlannerDump(
    int leg,
    bool swinging,
    const GaitState& gait,
    const SwingFootInputs& sw,
    const BodyTwist& kinematic_twist,
    const RobotState& kinematic_est,
    const Vec3& planned_pre_rot,
    const Vec3& target_clamped) {
    if (leg != kR2SwingDumpLeg || !swinging) {
        return;
    }
    SwingPlannerDumpState& state = swingPlannerDumpState();
    if (state.written) {
        return;
    }
    if (!state.decided) {
        state.decided = true;
        const char* path = std::getenv("HEXAPOD_SWING_PLANNER_DUMP_PATH");
        if (path == nullptr || path[0] == '\0') {
            return;
        }
        std::ifstream exists(path);
        if (exists.good()) {
            state.written = true;
            std::cerr << "[swing-planner-dump] skip existing path=" << path << '\n';
            return;
        }
        state.active = true;
        state.path = path;
        state.samples.reserve(kSwingPlannerDumpMaxSamples);
    }
    if (!state.active) {
        return;
    }
    SwingPlannerDumpSample sample;
    sample.phase = gait.phase[static_cast<std::size_t>(kR2SwingDumpLeg)];
    sample.duty_factor = gait.duty_factor;
    sample.in_stance = gait.in_stance[static_cast<std::size_t>(kR2SwingDumpLeg)];
    sample.sw = sw;
    sample.twist_linear_mps = kinematic_twist.linear_mps;
    sample.twist_angular_radps = kinematic_twist.angular_radps;
    sample.est_valid = kinematic_est.valid;
    sample.est_has_body_twist = kinematic_est.has_body_twist_state;
    if (kinematic_est.has_body_twist_state) {
        sample.est_linear_mps = kinematic_est.body_twist_state.body_trans_mps.raw();
        sample.est_angular_radps = kinematic_est.body_twist_state.twist_vel_radps.raw();
    }
    sample.planned_pre_rot = planned_pre_rot;
    sample.target_clamped = target_clamped;
    state.samples.push_back(sample);
    writeSwingPlannerDumpFile(state);
    if (state.samples.size() >= kSwingPlannerDumpMaxSamples) {
        state.written = true;
        state.active = false;
    }
}

double fusionTrustScale(const RobotState& est) {
    if (!est.has_fusion_diagnostics) {
        return 1.0;
    }
    return std::clamp(est.fusion.model_trust, 0.20, 1.0);
}

double bodyHeightHoldOffsetM(const RobotState& est,
                             const double commanded_body_height_m,
                             const bool correct_static_overshoot) {
    if (!est.has_body_twist_state) {
        return 0.0;
    }
    const double measured_body_height_m = est.body_twist_state.body_trans_m.z;
    if (!std::isfinite(measured_body_height_m)) {
        return 0.0;
    }

    const double height_error_m = commanded_body_height_m - measured_body_height_m;
    if (height_error_m < 0.0) {
        // A stationary stance can settle above its requested height because the
        // compliant joint equilibrium and spherical feet do not reproduce the
        // unloaded IK pose exactly. Close that steady-state error in STAND only;
        // applying downward feedback during a moving gait would deepen its
        // cyclic support dips.
        return correct_static_overshoot
            ? std::clamp(
                  kBodyHeightHoldGain * height_error_m,
                  -kStaticBodyHeightPullDownMaxM,
                  0.0)
            : 0.0;
    }

    // Height hold should follow the measured body height directly. Fusion trust is useful for
    // resync decisions, but it should not dilute the stance correction that keeps the chassis up.
    return std::clamp(
        kBodyHeightHoldGain * height_error_m, 0.0, kBodyHeightHoldMaxAdjustM);
}

bool clampPlanarStrokeFromPlant(const Vec3& plant, const double stroke_l_m, Vec3& p) {
    if (!(stroke_l_m > 0.0)) {
        return false;
    }
    const double dx = p.x - plant.x;
    const double dy = p.y - plant.y;
    const double r = std::hypot(dx, dy);
    if (r <= stroke_l_m || r < 1e-12) {
        return false;
    }
    const double scale = stroke_l_m / r;
    p.x = plant.x + dx * scale;
    p.y = plant.y + dy * scale;
    return true;
}

/// Screen scale on the body-height hold; unset or invalid keeps production 1.0.
double heightHoldScale() {
    static const double scale = [] {
        const char* value = std::getenv("HEXAPOD_HEIGHT_HOLD_SCALE");
        if (value == nullptr || value[0] == '\0') {
            return 1.0;
        }
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed < 0.0
            || parsed > 1.0) {
            return 1.0;
        }
        return parsed;
    }();
    return scale;
}

double terrainBlendScaleForHeightHold(const double height_hold_m) {
    const double hold_ratio = std::clamp(height_hold_m / std::max(kBodyHeightHoldMaxAdjustM, 1e-6), 0.0, 1.0);
    return std::clamp(1.0 - kTerrainBlendSagScale * hold_ratio, kTerrainBlendMinScale, 1.0);
}

BodyTwist legacyKinematicTwistFromServerBody(const BodyTwist& server_body_twist) {
    // Geometry, commands and estimator state now share the canonical server
    // body frame.  Retain this boundary function so older call sites stay
    // explicit and future hardware-frame adapters have one insertion point.
    return server_body_twist;
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
        const Mat3 body_from_leg = bodyFromLegFrame(leg_geo);
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
    last_stroke_clamp_hit_.fill(false);
    last_workspace_xy_hit_.fill(false);
    last_r2_swing_decomp_ = {};

    // Keep differential motion conversion explicit at the kinematic boundary.
    const BodyTwist kinematic_twist = legacyKinematicTwistFromServerBody(cmd_twist);
    RobotState kinematic_est = est;
    if (kinematic_est.has_body_twist_state) {
        BodyTwist measured_server{};
        measured_server.linear_mps = kinematic_est.body_twist_state.body_trans_mps.raw();
        measured_server.angular_radps = kinematic_est.body_twist_state.twist_vel_radps.raw();
        const BodyTwist measured_kinematic = legacyKinematicTwistFromServerBody(measured_server);
        kinematic_est.body_twist_state.body_trans_mps = measured_kinematic.linear_mps;
        kinematic_est.body_twist_state.twist_vel_radps = measured_kinematic.angular_radps;
    }

    // Pose shaping and differential foot motion share the canonical server frame.
    // Keep the adapter call so a future hardware-frame map has one insertion point.
    const PlanarMotionCommand cmd = planarMotionFromCommandTwist(cmd_twist);
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;
    const double trust_scale = fusionTrustScale(est);
    BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, gait.static_stability_margin_m, gait.stride_phase_rate_hz.value);
    if (!walking && intent.requested_mode == RobotMode::STAND && est.has_body_twist_state
        && std::abs(pose.roll_rad) + std::abs(pose.pitch_rad) < 1.0e-6) {
        const double roll_meas = est.body_twist_state.twist_pos_rad.x;
        const double pitch_meas = est.body_twist_state.twist_pos_rad.y;
        if (std::isfinite(roll_meas) && std::isfinite(pitch_meas)) {
            // Identity STAND freezes a prefix 3-support (body-frame hexagon is tilted in
            // world). Command a heading-level hexagon (pose = +meas) so airborne feet
            // reach the floor, then fade to identity so the chassis can untilt onto that
            // plane — the live analog of the P5 yaw-only plant. Holding pose=+meas into
            // WALK is the mixed-q SpeedLimit plant. Same 2 s settle; no STAND lengthen.
            ++stand_untilt_ticks_;
            constexpr int kHoldTicks = 80;
            constexpr int kFadeTicks = 240;
            double fade = 0.0;
            if (stand_untilt_ticks_ <= kHoldTicks) {
                fade = 1.0;
            } else if (stand_untilt_ticks_ < kHoldTicks + kFadeTicks) {
                const double u = static_cast<double>(stand_untilt_ticks_ - kHoldTicks) /
                                 static_cast<double>(kFadeTicks);
                fade = 0.5 * (1.0 + std::cos(3.141592653589793 * u));
            }
            pose.roll_rad = fade * roll_meas;
            pose.pitch_rad = fade * pitch_meas;
        }
    } else {
        stand_untilt_ticks_ = 0;
    }
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

    const bool correct_static_overshoot =
        intent.requested_mode == RobotMode::STAND && !safety.torque_cut;
    // Screen only (default 1.0 = production). Body heave is 27-40 mm against a
    // 24-31 mm commanded swing height, so a swing foot barely clears the ground
    // (measured 20th-percentile lift 0.2-1.2 mm) and drags; heave correlates with
    // drag at r = +0.66 over 112 cases, and aborting cases carry 40 mm heave
    // versus 27 mm on clean ones. This gate asks whether the unity-gain height
    // hold is driving that heave.
    const double body_height_hold_m = (bodyHeightHoldOffsetM(
                                           est,
                                           commanded_body_height_m,
                                           correct_static_overshoot)
                                       + height_hold_integral_m_)
                                      * heightHoldScale();
    const double terrain_blend_scale = terrainBlendScaleForHeightHold(body_height_hold_m);
    // Protective squat is applied upstream by CommandGovernor and is already reflected in
    // commanded_body_height_m. Keep this layer responsible only for compensating measured sag;
    // applying another tilt squat here made the chassis bob down twice for the same disturbance.
    const double min_effective_body_height_m = correct_static_overshoot
        ? std::max(0.04, commanded_body_height_m - kStaticBodyHeightPullDownMaxM)
        : std::max(0.04, commanded_body_height_m);
    const double effective_body_height_m = std::clamp(
        commanded_body_height_m + body_height_hold_m,
        min_effective_body_height_m,
        commanded_body_height_m + kBodyHeightHoldMaxEffectiveMarginM);
    const double swing_height_hold_release_m =
        std::max(0.0, effective_body_height_m - commanded_body_height_m);
    std::array<Vec3, kNumLegs> nominal = nominalStance(effective_body_height_m);

    if (walking && terrain_snapshot != nullptr && terrain_blend_scale > 0.0) {
        applyTerrainStanceZBias(*terrain_snapshot, est, intent, foot_terrain_cfg_, terrain_blend_scale, &nominal);
    }

    const Mat3 body_rotation =
        (Mat3::rotZ(pose.yaw_rad) * Mat3::rotY(pose.pitch_rad) * Mat3::rotX(pose.roll_rad)).transpose();
    const Mat3 measured_rotation = Mat3::rotZ(est.body_twist_state.twist_pos_rad.z)
        * Mat3::rotY(est.body_twist_state.twist_pos_rad.y) * Mat3::rotX(est.body_twist_state.twist_pos_rad.x);
    const bool measured_pose_valid = est.valid && est.has_body_twist_state
        && std::isfinite(est.body_twist_state.body_trans_m.z)
        && std::isfinite(est.body_twist_state.twist_pos_rad.x)
        && std::isfinite(est.body_twist_state.twist_pos_rad.y)
        && std::isfinite(est.body_twist_state.twist_pos_rad.z)
        && std::isfinite(measured_rotation.m[2][2]) && measured_rotation.m[2][2] > 1e-6;
    if (!measured_pose_valid) have_support_foot_world_z_.fill(false);
    // Default-on after the support/clearance campaign (leftover §3.26).
    // Keep an explicit diagnostic opt-out for same-binary comparisons.
    static const bool contact_height_enabled = [] {
        const char* value = std::getenv("HEXAPOD_SWING_CONTACT_HEIGHT");
        return value == nullptr || std::string{value} != "0";
    }();
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    const BodyVelocityCommand body_mot =
        bodyVelocityForFootPlanning(kinematic_est, kinematic_twist, foot_estimator_blend_ * trust_scale);
    const double duty = std::clamp(gait.duty_factor, 0.06, 0.94);
    const double f_hz = std::max(gait.stride_phase_rate_hz.value, 1e-6);
    const double swing_span = std::max(1.0 - duty, 1e-6);
    const double step_len = std::max(gait.step_length_m, 0.0);
    const double swing_h = std::max(gait.swing_height_m, 0.0);
    double dt_s = 0.0;
    if (!intent.timestamp_us.isZero() && !last_intent_timestamp_us_.isZero() &&
        intent.timestamp_us.value > last_intent_timestamp_us_.value) {
        dt_s = std::min(
            0.05,
            static_cast<double>(intent.timestamp_us.value - last_intent_timestamp_us_.value) * 1.0e-6);
    }
    if (!intent.timestamp_us.isZero()) {
        last_intent_timestamp_us_ = intent.timestamp_us;
    }
    if (!walking) {
        have_stance_pos_.fill(false);
        latched_plant_pos_.fill(Vec3{});
        latched_stroke_l_m_.fill(0.0);
        last_planned_stance_.fill(false);
        last_stroke_clamp_hit_.fill(false);
        last_workspace_xy_hit_.fill(false);
        have_last_clamped_stance_.fill(false);
        last_clamped_stance_body_.fill(Vec3{});
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg] - planar_body_offset;
        Vec3 target_vel = walking
                              ? Vec3{}
                              : Vec3{-intent.twist.body_trans_mps.x,
                                     -intent.twist.body_trans_mps.y,
                                     -intent.twist.body_trans_mps.z};
        bool apply_workspace_clamp = true;
        bool used_stance_kinematics = false;
        double swing_lift_m = 0.0;
        double swing_lift_fraction = 0.0;
        if (contact_height_enabled && measured_pose_valid &&
            (!walking || gait.in_stance[leg]) && est.foot_contacts[leg]) {
            LegFK fk;
            const Vec3 measured_foot = fk.footInBodyFrame(est.leg_states[leg], geometry_.legGeometry[leg]).pos_body_m.raw();
            const double z = est.body_twist_state.body_trans_m.z + (measured_rotation * measured_foot).z;
            if (std::isfinite(z)) {
                support_foot_world_z_[leg] = z;
                have_support_foot_world_z_[leg] = true;
            } else {
                have_support_foot_world_z_[leg] = false;
            }
        }
        bool r2_swing_this_leg = false;
        SwingFootInputs r2_sw{};
        Vec3 r2_planned_pre_rot{};
        Vec3 r2_untilted{};
        Vec3 r2_terrain_xy_delta{};
        SwingFootPlanDecomposition r2_foothold{};

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
            if (planned_stance) {
                // Swing planning is only reachable with `planned_stance` false, so
                // dropping the commit here yields exactly one commit per swing.
                committed_swing_plan_[leg_index] = SwingPlanCommit{};
            }
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
                    : (effective_stance || lost_candidate_grace);

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
                used_stance_kinematics = true;
                Vec3 p{};
                Vec3 v{};
                const bool new_plant = !have_stance_pos_[leg_index];
                const bool replant_planned =
                    !new_plant && planned_stance && !last_planned_stance_[leg_index] && !support_hold;
                if (new_plant || replant_planned) {
                    if (planned_stance) {
                        StanceFootInputs st{};
                        st.anchor = anchor;
                        st.v_foot_body = v_foot;
                        // New plants start at the stance origin. Using gait φ here jumped
                        // STAND→WALK feet to a mid-stroke target when first-stride coverage
                        // seeds Φ away from 0. After this sample the latch integrates v dt.
                        st.phase = new_plant ? 0.0 : ph;
                        st.f_hz = f_hz;
                        planStanceFoot(st, p, v);
                    } else {
                        // Hold first plant: keep the swing foothold XY.
                        // Snapping to stance_end parked the next stance at the back of
                        // the disk (workspace XY gone, Cartesian collapsed).
                        const double swing_f_hz = recovery_touchdown ? 1.0 : f_hz;
                        const Vec3 stance_end = anchor + v_foot * (duty / swing_f_hz);
                        const Vec3 v_liftoff = supportFootVelocityAt(stance_end, body_mot);
                        const double tau = clamp01((ph - duty) / swing_span);
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
                        SwingFootInputs sw{};
                        sw.anchor = anchor;
                        sw.stance_end = stance_end;
                        sw.v_liftoff_body = v_liftoff;
                        sw.tau01 = tau_use;
                        sw.swing_span = swing_span;
                        sw.f_hz = swing_f_hz;
                        sw.step_length_m = step_len;
                        sw.swing_height_m = swing_h;
                        sw.cmd_accel_body_x_mps2 = gait.cmd_accel_body_x_mps2;
                        sw.cmd_accel_body_y_mps2 = gait.cmd_accel_body_y_mps2;
                        sw.stance_lookahead_s = (duty / swing_f_hz) * 0.48;
                        sw.static_stability_margin_m = gait.static_stability_margin_m;
                        sw.swing_time_ease_01 = gait.swing_time_ease_01;
                        evalSwingPlan(commitSwingPlan(leg_index, kinematic_est, kinematic_twist, sw),
                                      sw.tau01, p, v);
                        v = supportFootVelocityAt(p, body_mot);
                    }
                } else {
                    const Vec3 v_now = supportFootVelocityAt(latched_stance_pos_[leg_index], body_mot);
                    p = latched_stance_pos_[leg_index] + v_now * dt_s;
                    v = supportFootVelocityAt(p, body_mot);
                }
                target = p;
                target.z = anchor.z;
                target_vel = target_vel + v;
                if (terrain_blend_scale > 0.0 && foot_terrain_cfg_.enable_stance_tilt_leveling &&
                    est.foot_contacts[leg_index]) {
                    target.z += terrain_blend_scale * trust_scale *
                                contact_foot_response::stanceTiltLevelingDeltaZ(est, intent, anchor.x, anchor.y);
                }
                if (new_plant || replant_planned) {
                    latched_plant_pos_[leg_index] = target;
                    const double v_xy = std::hypot(v_foot.x, v_foot.y);
                    latched_stroke_l_m_[leg_index] = v_xy * (duty / f_hz);
                }
                last_stroke_clamp_hit_[leg_index] = clampPlanarStrokeFromPlant(
                    latched_plant_pos_[leg_index], latched_stroke_l_m_[leg_index], target);
                have_stance_pos_[leg_index] = true;
                latched_stance_pos_[leg_index] = target;
                last_planned_stance_[leg_index] = planned_stance;
            } else {
                have_stance_pos_[leg_index] = false;
                latched_plant_pos_[leg_index] = Vec3{};
                latched_stroke_l_m_[leg_index] = 0.0;
                last_planned_stance_[leg_index] = false;
                have_last_clamped_stance_[leg_index] = false;
                last_clamped_stance_body_[leg_index] = Vec3{};
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

                SwingFootInputs sw{};
                sw.anchor = anchor;
                sw.stance_end = stance_end;
                sw.v_liftoff_body = v_liftoff;
                sw.tau01 = tau_use;
                sw.swing_span = swing_span;
                sw.f_hz = swing_f_hz;
                sw.step_length_m = step_len;
                sw.swing_height_m = swing_h;
                sw.cmd_accel_body_x_mps2 = gait.cmd_accel_body_x_mps2;
                sw.cmd_accel_body_y_mps2 = gait.cmd_accel_body_y_mps2;
                sw.stance_lookahead_s = (duty / swing_f_hz) * 0.48;
                sw.static_stability_margin_m = gait.static_stability_margin_m;
                sw.swing_time_ease_01 = gait.swing_time_ease_01;
                Vec3 p{};
                Vec3 v{};
                evalSwingPlan(commitSwingPlan(leg_index, kinematic_est, kinematic_twist, sw),
                              sw.tau01, p, v);
                if (leg == kR2SwingDumpLeg) {
                    r2_swing_this_leg = true;
                    r2_sw = sw;
                    r2_planned_pre_rot = p;
                    r2_foothold = computeSwingFootPlacement(kinematic_est, kinematic_twist, sw);
                }
                target = p;
                // Height hold deliberately pushes planted feet below their unloaded nominal
                // position to counter servo compliance. A swing foot must not inherit that
                // support preload: doing so consumes most of the clearance arc and leaves the
                // 18 mm contact sphere dragging through nearly the entire swing.
                target.z += swing_height_hold_release_m;
                swing_lift_m = std::max(0.0, p.z - sw.anchor.z);
                const double resolved_height = committed_swing_plan_[leg_index].swing_height_m;
                swing_lift_fraction = resolved_height > 1e-9
                    ? std::clamp(swing_lift_m / resolved_height, 0.0, 1.0) : 0.0;
                const Vec3 before_terrain = target;
                if (terrain_snapshot != nullptr) {
                    applyTerrainSwingXYNudge(*terrain_snapshot, est, foot_terrain_cfg_, tau_for_terrain_xy, &target);
                    applyTerrainSwingClearance(*terrain_snapshot, est, foot_terrain_cfg_, &target);
                }
                target.z -= swing_extra_down_z;
                if (r2_swing_this_leg) {
                    r2_terrain_xy_delta = Vec3{target.x - before_terrain.x, target.y - before_terrain.y, 0.0};
                    r2_untilted = target;
                }
                target_vel = target_vel + v;
            }
        } else if (intent.requested_mode == RobotMode::STAND &&
                   est.foot_contacts[static_cast<std::size_t>(leg)]) {
            if (terrain_blend_scale > 0.0 && foot_terrain_cfg_.enable_stance_tilt_leveling) {
                target.z += terrain_blend_scale * trust_scale *
                            contact_foot_response::stanceTiltLevelingDeltaZ(est, intent, target.x, target.y);
            }
        }

        if (used_stance_kinematics) {
            // Stance lean about the coxa, not the body origin, so pitch/roll do not
            // translate the untilted plant in body XY before the stroke projector.
            const Vec3 coxa = geometry_.legGeometry[leg].bodyCoxaOffset;
            target = coxa + (body_rotation * (target - coxa));
            target_vel = body_rotation * target_vel;
        } else {
            target = body_rotation * target;
            target_vel = body_rotation * target_vel;
        }
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);

        if (contact_height_enabled && walking && !used_stance_kinematics &&
            measured_pose_valid && have_support_foot_world_z_[leg]) {
            // A loaded stance target can lie below the observed contact point.
            // Do not spend the swing clearance merely releasing that preload.
            // Use the existing smooth lift profile (zero at either endpoint),
            // not a new gain or a discontinuous target jump at liftoff.
            const double world_z = est.body_twist_state.body_trans_m.z + (measured_rotation * target).z;
            const double missing_clearance = support_foot_world_z_[leg] + swing_lift_m - world_z;
            target.z += swing_lift_fraction * std::max(0.0, missing_clearance) / measured_rotation.m[2][2];
        }

        if (apply_workspace_clamp) {
            const Vec3 target_before_reach = target;
            target = foot_reachability::clampFootPositionBody(geometry_.legGeometry[leg], target, kFootReachInsetM);
            foot_reachability::clipVelocityForReachClamp(target_before_reach, target, &target_vel);
        } else if (used_stance_kinematics) {
            const Vec3 target_before_reach = target;
            const std::size_t leg_index = static_cast<std::size_t>(leg);
            const Vec3* last_in_reach =
                have_last_clamped_stance_[leg_index] ? &last_clamped_stance_body_[leg_index] : nullptr;
            const foot_reachability::StrokeAlongStrokeResult projected =
                foot_reachability::clampPlantedFootPosition(
                    geometry_.legGeometry[leg], last_in_reach, target, kFootReachInsetM);
            target = projected.pos_body_m;
            last_workspace_xy_hit_[leg_index] = projected.planar_xy_hit;
            foot_reachability::clipVelocityForReachClamp(target_before_reach, target, &target_vel);
            last_clamped_stance_body_[leg_index] = target;
            have_last_clamped_stance_[leg_index] = true;
            const double planar_shift =
                std::hypot(target.x - target_before_reach.x, target.y - target_before_reach.y);
            if (planar_shift > 1e-9) {
                const Vec3 coxa = geometry_.legGeometry[leg].bodyCoxaOffset;
                Vec3 mixed = target_before_reach;
                mixed.x = target.x;
                mixed.y = target.y;
                const Vec3 untilted = coxa + (body_rotation.transpose() * (mixed - coxa));
                latched_stance_pos_[leg_index].x = untilted.x;
                latched_stance_pos_[leg_index].y = untilted.y;
            }
        }

        // Part of the rejected `HEXAPOD_SWING_PLAN_COMMIT` screen, default off.
        // Even with the swing plan committed, the contact-reactive tau advance and
        // the stance/swing branch switch step this target 65-107 mm inside one 5 ms
        // sample when a swinging foot touches down early. Bounding the emitted step
        // to the leg's Cartesian actuator envelope removes those steps entirely and
        // still regressed every screen; see leftover §3.15.
        const std::size_t emit_index = static_cast<std::size_t>(leg);
        if (intent.requested_mode == RobotMode::WALK && swingPlanCommitEnabled()
            && have_last_emitted_target_[emit_index] && dt_s > 0.0) {
            const Vec3 step = target - last_emitted_target_[emit_index];
            const double step_mag = vecNorm(step);
            const Vec3 coxa = geometry_.legGeometry[leg].bodyCoxaOffset;
            const double radius = std::max(vecNorm(target - coxa), kMinFootLeverArmM);
            const double envelope_mps =
                kFootEnvelopeNoLoadFraction * hexapod_dynamics::kServoNoLoadSpeedRadPerSec * radius;
            const double allowed =
                std::max(kPlannerSpeedMargin * vecNorm(target_vel), envelope_mps) * dt_s;
            if (step_mag > allowed && step_mag > 1e-12) {
                const double scale = allowed / step_mag;
                target = last_emitted_target_[emit_index] + step * scale;
                target_vel = target_vel * scale;
            }
        }
        last_emitted_target_[emit_index] = target;
        have_last_emitted_target_[emit_index] = true;

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
        maybeRecordR2SwingPlannerDump(
            leg,
            r2_swing_this_leg,
            gait,
            r2_sw,
            kinematic_twist,
            kinematic_est,
            r2_planned_pre_rot,
            target);
        if (r2_swing_this_leg) {
            const Vec3 coxa = geometry_.legGeometry[leg].bodyCoxaOffset;
            last_r2_swing_decomp_.valid = true;
            last_r2_swing_decomp_.anchor = r2_sw.anchor;
            last_r2_swing_decomp_.stance_end = r2_sw.stance_end;
            last_r2_swing_decomp_.v_liftoff_body = r2_sw.v_liftoff_body;
            last_r2_swing_decomp_.tau01 = r2_sw.tau01;
            last_r2_swing_decomp_.swing_span = r2_sw.swing_span;
            last_r2_swing_decomp_.f_hz = r2_sw.f_hz;
            last_r2_swing_decomp_.step_length_m = r2_sw.step_length_m;
            last_r2_swing_decomp_.swing_height_m = r2_sw.swing_height_m;
            last_r2_swing_decomp_.cmd_accel_body_x_mps2 = r2_sw.cmd_accel_body_x_mps2;
            last_r2_swing_decomp_.cmd_accel_body_y_mps2 = r2_sw.cmd_accel_body_y_mps2;
            last_r2_swing_decomp_.stance_lookahead_s = r2_sw.stance_lookahead_s;
            last_r2_swing_decomp_.static_stability_margin_m = r2_sw.static_stability_margin_m;
            last_r2_swing_decomp_.swing_time_ease_01 = r2_sw.swing_time_ease_01;
            last_r2_swing_decomp_.kinematic_twist = kinematic_twist;
            last_r2_swing_decomp_.est_valid = kinematic_est.valid;
            last_r2_swing_decomp_.est_has_body_twist = kinematic_est.has_body_twist_state;
            if (kinematic_est.has_body_twist_state) {
                last_r2_swing_decomp_.est_linear_mps = kinematic_est.body_twist_state.body_trans_mps.raw();
                last_r2_swing_decomp_.est_angular_radps = kinematic_est.body_twist_state.twist_vel_radps.raw();
            }
            last_r2_swing_decomp_.planned_pre_rot = r2_planned_pre_rot;
            last_r2_swing_decomp_.after_terrain = r2_untilted;
            last_r2_swing_decomp_.origin_rot = body_rotation * r2_untilted;
            last_r2_swing_decomp_.coxa_rot = coxa + (body_rotation * (r2_untilted - coxa));
            last_r2_swing_decomp_.coxa = coxa;
            last_r2_swing_decomp_.target_clamped = target;
            last_r2_swing_decomp_.foothold_nominal = r2_foothold.nominal_body;
            last_r2_swing_decomp_.capture_body = r2_foothold.capture_body;
            last_r2_swing_decomp_.foothold_final = r2_foothold.final_body;
            last_r2_swing_decomp_.terrain_xy_delta = r2_terrain_xy_delta;
            last_r2_swing_decomp_.capture_limit_m = r2_foothold.capture_limit_m;
            last_r2_swing_decomp_.clamp_dxy = std::hypot(
                last_r2_swing_decomp_.origin_rot.x - target.x,
                last_r2_swing_decomp_.origin_rot.y - target.y);
            last_r2_swing_decomp_.roll_rad = pose.roll_rad;
            last_r2_swing_decomp_.pitch_rad = pose.pitch_rad;
            last_r2_swing_decomp_.yaw_rad = pose.yaw_rad;
        }
    }

    return out;
}
