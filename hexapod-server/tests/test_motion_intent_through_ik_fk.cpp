#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "foot_reachability.hpp"
#include "gait_scheduler.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"
#include "physics_sim_protocol.hpp"
#include "servo_dynamics_clamp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool finiteJointTargets(const JointTargets& targets) {
    for (const auto& leg : targets.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool fkMountConventionMatchesPhysicsContactSphere() {
    const HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    LegFK fk{};
    for (int leg_index = 0; leg_index < kNumLegs; ++leg_index) {
        const LegGeometry& leg = geometry.legGeometry[leg_index];
        LegState mechanical{};
        mechanical.joint_state[COXA].pos_rad =
            AngleRad{physics_sim::kWireZeroCoxaMechanicalRad};
        mechanical.joint_state[FEMUR].pos_rad =
            AngleRad{physics_sim::kWireZeroFemurMechanicalRad};
        mechanical.joint_state[TIBIA].pos_rad =
            AngleRad{physics_sim::kWireZeroTibiaMechanicalRad};
        const LegState servo = leg.servo.toServoAngles(mechanical);
        const Vec3 actual = fk.footInBodyFrame(servo, leg).pos_body_m.raw();

        // The simulator radial axis is (sin(mount),0,cos(mount)).  The
        // canonical bridge map (-z,x,y) makes that
        // (-cos(mount),sin(mount),0) in server body coordinates.
        const Vec3 radial_axis{
            -std::cos(leg.mountAngle.value),
            std::sin(leg.mountAngle.value),
            0.0};
        const double q2 = physics_sim::kWireZeroFemurMechanicalRad;
        const double q3 = physics_sim::kWireZeroTibiaMechanicalRad;
        const double radial = leg.coxaLength.value
            + leg.femurLength.value * std::cos(q2)
            + leg.tibiaLength.value * std::cos(q2 + q3);
        const double z = leg.femurLength.value * std::sin(q2)
            + leg.tibiaLength.value * std::sin(q2 + q3);
        const Vec3 expected = leg.bodyCoxaOffset + radial_axis * radial + Vec3{0.0, 0.0, z};
        const double error = vecNorm(actual - expected);
        if (error > 1.0e-10) {
            std::cerr << "FAIL: FK/contact-sphere rest mapping leg=" << leg_index
                      << " error_m=" << error
                      << " actual=(" << actual.x << ',' << actual.y << ',' << actual.z << ')'
                      << " expected=(" << expected.x << ',' << expected.y << ',' << expected.z
                      << ")\n";
            return false;
        }
    }
    return true;
}

struct IkCandidate {
    LegState joint{};
    double knee_height{0.0};
};

bool jointStatesClose(const LegState& lhs, const LegState& rhs, double eps) {
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        if (std::abs(lhs.joint_state[joint].pos_rad.value - rhs.joint_state[joint].pos_rad.value) > eps) {
            return false;
        }
    }
    return true;
}

IkCandidate buildIkCandidate(double q1,
                             double Dclamped,
                             double s3,
                             double rhoSolved,
                             double zSolved,
                             const LegGeometry& leg)
{
    IkCandidate candidate{};
    const double q3 = std::atan2(s3, Dclamped);
    candidate.joint.joint_state[0].pos_rad = AngleRad{q1};
    candidate.joint.joint_state[1].pos_rad = AngleRad{
        std::atan2(zSolved, rhoSolved) -
        std::atan2(leg.tibiaLength.value * std::sin(q3),
                   leg.femurLength.value + leg.tibiaLength.value * std::cos(q3))};
    candidate.joint.joint_state[2].pos_rad = AngleRad{q3};
    candidate.knee_height = leg.femurLength.value * std::sin(candidate.joint.joint_state[1].pos_rad.value);
    return candidate;
}

std::array<IkCandidate, 2> computeIkCandidates(const FootTarget& foot,
                                               const LegGeometry& leg)
{
    const Vec3 relativeToCoxa = foot.pos_body_m - leg.bodyCoxaOffset;
    const Mat3 R_leg = legFromBodyFrame(leg);
    const Vec3 footLeg = R_leg * relativeToCoxa;

    const double x = footLeg.x;
    const double y = footLeg.y;
    const double z = footLeg.z;
    const double q1 = std::atan2(y, x);
    const double r = std::hypot(x, y);
    const double rho = r - leg.coxaLength.value;
    const double d = std::hypot(rho, z);
    const double minReach = std::fabs(leg.femurLength.value - leg.tibiaLength.value);
    const double maxReach = leg.femurLength.value + leg.tibiaLength.value;
    const double clampedD = std::clamp(d, minReach, maxReach);
    const double reachScale = (d > 1e-9) ? (clampedD / d) : 1.0;
    const double rhoSolved = rho * reachScale;
    const double zSolved = z * reachScale;
    const double D =
        (rhoSolved * rhoSolved + zSolved * zSolved - leg.femurLength.value * leg.femurLength.value -
         leg.tibiaLength.value * leg.tibiaLength.value) /
        (2.0 * leg.femurLength.value * leg.tibiaLength.value);
    const double Dclamped = std::clamp(D, -1.0, 1.0);
    const double s3abs = std::sqrt(std::max(0.0, 1.0 - Dclamped * Dclamped));

    return {
        buildIkCandidate(q1, Dclamped, -s3abs, rhoSolved, zSolved, leg),
        buildIkCandidate(q1, Dclamped, +s3abs, rhoSolved, zSolved, leg),
    };
}

const IkCandidate& chooseKneeUpCandidate(const std::array<IkCandidate, 2>& candidates) {
    return (candidates[0].knee_height >= candidates[1].knee_height) ? candidates[0] : candidates[1];
}

bool ikTracksKneeUpBranch(const HexapodGeometry& geometry,
                          LegIK& ik,
                          LegFK& fk,
                          const SafetyState& safety,
                          const LegState& seed_joint)
{
    RobotState est{};
    est.timestamp_us = now_us();

    LegTargets body_targets{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        est.leg_states[leg] = geometry.legGeometry[leg].servo.toServoAngles(seed_joint);
        body_targets.feet[leg] = fk.footInBodyFrame(
            est.leg_states[leg], geometry.legGeometry[leg]);
    }

    const JointTargets first = ik.solve(est, body_targets, safety);
    const LegState first_joint = geometry.legGeometry[0].servo.toJointAngles(first.leg_states[0]);
    const IkCandidate& expected_first =
        chooseKneeUpCandidate(computeIkCandidates(body_targets.feet[0], geometry.legGeometry[0]));
    if (!expect(jointStatesClose(first_joint, expected_first.joint, 1e-9),
                "IK should choose the knee-up branch for the seed pose")) {
        return false;
    }

    est.leg_states = first.leg_states;

    LegTargets nudged_targets = body_targets;
    nudged_targets.feet[0].pos_body_m.x += 0.002;
    nudged_targets.feet[0].pos_body_m.y -= 0.001;

    const JointTargets second = ik.solve(est, nudged_targets, safety);
    const LegState second_joint = geometry.legGeometry[0].servo.toJointAngles(second.leg_states[0]);
    const IkCandidate& expected_second =
        chooseKneeUpCandidate(computeIkCandidates(nudged_targets.feet[0], geometry.legGeometry[0]));
    if (!expect(jointStatesClose(second_joint, expected_second.joint, 1e-9),
                "IK should keep the knee-up branch after a small foot move")) {
        return false;
    }

    const double knee_delta =
        std::abs(shortestAngleDeltaRad(first_joint.joint_state[2].pos_rad.value,
                                       second_joint.joint_state[2].pos_rad.value));
    if (!expect(knee_delta < 0.35, "IK should preserve the knee-up branch across a small foot move")) {
        return false;
    }

    return true;
}

bool gaitSchedulerRespondsToWalkIntent() {
    GaitScheduler gait;
    RobotState est{};
    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.cmd_vx_mps = LinearRateMps{0.35};
    walk.timestamp_us = TimePointUs{1'000'000};

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    const BodyTwist walk_twist =
        rawLocomotionTwistFromIntent(walk, planarMotionCommand(walk));
    const GaitState entry = gait.update(est, walk, safety, walk_twist);
    if (!expect(std::all_of(entry.in_stance.begin(), entry.in_stance.end(), [](const bool stance) { return stance; }),
                "walk entry should begin with all legs in stance")) {
        return false;
    }

    GaitState advanced{};
    for (int i = 1; i <= 100; ++i) {
        walk.timestamp_us = TimePointUs{static_cast<uint64_t>(1'000'000 + i * 4'000)};
        advanced = gait.update(est, walk, safety, walk_twist);
    }

    const bool leg0_leg1_offset = std::fabs(advanced.phase[0] - advanced.phase[1]) > 0.25;
    const bool stride_active = advanced.stride_phase_rate_hz.value >= 0.5;
    const bool timestamp_set = !advanced.timestamp_us.isZero();

    return expect(stride_active, "walk intent should produce positive stride phase rate") &&
           expect(leg0_leg1_offset, "tripod gait should offset neighboring leg phases") &&
           expect(timestamp_set, "gait update should stamp output time");
}

bool bodyControllerUsesGaitState() {
    BodyController body;
    RobotState est{};
    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;

    SafetyState safety{};
    safety.inhibit_motion = false;

    GaitState gait{};
    gait.in_stance.fill(true);
    gait.phase.fill(0.0);
    gait.in_stance[1] = false;
    gait.phase[1] = 0.75;

    const BodyTwist body_twist = rawLocomotionTwistFromIntent(walk, planarMotionCommand(walk));
    const LegTargets targets = body.update(est, walk, gait, safety, body_twist);

    const double leg0_x = targets.feet[0].pos_body_m.x;
    const double leg1_x = targets.feet[1].pos_body_m.x;
    const double leg1_z = targets.feet[1].pos_body_m.z;

    return expect(leg0_x != leg1_x, "stance and swing legs should get different x placement") &&
           expect(leg1_z > targets.feet[0].pos_body_m.z, "swing leg should receive swing height lift");
}

bool ikChoosesKneeUpBranch() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegIK ik(geometry);
    LegFK fk;

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.leg_enabled.fill(true);

    LegState seed_negative{};
    seed_negative.joint_state[0].pos_rad = AngleRad{0.15};
    seed_negative.joint_state[1].pos_rad = AngleRad{-0.25};
    seed_negative.joint_state[2].pos_rad = AngleRad{-0.85};

    LegState seed_positive = seed_negative;
    seed_positive.joint_state[2].pos_rad = AngleRad{0.85};

    return ikTracksKneeUpBranch(geometry, ik, fk, safety, seed_negative) &&
           ikTracksKneeUpBranch(geometry, ik, fk, safety, seed_positive);
}

bool ikFkChainTracksBodyTargets() {
    const HexapodGeometry geometry = defaultHexapodGeometry();

    LegIK ik(geometry);
    LegFK fk;

    RobotState est{};
    est.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.leg_enabled.fill(true);

    LegTargets body_targets{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState known_joint{};
        known_joint.joint_state[0].pos_rad = AngleRad{0.15};
        known_joint.joint_state[1].pos_rad = AngleRad{-0.25};
        known_joint.joint_state[2].pos_rad = AngleRad{-0.85};
        body_targets.feet[leg] = fk.footInBodyFrame(
            geometry.legGeometry[leg].servo.toServoAngles(known_joint),
            geometry.legGeometry[leg]);
    }

    const JointTargets joints = ik.solve(est, body_targets, safety);

    constexpr int kReferenceLeg = 0;
    const FootTarget fk_body = fk.footInBodyFrame(
        joints.leg_states[kReferenceLeg], geometry.legGeometry[kReferenceLeg]);
    const Vec3 diff = fk_body.pos_body_m - body_targets.feet[kReferenceLeg].pos_body_m;
    const double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    if (!expect(err < 0.04, "ik/fk round trip should stay close to body target")) {
        return false;
    }

    return expect(finiteJointTargets(joints), "ik output should stay finite for reachable body targets");
}

bool ikStaysContinuousAcrossSmallFootMoves() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegIK ik(geometry);

    RobotState est{};
    est.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.leg_enabled.fill(true);

    LegTargets body_targets{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState known_joint{};
        known_joint.joint_state[0].pos_rad = AngleRad{0.10};
        known_joint.joint_state[1].pos_rad = AngleRad{-0.20};
        known_joint.joint_state[2].pos_rad = AngleRad{-0.75};
        body_targets.feet[leg] = LegFK().footInBodyFrame(
            geometry.legGeometry[leg].servo.toServoAngles(known_joint),
            geometry.legGeometry[leg]);
    }

    const JointTargets first = ik.solve(est, body_targets, safety);
    est.leg_states = first.leg_states;

    LegTargets body_targets2 = body_targets;
    body_targets2.feet[0].pos_body_m.x += 0.002;
    body_targets2.feet[0].pos_body_m.y -= 0.001;

    const JointTargets second = ik.solve(est, body_targets2, safety);

    const LegState first_joint = geometry.legGeometry[0].servo.toJointAngles(first.leg_states[0]);
    const LegState second_joint = geometry.legGeometry[0].servo.toJointAngles(second.leg_states[0]);
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        const double delta =
            std::abs(second_joint.joint_state[joint].pos_rad.value - first_joint.joint_state[joint].pos_rad.value);
        if (!expect(delta < 0.35, "IK should stay continuous across a small foot move")) {
            return false;
        }
    }
    return true;
}

bool fkUndoesServoCalibrationForEveryLeg() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    LegState mechanical{};
    mechanical.joint_state[COXA].pos_rad = AngleRad{0.17};
    mechanical.joint_state[FEMUR].pos_rad = AngleRad{-0.42};
    mechanical.joint_state[TIBIA].pos_rad = AngleRad{-0.91};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& calibrated_leg = geometry.legGeometry[leg];
        LegGeometry identity_leg = calibrated_leg;
        identity_leg.servo = ServoCalibration{};
        const Vec3 expected = fk.footInBodyFrame(mechanical, identity_leg).pos_body_m.raw();
        const LegState servo = calibrated_leg.servo.toServoAngles(mechanical);
        const Vec3 actual = fk.footInBodyFrame(servo, calibrated_leg).pos_body_m.raw();
        if (!expect(vecNorm(actual - expected) < 1.0e-12,
                    "FK should undo each leg's servo sign and attachment offsets")) {
            return false;
        }
    }
    return true;
}

bool controlPipelineProducesStableOutputs() {
    ControlPipeline pipeline;

    RobotState estimated{};
    estimated.timestamp_us = now_us();

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.active_fault = FaultCode::NONE;

    const PipelineStepResult result = pipeline.runStep(estimated, walk_intent, safety, true, 99);

    return expect(result.status.active_mode == RobotMode::WALK, "pipeline should preserve walk mode") &&
           expect(result.status.loop_counter == 99, "pipeline should preserve loop counter") &&
           expect(finiteJointTargets(result.joint_targets), "pipeline joint targets should be finite");
}

Vec3 serverBody(const Vec3& canonical) {
    return canonical;
}

bool ikFkStrokeRoundtripMatchesCartesianOpposition() {
    BodyController body{};
    LegIK ik{};
    LegFK fk{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.twist.body_trans_m.z = 0.12;
    intent.timestamp_us = TimePointUs{1'000'000};

    RobotState planted{};
    planted.foot_contacts[0] = true;
    planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;

    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.step_length_m = 0.06;
    gait.swing_height_m = 0.03;
    gait.phase[0] = 0.10;
    gait.in_stance[0] = true;

    const BodyTwist cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
    const HexapodGeometry geometry = defaultHexapodGeometry();
    constexpr double dt_s = 0.005;

    LegTargets first = body.update(planted, intent, gait, safety, cmd);
    JointTargets first_joints = ik.solve(planted, first, safety);
    Vec3 last_cart = first.feet[0].pos_body_m;
    Vec3 last_fk = fk.footInBodyFrame(first_joints.leg_states[0], geometry.legGeometry[0]).pos_body_m.raw();
    double cart_sum = 0.0;
    double fk_sum = 0.0;
    int samples = 0;
    bool phase_ok = true;
    bool reach_hit = false;
    for (int frame = 1; frame < 40; ++frame) {
        intent.timestamp_us.value += 5'000;
        gait.phase[0] = 0.10 + static_cast<double>(frame) * dt_s;
        phase_ok = phase_ok && gait.phase[0] > 0.05 && gait.phase[0] < 0.45;
        const LegTargets now = body.update(planted, intent, gait, safety, cmd);
        const JointTargets joints = ik.solve(planted, now, safety);
        reach_hit = reach_hit || ik.lastReachClampHit()[0];
        const Vec3 cart = now.feet[0].pos_body_m;
        const Vec3 fk_pos =
            fk.footInBodyFrame(joints.leg_states[0], geometry.legGeometry[0]).pos_body_m.raw();
        const Vec3 cart_step = serverBody(cart) - serverBody(last_cart);
        const Vec3 fk_step = serverBody(fk_pos) - serverBody(last_fk);
        cart_sum += -(cmd.linear_mps.x * cart_step.x + cmd.linear_mps.y * cart_step.y) / (0.12 * dt_s);
        fk_sum += -(cmd.linear_mps.x * fk_step.x + cmd.linear_mps.y * fk_step.y) / (0.12 * dt_s);
        last_cart = cart;
        last_fk = fk_pos;
        ++samples;
    }
    const double mean_cart = cart_sum / static_cast<double>(samples);
    const double mean_fk = fk_sum / static_cast<double>(samples);
    return expect(phase_ok, "identity stroke should keep φ in (0.05, 0.45)") &&
           expect(!reach_hit, "reachable identity stroke should not hit IK reach clamp") &&
           expect(std::abs(mean_cart - 0.12) <= 0.15 * 0.12,
                  "Cartesian opposition should match |v| within 15%") &&
           expect(std::abs(mean_fk - mean_cart) <= 0.15 * std::max(std::abs(mean_cart), 0.12),
                  "IK+FK planar opposition should match Cartesian within 15%");
}

bool ikReachClampHitsAndShortensTowardCoxa() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    const LegGeometry& leg = geometry.legGeometry[0];
    LegIK ik{geometry};
    LegFK fk{};
    SafetyState safety{};
    safety.inhibit_motion = false;

    const double max_reach = leg.femurLength.value + leg.tibiaLength.value;
    const Vec3 request = leg.bodyCoxaOffset + Vec3{0.40, 0.0, -0.14};
    const double request_d = foot_reachability::femurPlaneDistanceM(leg, request);
    if (!expect(request_d > max_reach + 0.05, "reach-clamp fixture should be outside Lf+Lt")) {
        return false;
    }

    LegTargets targets{};
    for (int i = 0; i < kNumLegs; ++i) {
        targets.feet[i].pos_body_m = computeNominalStance(geometry, 0.12)[static_cast<std::size_t>(i)];
    }
    targets.feet[0].pos_body_m = request;
    RobotState est{};
    const JointTargets joints = ik.solve(est, targets, safety);
    const Vec3 fk_pos = fk.footInBodyFrame(joints.leg_states[0], leg).pos_body_m.raw();
    const double fk_d = foot_reachability::femurPlaneDistanceM(leg, fk_pos);

    const Mat3 R_leg = legFromBodyFrame(leg);
    const Vec3 req_leg = R_leg * (request - leg.bodyCoxaOffset);
    const Vec3 fk_leg = R_leg * (fk_pos - leg.bodyCoxaOffset);
    const double req_rho = std::hypot(req_leg.x, req_leg.y) - leg.coxaLength.value;
    const double fk_rho = std::hypot(fk_leg.x, fk_leg.y) - leg.coxaLength.value;
    const double req_plane = std::hypot(req_rho, req_leg.z);
    const double fk_plane = std::hypot(fk_rho, fk_leg.z);
    const double dir_dot =
        (req_rho * fk_rho + req_leg.z * fk_leg.z) / std::max(req_plane * fk_plane, 1e-12);

    return expect(ik.lastReachClampHit()[0], "out-of-reach target must set the IK reach-clamp hit") &&
           expect(fk_d <= max_reach + 1e-6, "IK reach clamp must bring d onto the annulus") &&
           expect(fk_plane < req_plane - 0.05, "reach clamp must shorten toward the coxa") &&
           expect(dir_dot > 0.98, "reach clamp should scale along the femur-plane ray, not along v");
}

bool leanedStanceClampKeepsPlanarStrokeAndIkReach() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    const LegGeometry& leg = geometry.legGeometry[0];
    constexpr double inset = 0.004;
    const Vec3 last = computeNominalStance(geometry, 0.14)[0];
    if (!expect(foot_reachability::footInReachAnnulus(leg, last, inset),
                "nominal 0.14 m stance should be inside the annulus")) {
        return false;
    }
    const Vec3 desired = last + Vec3{-0.10, 0.0, -0.06};
    if (!expect(!foot_reachability::footInReachAnnulus(leg, desired, inset),
                "leaned+stroked desired should leave the annulus")) {
        return false;
    }
    const Vec3 radial = foot_reachability::clampFootPositionBody(leg, desired, inset);
    const Vec3 stroke = foot_reachability::clampFootPositionAlongStroke(leg, &last, desired, inset).pos_body_m;
    const double stroke_xy_err = std::hypot(stroke.x - desired.x, stroke.y - desired.y);
    const double radial_xy_err = std::hypot(radial.x - desired.x, radial.y - desired.y);

    LegIK ik{geometry};
    LegFK fk{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    LegTargets targets{};
    for (int i = 0; i < kNumLegs; ++i) {
        targets.feet[i].pos_body_m = computeNominalStance(geometry, 0.14)[static_cast<std::size_t>(i)];
    }
    targets.feet[0].pos_body_m = stroke;
    RobotState est{};
    const JointTargets joints = ik.solve(est, targets, safety);
    const bool projector_ik_hit = ik.lastReachClampHit()[0];
    const Vec3 fk_pos = fk.footInBodyFrame(joints.leg_states[0], leg).pos_body_m.raw();
    const double fk_xy_err = std::hypot(fk_pos.x - stroke.x, fk_pos.y - stroke.y);

    BodyController body{};
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.twist.body_trans_m.z = 0.14;
    intent.timestamp_us = TimePointUs{1'000'000};
    RobotState planted{};
    planted.foot_contacts.fill(true);
    for (auto& fusion : planted.foot_contact_fusion) {
        fusion.phase = ContactPhase::ConfirmedStance;
    }
    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{0.68};
    gait.step_length_m = 0.06;
    gait.swing_height_m = 0.03;
    gait.static_stability_margin_m = 0.05;
    gait.phase.fill(0.10);
    gait.in_stance.fill(true);
    const BodyTwist cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
    bool cart_in_annulus = true;
    bool ik_reach_hit = false;
    int workspace_xy_hits = 0;
    double cart_sum = 0.0;
    int cart_samples = 0;
    LegTargets prev = body.update(planted, intent, gait, safety, cmd);
    Vec3 last_cart = prev.feet[0].pos_body_m;
    for (int frame = 1; frame < 60; ++frame) {
        intent.timestamp_us.value += 5'000;
        gait.phase[0] = 0.10 + static_cast<double>(frame) * 0.005 * 0.68;
        const LegTargets now = body.update(planted, intent, gait, safety, cmd);
        (void)ik.solve(planted, now, safety);
        ik_reach_hit = ik_reach_hit || ik.lastReachClampHit()[0];
        workspace_xy_hits += body.lastWorkspaceXyHit()[0] ? 1 : 0;
        cart_in_annulus =
            cart_in_annulus &&
            foot_reachability::footInReachAnnulus(leg, now.feet[0].pos_body_m, inset);
        const Vec3 cart = now.feet[0].pos_body_m;
        const Vec3 step = serverBody(cart) - serverBody(last_cart);
        cart_sum += -(cmd.linear_mps.x * step.x + cmd.linear_mps.y * step.y) / (0.12 * 0.005);
        last_cart = cart;
        ++cart_samples;
    }
    const double mean_cart = cart_sum / static_cast<double>(cart_samples);
    const double workspace_xy_hit_fraction =
        static_cast<double>(workspace_xy_hits) / static_cast<double>(cart_samples);

    return expect(foot_reachability::footInReachAnnulus(leg, stroke, inset),
                  "stroke projector must land on the annulus") &&
           expect(stroke_xy_err + 1e-3 < radial_xy_err,
                  "stroke projector must keep more planar stroke than coxa-radial clamp") &&
           expect(!projector_ik_hit,
                  "IK must not coxa-scale a stroke-projected stance target") &&
           expect(fk_xy_err <= 0.005, "IK+FK planar should track the stroke-projected target") &&
           expect(cart_in_annulus, "leaned BodyController stance targets must stay in the annulus") &&
           expect(!ik_reach_hit, "leaned BodyController stance must not hit IK reach clamp") &&
           expect(workspace_xy_hit_fraction < 0.05,
                  "coxa-centered lean must not skate stance XY off the annulus") &&
           expect(mean_cart > 0.07, "leaned stance Cartesian opposition should stay well above 50%");
}

bool servoSlewDoesNotBiteStanceStepAndHitsLargeJump() {
    HexapodGeometry geometry = defaultHexapodGeometry();
    for (LegGeometry& leg : geometry.legGeometry) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            leg.servoDynamics[joint].positive_direction.vmax_radps = 8.0;
            leg.servoDynamics[joint].negative_direction.vmax_radps = 8.0;
        }
    }

    BodyController body{};
    LegIK ik{geometry};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.twist.body_trans_m.z = 0.12;
    intent.timestamp_us = TimePointUs{1'000'000};
    RobotState planted{};
    planted.foot_contacts[0] = true;
    planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.step_length_m = 0.06;
    gait.swing_height_m = 0.03;
    gait.phase[0] = 0.10;
    gait.in_stance[0] = true;
    const BodyTwist cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
    const LegTargets first = body.update(planted, intent, gait, safety, cmd);
    const JointTargets first_joints = ik.solve(planted, first, safety);
    intent.timestamp_us.value += 5'000;
    gait.phase[0] = 0.105;
    const LegTargets second = body.update(planted, intent, gait, safety, cmd);
    const JointTargets second_joints = ik.solve(planted, second, safety);
    const ServoDynamicsClampResult stance = clampJointTargetsToServoDynamics(
        first_joints, second_joints, geometry, 0.005);

    JointTargets jumped = first_joints;
    jumped.leg_states[0].joint_state[0].pos_rad =
        AngleRad{first_joints.leg_states[0].joint_state[0].pos_rad.value + 1.0};
    const ServoDynamicsClampResult large = clampJointTargetsToServoDynamics(
        first_joints, jumped, geometry, 0.005);

    return expect(!stance.leg_limited[0],
                  "8 rad/s slew must not bite a 0.12 m/s 5 ms stance step") &&
           expect(large.leg_limited[0], "8 rad/s slew must hit a 1 rad joint jump");
}

} // namespace

int main() {
    if (!fkMountConventionMatchesPhysicsContactSphere()) {
        return EXIT_FAILURE;
    }
    if (!gaitSchedulerRespondsToWalkIntent()) {
        return EXIT_FAILURE;
    }
    if (!bodyControllerUsesGaitState()) {
        return EXIT_FAILURE;
    }
    if (!ikChoosesKneeUpBranch()) {
        return EXIT_FAILURE;
    }
    if (!ikFkChainTracksBodyTargets()) {
        return EXIT_FAILURE;
    }
    if (!ikStaysContinuousAcrossSmallFootMoves()) {
        return EXIT_FAILURE;
    }
    if (!fkUndoesServoCalibrationForEveryLeg()) {
        return EXIT_FAILURE;
    }
    if (!controlPipelineProducesStableOutputs()) {
        return EXIT_FAILURE;
    }
    if (!ikFkStrokeRoundtripMatchesCartesianOpposition()) {
        return EXIT_FAILURE;
    }
    if (!ikReachClampHitsAndShortensTowardCoxa()) {
        return EXIT_FAILURE;
    }
    if (!leanedStanceClampKeepsPlanarStrokeAndIkReach()) {
        return EXIT_FAILURE;
    }
    if (!servoSlewDoesNotBiteStanceStepAndHitsLargeJump()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
