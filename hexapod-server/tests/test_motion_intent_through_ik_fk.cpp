#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "gait_scheduler.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"
#include "stability_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

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

double wrappedPositiveDelta(double from, double to) {
    const double raw = std::fmod((to - from) + 1.0, 1.0);
    return raw < 0.0 ? raw + 1.0 : raw;
}

Vec3 mirrorAcrossSagittal(const Vec3& v) {
    return Vec3{v.x, -v.y, v.z};
}

MotionIntent mirroredLeftRightIntent(const MotionIntent& source) {
    MotionIntent mirrored = source;
    mirrored.heading_rad = AngleRad{-source.heading_rad.value};
    mirrored.body_pose_setpoint.body_trans_m = mirrorAcrossSagittal(source.body_pose_setpoint.body_trans_m);
    mirrored.body_pose_setpoint.body_trans_mps = mirrorAcrossSagittal(source.body_pose_setpoint.body_trans_mps);
    mirrored.body_pose_setpoint.angular_velocity_radps = Vec3{
        -source.body_pose_setpoint.angular_velocity_radps.x,
        source.body_pose_setpoint.angular_velocity_radps.y,
        -source.body_pose_setpoint.angular_velocity_radps.z};
    mirrored.body_pose_setpoint.orientation_rad = Vec3{
        -source.body_pose_setpoint.orientation_rad.x,
        source.body_pose_setpoint.orientation_rad.y,
        -source.body_pose_setpoint.orientation_rad.z};
    return mirrored;
}

bool plannerBodyIkRemainLeftRightMirrored() {
    constexpr std::array<std::pair<int, int>, 3> kLeftRightLegPairs{
        std::pair<int, int>{0, 1},
        std::pair<int, int>{2, 3},
        std::pair<int, int>{4, 5}};
    constexpr double kPhaseTolerance = 1e-6;
    constexpr double kPositionToleranceM = 8e-3;
    constexpr double kVelocityToleranceMps = 6e-2;
    constexpr double kJointToleranceRad = 6e-2;

    const HexapodGeometry geometry = defaultHexapodGeometry();
    BodyController body;
    LegIK ik(geometry);

    RobotState estimate{};
    estimate.timestamp_us = now_us();
    estimate.foot_contacts = {true, true, true, true, true, true};
    estimate.has_measured_body_pose_state = true;
    estimate.body_pose_state.orientation_rad = Vec3{0.04, -0.03, 0.02};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState joint{};
        joint.joint_state[0].pos_rad = AngleRad{0.10};
        joint.joint_state[1].pos_rad = AngleRad{-0.55};
        joint.joint_state[2].pos_rad = AngleRad{-0.95};
        estimate.leg_states[leg] = geometry.legGeometry[leg].servo.toServoAngles(joint);
    }

    RobotState mirrored_estimate = estimate;
    mirrored_estimate.body_pose_state.orientation_rad =
        Vec3{-estimate.body_pose_state.orientation_rad.x,
             estimate.body_pose_state.orientation_rad.y,
             -estimate.body_pose_state.orientation_rad.z};

    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.heading_rad = AngleRad{0.38};
    intent.body_pose_setpoint.body_trans_m = Vec3{0.010, 0.018, 0.205};
    intent.body_pose_setpoint.body_trans_mps = Vec3{0.12, -0.05, 0.0};
    intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.02, -0.01, 0.35};
    intent.body_pose_setpoint.orientation_rad = Vec3{0.08, -0.04, 0.16};

    const MotionIntent mirrored_intent = mirroredLeftRightIntent(intent);

    RuntimeGaitPolicy policy{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        policy.per_leg[leg].step_length_m = LengthM{0.065 + 0.004 * leg};
        policy.per_leg[leg].swing_height_m = LengthM{0.028 + 0.001 * leg};
        policy.per_leg[leg].duty_cycle = 0.58;
    }

    GaitState gait{};
    gait.stride_phase_rate_hz = FrequencyHz{1.35};
    gait.phase = {0.16, 0.61, 0.34, 0.86, 0.08, 0.57};
    gait.in_stance = {true, false, true, false, false, true};

    GaitState mirrored_gait = gait;
    for (const auto& [right, left] : kLeftRightLegPairs) {
        mirrored_gait.phase[left] = gait.phase[right];
        mirrored_gait.phase[right] = gait.phase[left];
        mirrored_gait.in_stance[left] = gait.in_stance[right];
        mirrored_gait.in_stance[right] = gait.in_stance[left];
    }

    for (const auto& [right, left] : kLeftRightLegPairs) {
        if (!expect(std::abs(gait.phase[right] - mirrored_gait.phase[left]) <= kPhaseTolerance,
                    "mirrored gait must preserve left/right phase timing")) {
            return false;
        }
        if (!expect(gait.in_stance[right] == mirrored_gait.in_stance[left],
                    "mirrored gait must preserve left/right stance timing")) {
            return false;
        }
    }

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.support_contact_count = kNumLegs;
    safety.stability_margin_m = 0.03;
    safety.leg_enabled.fill(true);

    const LegTargets body_targets = body.update(estimate, intent, gait, policy, safety);
    const LegTargets mirrored_body_targets =
        body.update(mirrored_estimate, mirrored_intent, mirrored_gait, policy, safety);

    for (const auto& [right, left] : kLeftRightLegPairs) {
        const Vec3 mirrored_position_error =
            mirrorAcrossSagittal(body_targets.feet[right].pos_body_m) -
            mirrored_body_targets.feet[left].pos_body_m;
        const double position_error_norm = vecNorm(mirrored_position_error);
        if (!expect(position_error_norm < kPositionToleranceM,
                    "body-controller left/right mirrored foot position mismatch")) {
            return false;
        }

        const Vec3 mirrored_velocity_error =
            mirrorAcrossSagittal(body_targets.feet[right].vel_body_mps) -
            mirrored_body_targets.feet[left].vel_body_mps;
        const double velocity_error_norm = vecNorm(mirrored_velocity_error);
        if (!expect(velocity_error_norm < kVelocityToleranceMps,
                    "body-controller left/right mirrored foot velocity mismatch")) {
            return false;
        }
    }

    const JointTargets joints = ik.solve(estimate, body_targets, safety);
    const JointTargets mirrored_joints = ik.solve(mirrored_estimate, mirrored_body_targets, safety);

    for (const auto& [right, left] : kLeftRightLegPairs) {
        const LegState right_joint_frame =
            geometry.legGeometry[right].servo.toJointAngles(joints.leg_states[right]);
        const LegState left_joint_frame =
            geometry.legGeometry[left].servo.toJointAngles(mirrored_joints.leg_states[left]);

        const double coxa_sum = std::abs(right_joint_frame.joint_state[0].pos_rad.value +
                                         left_joint_frame.joint_state[0].pos_rad.value);
        const double femur_diff = std::abs(right_joint_frame.joint_state[1].pos_rad.value -
                                           left_joint_frame.joint_state[1].pos_rad.value);
        const double tibia_diff = std::abs(right_joint_frame.joint_state[2].pos_rad.value -
                                           left_joint_frame.joint_state[2].pos_rad.value);

        if (!expect(coxa_sum < kJointToleranceRad,
                    "ik mirrored coxa joints should have opposite signs")) {
            return false;
        }
        if (!expect(femur_diff < kJointToleranceRad,
                    "ik mirrored femur joints should match")) {
            return false;
        }
        if (!expect(tibia_diff < kJointToleranceRad,
                    "ik mirrored tibia joints should match")) {
            return false;
        }
    }

    return true;
}

bool gaitSchedulerRespondsToWalkIntent() {
    GaitScheduler gait;
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    gait.update(est, walk, safety);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const GaitState advanced = gait.update(est, walk, safety);

    const bool leg0_leg1_offset = std::fabs(advanced.phase[0] - advanced.phase[1]) > 0.25;
    const bool stride_active = advanced.stride_phase_rate_hz.value > 0.0;
    const bool timestamp_set = !advanced.timestamp_us.isZero();

    RobotState near_limit = est;
    const HexapodGeometry geometry = defaultHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState straight_joint{};
        straight_joint.joint_state[0].pos_rad = AngleRad{0.0};
        straight_joint.joint_state[1].pos_rad = AngleRad{0.0};
        straight_joint.joint_state[2].pos_rad = AngleRad{0.0};
        near_limit.leg_states[leg] = geometry.legGeometry[leg].servo.toServoAngles(straight_joint);
    }

    GaitScheduler reach_limited;
    reach_limited.update(near_limit, walk, safety);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const GaitState slowed = reach_limited.update(near_limit, walk, safety);

    return expect(stride_active, "walk intent should produce positive stride phase rate") &&
           expect(leg0_leg1_offset, "tripod gait should offset neighboring leg phases") &&
           expect(timestamp_set, "gait update should stamp output time") &&
           expect(slowed.stride_phase_rate_hz.value < advanced.stride_phase_rate_hz.value,
                  "gait scheduler should reduce stride rate near reach envelope limits");
}



bool gaitSchedulerHoldsWhenUnstable() {
    GaitScheduler gait;
    RobotState est{};
    est.timestamp_us = now_us();
    est.foot_contacts = {true, false, true, false, false, false};
    for (auto& leg : est.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    const GaitState output = gait.update(est, walk, safety);

    bool all_stance = true;
    for (bool leg_stance : output.in_stance) {
        all_stance = all_stance && leg_stance;
    }

    return expect(!output.stable, "gait output should report instability with insufficient support contacts") &&
           expect(output.support_contact_count == 2, "gait output should track support contact count") &&
           expect(output.stride_phase_rate_hz.value == 0.0, "unstable support should stop gait phase progression") &&
           expect(all_stance, "unstable support should keep all legs in stance");
}

bool gaitSchedulerRecoversWithoutPhaseSnapAfterTransientInstability() {
    GaitScheduler gait;
    RobotState stable{};
    stable.timestamp_us = now_us();
    stable.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : stable.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    gait.update(stable, walk, safety);
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    const GaitState before_blip = gait.update(stable, walk, safety);

    RobotState unstable = stable;
    unstable.foot_contacts = {true, false, true, false, false, false};
    const GaitState hold = gait.update(unstable, walk, safety);

    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    const GaitState resumed = gait.update(stable, walk, safety);

    const double resumed_progress = wrappedPositiveDelta(before_blip.phase[0], resumed.phase[0]);
    const double hold_deviation = std::abs(hold.phase[0] - before_blip.phase[0]);

    return expect(hold.stride_phase_rate_hz.value == 0.0, "transient instability should pause cadence") &&
           expect(hold_deviation < 0.02, "transient instability should not snap phase to nominal offset") &&
           expect(resumed.stride_phase_rate_hz.value > 0.0, "scheduler should resume cadence once stable") &&
           expect(resumed_progress > 0.001,
                  "phase should continue progressing after instability clears");
}

bool gaitSchedulerCadenceSlewRespectsUpDownLimitsAndWalkRestartSign() {
    constexpr double kCadenceSlewUpHzPerSec = 150.0;
    constexpr double kCadenceSlewDownHzPerSec = 200.0;
    constexpr double kSlewToleranceHz = 0.02;

    GaitScheduler gait;
    RobotState est{};
    est.timestamp_us = now_us();
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = now_us();

    MotionIntent stop = walk;
    stop.requested_mode = RobotMode::STAND;

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    RuntimeGaitPolicy low_policy{};
    low_policy.cadence_hz = FrequencyHz{0.2};

    RuntimeGaitPolicy high_policy = low_policy;
    high_policy.cadence_hz = FrequencyHz{2.5};

    gait.update(est, walk, safety, low_policy);

    std::this_thread::sleep_for(std::chrono::milliseconds(8));
    const GaitState ramp_start = gait.update(est, walk, safety, low_policy);
    TimePointUs previous_time = ramp_start.timestamp_us;
    double previous_rate = ramp_start.stride_phase_rate_hz.value;

    for (int i = 0; i < 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        const GaitState ramped = gait.update(est, walk, safety, high_policy);
        const double dt_sec = static_cast<double>((ramped.timestamp_us - previous_time).value) * 1e-6;
        const double delta_rate = ramped.stride_phase_rate_hz.value - previous_rate;
        const double max_up = kCadenceSlewUpHzPerSec * std::max(0.0, dt_sec) + kSlewToleranceHz;
        if (!expect(delta_rate <= max_up, "abrupt cadence increase exceeded up slew limit")) {
            return false;
        }
        if (!expect(delta_rate >= -kSlewToleranceHz,
                    "cadence increase produced negative slew delta (sign error)")) {
            return false;
        }
        previous_time = ramped.timestamp_us;
        previous_rate = ramped.stride_phase_rate_hz.value;
    }

    for (int i = 0; i < 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        const GaitState ramped = gait.update(est, walk, safety, low_policy);
        const double dt_sec = static_cast<double>((ramped.timestamp_us - previous_time).value) * 1e-6;
        const double delta_rate = ramped.stride_phase_rate_hz.value - previous_rate;
        const double max_down = kCadenceSlewDownHzPerSec * std::max(0.0, dt_sec) + kSlewToleranceHz;
        if (!expect((-delta_rate) <= max_down, "abrupt cadence decrease exceeded down slew limit")) {
            return false;
        }
        if (!expect(delta_rate <= kSlewToleranceHz,
                    "cadence decrease produced positive slew delta (sign error)")) {
            return false;
        }
        if (!expect(ramped.stride_phase_rate_hz.value >= -kSlewToleranceHz,
                    "cadence should never cross negative while ramping down")) {
            return false;
        }
        previous_time = ramped.timestamp_us;
        previous_rate = ramped.stride_phase_rate_hz.value;
    }

    const GaitState stopped = gait.update(est, stop, safety, high_policy);
    if (!expect(stopped.stride_phase_rate_hz.value == 0.0, "stopping walk should force zero cadence")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(8));
    const GaitState restarted = gait.update(est, walk, safety, high_policy);
    if (!expect(restarted.stride_phase_rate_hz.value >= 0.0, "restart should not produce negative cadence")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(8));
    const GaitState restarted_2 = gait.update(est, walk, safety, high_policy);
    const double restart_dt_sec = static_cast<double>((restarted_2.timestamp_us - restarted.timestamp_us).value) * 1e-6;
    const double restart_delta = restarted_2.stride_phase_rate_hz.value - restarted.stride_phase_rate_hz.value;
    const double restart_max_up = kCadenceSlewUpHzPerSec * std::max(0.0, restart_dt_sec) + kSlewToleranceHz;

    return expect(restart_delta >= -kSlewToleranceHz,
                  "restart cadence ramp should not flip sign") &&
           expect(restart_delta <= restart_max_up,
                  "restart cadence ramp should still obey up slew limit");
}

bool bodyControllerUsesGaitState() {
    BodyController body;
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;

    SafetyState safety{};
    safety.inhibit_motion = false;

    GaitState gait{};
    gait.in_stance.fill(true);
    gait.phase.fill(0.0);
    gait.in_stance[1] = false;
    gait.phase[1] = 0.75;

    const LegTargets targets = body.update(est, walk, gait, safety);

    const double leg0_x = targets.feet[0].pos_body_m.x;
    const double leg1_x = targets.feet[1].pos_body_m.x;
    const double leg1_z = targets.feet[1].pos_body_m.z;

    return expect(leg0_x != leg1_x, "stance and swing legs should get different x placement") &&
           expect(leg1_z > targets.feet[0].pos_body_m.z, "swing leg should receive swing height lift");
}

bool bodyControllerKeepsSwingTransitionContinuous() {
    BodyController body;
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.heading_rad = AngleRad{0.0};

    SafetyState safety{};
    safety.inhibit_motion = false;

    GaitState near_stance_end{};
    near_stance_end.in_stance.fill(true);
    near_stance_end.phase.fill(0.0);
    near_stance_end.phase[0] = 0.49;
    near_stance_end.in_stance[0] = true;
    near_stance_end.stride_phase_rate_hz = FrequencyHz{1.0};

    GaitState near_swing_start = near_stance_end;
    near_swing_start.phase[0] = 0.51;
    near_swing_start.in_stance[0] = false;

    const LegTargets before = body.update(est, walk, near_stance_end, safety);
    const LegTargets after = body.update(est, walk, near_swing_start, safety);

    const double x_jump = std::abs(after.feet[0].pos_body_m.x - before.feet[0].pos_body_m.x);
    const double z_jump = std::abs(after.feet[0].pos_body_m.z - before.feet[0].pos_body_m.z);
    const double x_vel_jump = std::abs(after.feet[0].vel_body_mps.x - before.feet[0].vel_body_mps.x);

    return expect(x_jump < 0.005, "swing transition should keep x target continuous") &&
           expect(z_jump < 0.003, "swing transition should keep z target continuous") &&
           expect(x_vel_jump < 0.05, "swing transition should avoid large x velocity discontinuity");
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
        body_targets.feet[leg] = fk.footInBodyFrame(known_joint, geometry.legGeometry[leg]);
    }

    const JointTargets joints = ik.solve(est, body_targets, safety);

    constexpr int kReferenceLeg = 0;
    const LegState joint_frame =
        geometry.legGeometry[kReferenceLeg].servo.toJointAngles(joints.leg_states[kReferenceLeg]);
    const FootTarget fk_body = fk.footInBodyFrame(joint_frame, geometry.legGeometry[kReferenceLeg]);
    const Vec3 diff = fk_body.pos_body_m - body_targets.feet[kReferenceLeg].pos_body_m;
    const double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    if (!expect(err < 0.04, "ik/fk round trip should stay close to body target")) {
        return false;
    }

    return expect(finiteJointTargets(joints), "ik output should stay finite for reachable body targets");
}

bool ikMaintainsCoxaContinuityAcrossAtanBranchCut() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegIK ik(geometry);

    RobotState est{};
    SafetyState safety{};
    safety.leg_enabled.fill(true);

    const int leg = 0;
    const LegGeometry& leg_geo = geometry.legGeometry[leg];
    LegState previous_joint{};
    previous_joint.joint_state[0].pos_rad = AngleRad{kPi - 0.01};
    previous_joint.joint_state[1].pos_rad = AngleRad{-0.30};
    previous_joint.joint_state[2].pos_rad = AngleRad{-0.90};
    est.leg_states[leg] = leg_geo.servo.toServoAngles(previous_joint);

    const Vec3 foot_leg_frame{-0.05, -1e-4, -0.08};
    const Vec3 relative_to_body = Mat3::rotZ(leg_geo.mountAngle.value) * foot_leg_frame;
    LegTargets targets{};
    targets.feet[leg].pos_body_m = leg_geo.bodyCoxaOffset + relative_to_body;

    const JointTargets solved = ik.solve(est, targets, safety);
    const double previous_servo = est.leg_states[leg].joint_state[0].pos_rad.value;
    const double solved_servo = solved.leg_states[leg].joint_state[0].pos_rad.value;
    const double delta = std::abs(solved_servo - previous_servo);

    return expect(delta < 0.5,
                  "ik coxa command should stay continuous near +/-pi branch cut");
}

bool ikFkRoundTripStaysAccurateAcrossBodyAttitudeSweep() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegIK ik(geometry);
    LegFK fk;

    RobotState est{};
    SafetyState safety{};
    safety.leg_enabled.fill(true);

    LegTargets targets{};
    constexpr double kRollSweep[] = {-0.05, 0.0, 0.05};
    constexpr double kPitchSweep[] = {-0.05, 0.0, 0.05};
    constexpr double kYawSweep[] = {-0.10, 0.0, 0.10};
    constexpr double kForwardOffsetsM[] = {-0.008, 0.0, 0.008};
    constexpr double kLateralOffsetsM[] = {-0.008, 0.0, 0.008};
    constexpr double kVerticalOffsetsM[] = {-0.006, 0.0, 0.006};
    constexpr double kMaxRoundTripErrorM = 0.08;
    constexpr int kRepresentativeLegs[] = {0};
    double max_round_trip_error_m = 0.0;

    for (double roll_rad : kRollSweep) {
        for (double pitch_rad : kPitchSweep) {
            for (double yaw_rad : kYawSweep) {
                for (double x_offset_m : kForwardOffsetsM) {
                    for (double y_offset_m : kLateralOffsetsM) {
                        for (double z_offset_m : kVerticalOffsetsM) {
                            for (int leg : kRepresentativeLegs) {
                                LegState nominal_joint{};
                                nominal_joint.joint_state[0].pos_rad = AngleRad{0.0};
                                nominal_joint.joint_state[1].pos_rad = AngleRad{-0.55};
                                nominal_joint.joint_state[2].pos_rad = AngleRad{-0.95};
                                const FootTarget nominal_stance =
                                    fk.footInBodyFrame(nominal_joint, geometry.legGeometry[leg]);
                                const Vec3 attitude_coupled_offset{
                                    x_offset_m + (0.01 * pitch_rad),
                                    y_offset_m + (0.01 * yaw_rad),
                                    z_offset_m - (0.01 * roll_rad)};
                                targets.feet[leg].pos_body_m =
                                    nominal_stance.pos_body_m + attitude_coupled_offset;
                            }

                            const JointTargets solved = ik.solve(est, targets, safety);
                            if (!expect(finiteJointTargets(solved),
                                        "ik output should stay finite through body attitude sweep")) {
                                return false;
                            }

                            for (int leg : kRepresentativeLegs) {
                                const LegState joint_frame =
                                    geometry.legGeometry[leg].servo.toJointAngles(solved.leg_states[leg]);
                                const FootTarget fk_body =
                                    fk.footInBodyFrame(joint_frame, geometry.legGeometry[leg]);
                                const Vec3 error =
                                    fk_body.pos_body_m - targets.feet[leg].pos_body_m;
                                const double err_norm_m =
                                    std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
                                max_round_trip_error_m = std::max(max_round_trip_error_m, err_norm_m);
                                est.leg_states[leg] = solved.leg_states[leg];
                            }
                        }
                    }
                }
            }
        }
    }

    if (max_round_trip_error_m >= kMaxRoundTripErrorM) {
        std::cerr << "FAIL: ik/fk sweep max round-trip error was " << max_round_trip_error_m
                  << " m (tolerance " << kMaxRoundTripErrorM << " m)\n";
        return false;
    }
    return true;
}

bool stabilityTrackerUsesJointSpaceForServoFeedbackState() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState joint_state{};
        joint_state.joint_state[0].pos_rad = AngleRad{0.0};
        joint_state.joint_state[1].pos_rad = AngleRad{-0.6};
        joint_state.joint_state[2].pos_rad = AngleRad{-0.8};
        est.leg_states[leg] = geometry.legGeometry[leg].servo.toServoAngles(joint_state);
    }

    const StabilityAssessment stability = assessStability(est);
    return expect(stability.support_contact_count == kNumLegs,
                  "stability tracker should count all servo-feedback contacts") &&
           expect(stability.has_support_polygon,
                  "stability tracker should produce support polygon from servo-feedback state") &&
           expect(stability.com_inside_support_polygon,
                  "servo-feedback stance should keep COM inside support polygon");
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

    const DurationSec loop_dt{0.02};
    const PipelineStepResult result = pipeline.runStep(estimated, walk_intent, safety, loop_dt, true, 99);

    return expect(result.status.active_mode == RobotMode::WALK || result.status.active_mode == RobotMode::STAND,
                  "pipeline should gate startup into preload or walk mode") &&
           expect(result.status.loop_counter == 99, "pipeline should preserve loop counter") &&
           expect(finiteJointTargets(result.joint_targets), "pipeline joint targets should be finite");
}

} // namespace

int main() {
    if (!gaitSchedulerRespondsToWalkIntent()) {
        return EXIT_FAILURE;
    }
    if (!gaitSchedulerHoldsWhenUnstable()) {
        return EXIT_FAILURE;
    }
    if (!gaitSchedulerRecoversWithoutPhaseSnapAfterTransientInstability()) {
        return EXIT_FAILURE;
    }
    if (!gaitSchedulerCadenceSlewRespectsUpDownLimitsAndWalkRestartSign()) {
        return EXIT_FAILURE;
    }
    if (!plannerBodyIkRemainLeftRightMirrored()) {
        return EXIT_FAILURE;
    }
    if (!bodyControllerUsesGaitState()) {
        return EXIT_FAILURE;
    }
    if (!bodyControllerKeepsSwingTransitionContinuous()) {
        return EXIT_FAILURE;
    }
    if (!ikFkChainTracksBodyTargets()) {
        return EXIT_FAILURE;
    }
    if (!ikMaintainsCoxaContinuityAcrossAtanBranchCut()) {
        return EXIT_FAILURE;
    }
    if (!ikFkRoundTripStaysAccurateAcrossBodyAttitudeSweep()) {
        return EXIT_FAILURE;
    }
    if (!stabilityTrackerUsesJointSpaceForServoFeedbackState()) {
        return EXIT_FAILURE;
    }
    if (!controlPipelineProducesStableOutputs()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
