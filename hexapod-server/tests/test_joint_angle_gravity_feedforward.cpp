#include "joint_angle_gravity_feedforward.hpp"

#include "geometry_config.hpp"
#include "hexapod_dynamics_constants.hpp"
#include "velocity_lead_experiment.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool expect(bool ok, const char* msg) {
    if (!ok) {
        std::cerr << "FAIL: " << msg << '\n';
    }
    return ok;
}

control_config::GravityFeedforwardConfig sampleConfig() {
    control_config::GravityFeedforwardConfig cfg{};
    cfg.enabled = true;
    cfg.max_gyro_radps = 0.35;
    cfg.accel_norm_margin_mps2 = 1.5;
    cfg.scale_coxa = 0.0;
    cfg.scale_femur = 1.0;
    cfg.scale_tibia = 1.0;
    cfg.include_foot_reaction = true;
    cfg.include_self_weight = false;
    return cfg;
}

RobotState imuUpright() {
    RobotState est{};
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.accel_mps2 = {0.0, 0.0, 9.80665};
    est.imu.gyro_radps = {0.0, 0.0, 0.0};
    for (auto& quality : est.joint_state_quality) {
        quality.position_valid = true;
        quality.velocity_valid = true;
        quality.source = JointStateSource::Simulated;
        quality.confidence = 1.0;
    }
    return est;
}

GaitState allStanceContacts() {
    GaitState gait{};
    gait.in_stance.fill(true);
    return gait;
}

Vec3 gravityDownFromUprightImu() {
    return Vec3{0.0, 0.0, -1.0};
}

double footReactionPerLeg(int n_stance_legs) {
    if (n_stance_legs <= 0) {
        return 0.0;
    }
    return hexapod_dynamics::kBodyMassKg * hexapod_dynamics::kStandardGravityMps2 /
           static_cast<double>(n_stance_legs);
}

} // namespace

int main() {
    const HexapodGeometry geo = geometry_config::buildDefaultHexapodGeometry();
    {
        physics_sim_test_utils::VelocityLeadExperiment lead(.005);
        JointTargets reference{};
        reference.leg_states[0].joint_state[FEMUR].pos_rad.value = .2;
        reference.leg_states[0].joint_state[FEMUR].vel_radps.value = -2;
        auto unchanged = lead.apply(reference, false);
        if (!expect(unchanged.leg_states[0].joint_state[FEMUR].pos_rad.value == .2,
                    "disabled lead experiment must preserve the reference")) return 1;
        auto motor = lead.apply(reference, true);
        const double error = motor.leg_states[0].joint_state[FEMUR].pos_rad.value - .1;
        if (!expect(std::abs(error - (.2-.1+.08*-2)) < 1e-12,
                    "test motor target must implement Kd/Kp times signed reference velocity")) return 1;
        motor = lead.apply(reference, true);
        if (!expect(motor.leg_states[0].joint_state[FEMUR].vel_radps.value == 0,
                    "constant lead target must emit zero target-difference metadata")) return 1;
        reference.leg_states[0].joint_state[FEMUR].vel_radps.value = 100;
        motor = lead.apply(reference, true);
        if (!expect(std::abs(motor.leg_states[0].joint_state[FEMUR].pos_rad.value-.2
                    -.08*hexapod_dynamics::kServoNoLoadSpeedRadPerSec) < 1e-12,
                    "test reference velocity is bounded by the motor's existing no-load envelope")) return 1;
    }

    for (int hz : {120, 200, 240, 480}) {
        physics_sim_test_utils::VelocityLeadExperiment lead(1.0/hz);
        JointTargets reference{}, motor{};
        reference.leg_states[0].joint_state[FEMUR].vel_radps.value = 2.0;
        for (int i=0; i<hz/2; ++i) motor = lead.apply(reference, true, true);
        const double offset = .16 * (1-std::exp(-.5/.08));
        if (!expect(std::abs(motor.leg_states[0].joint_state[FEMUR].pos_rad.value-offset) < 1e-12,
                    "filtered lead uses exact cadence-independent motor timescale")) return 1;
        reference.leg_states[0].joint_state[FEMUR].vel_radps.value = 0;
        for (int i=0; i<hz/2; ++i) motor = lead.apply(reference, true, true);
        if (!expect(std::abs(motor.leg_states[0].joint_state[FEMUR].pos_rad.value-offset*std::exp(-.5/.08)) < 1e-12,
                    "filtered lead decays continuously when reference stops")) return 1;
        (void)lead.apply(reference, false, true);
        motor = lead.apply(reference, true, true);
        if (!expect(motor.leg_states[0].joint_state[FEMUR].pos_rad.value == 0,
                    "disabled experiment clears filtered bias")) return 1;
    }

    {
        auto cfg = sampleConfig();
        cfg.include_self_weight = true;
        cfg.include_foot_reaction = false;
        cfg.delta_lpf_tau_s = .08;
        auto measured = imuUpright();
        GaitState gait{};
        JointTargets previous{};
        resetJointAngleGravityFeedforwardState();
        for (int step = 0; step < 100; ++step) {
            previous = {};
            applyJointAngleGravityFeedforward(cfg, geo, measured, gait, previous);
        }
        measured.imu.gyro_radps.x = cfg.max_gyro_radps + .01;
        JointTargets gated{};
        applyJointAngleGravityFeedforward(cfg, geo, measured, gait, gated);
        const double before = std::abs(previous.leg_states[0].joint_state[FEMUR].pos_rad.value);
        const double after = std::abs(gated.leg_states[0].joint_state[FEMUR].pos_rad.value);
        if (!expect(after > 0 && after < before,
                    "gyro rejection must fade the existing filtered bias, not bypass the filter")) return 1;
        if (!expect(std::abs(after - before * std::exp(-.004 / cfg.delta_lpf_tau_s)) < 1e-12,
                    "rejected measurements must drive a zero-input filter decay")) return 1;
        double reference = 0;
        measured = imuUpright();
        for (int hz : {120, 240, 480}) {
            resetJointAngleGravityFeedforwardState();
            JointTargets filtered{};
            for (int step = 0; step < hz / 2; ++step) {
                filtered = {};
                applyJointAngleGravityFeedforward(cfg, geo, measured, gait, filtered, nullptr, 1.0 / hz);
            }
            const double final = filtered.leg_states[0].joint_state[FEMUR].pos_rad.value;
            if (hz == 120) reference = final;
            if (!expect(std::abs(final - reference) < 1e-12,
                        "filter response over elapsed time must agree at 120/240/480 Hz")) return 1;
        }
        cfg.enabled = false;
        JointTargets disabled{};
        applyJointAngleGravityFeedforward(cfg, geo, measured, gait, disabled);
        if (!expect(disabled.leg_states[0].joint_state[FEMUR].pos_rad.value == 0,
                    "explicitly disabling FF must clear compensation immediately")) return 1;
        resetJointAngleGravityFeedforwardState();
    }

    {
        auto cfg = sampleConfig();
        cfg.include_self_weight = true;
        cfg.include_foot_reaction = false;
        std::array<double, kJointsPerLeg> stiffness{1.0, 2.0, 0.8};
        const auto d = computeLegGravityCompensation(geo.legGeometry[0], 0, .2, -.8,
            {0, 0, -1}, 0, cfg, &stiffness);
        if (!expect(std::abs(d.delta_femur_rad * stiffness[FEMUR] - d.torque_femur_nm) < 1e-12
                 && std::abs(d.delta_tibia_rad * stiffness[TIBIA] - d.torque_tibia_nm) < 1e-12,
                    "reported actuator stiffness must convert torque to angle without an inertia proxy")) return 1;
        cfg.stiffness_gain_scale = .2;
        const auto scaled = computeLegGravityCompensation(geo.legGeometry[0], 0, .2, -.8,
            {0, 0, -1}, 0, cfg, &stiffness);
        if (!expect(scaled.delta_femur_rad == d.delta_femur_rad && scaled.delta_tibia_rad == d.delta_tibia_rad,
                    "proxy calibration must not rescale a reported actuator gain")) return 1;
        stiffness[FEMUR] = 0;
        stiffness[TIBIA] = std::numeric_limits<double>::quiet_NaN();
        const auto invalid = computeLegGravityCompensation(geo.legGeometry[0], 0, .2, -.8,
            {0, 0, -1}, 0, cfg, &stiffness);
        if (!expect(invalid.delta_femur_rad == 0 && invalid.delta_tibia_rad == 0,
                    "invalid explicit stiffness must not produce an angle or silently use the proxy")) return 1;
        stiffness = {1, 1e-6, 1e-6};
        const auto bounded = computeLegGravityCompensation(geo.legGeometry[0], 0, .2, -.8,
            {0, 0, -1}, 0, cfg, &stiffness);
        if (!expect(std::abs(bounded.delta_femur_rad) == cfg.max_delta_femur_rad
                 && std::abs(bounded.delta_tibia_rad) == cfg.max_delta_tibia_rad,
                    "reported stiffness must retain existing angle clamps")) return 1;
        RobotState measured = imuUpright();
        JointTargets target{};
        GaitState gait{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            measured.joint_stiffness_valid[leg] = true;
            measured.joint_stiffness_nm_per_rad[leg] = {1, 2, .8};
        }
        resetJointAngleGravityFeedforwardState();
        applyJointAngleGravityFeedforward(cfg, geo, measured, gait, target);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const auto q = geo.legGeometry[leg].servo.toJointAngles(measured.leg_states[leg]);
            const auto expected = computeLegGravityCompensation(geo.legGeometry[leg],
                q.joint_state[COXA].pos_rad.value, q.joint_state[FEMUR].pos_rad.value,
                q.joint_state[TIBIA].pos_rad.value, {0,0,-1}, 0, cfg,
                &measured.joint_stiffness_nm_per_rad[leg]);
            if (!expect(std::abs(target.leg_states[leg].joint_state[FEMUR].pos_rad.value
                     - geo.legGeometry[leg].servo.femurSign * expected.delta_femur_rad) < 1e-12,
                        "controller must use the measured actuator contract on every mirrored leg")) return 1;
        }
    }

    {
        auto cfg = sampleConfig();
        cfg.include_self_weight = true;
        cfg.include_foot_reaction = false;
        const auto self = computeLegGravityCompensation(geo.legGeometry[0], 0, 0, 0,
                                                       {0, 0, -1}, 0, cfg);
        // FK z increases with positive pitch at the horizontal pose. Positive
        // gravity compensation must lift, not push the leg further downward.
        if (!expect(self.delta_femur_rad > 0 && self.delta_tibia_rad > 0,
                    "self-weight compensation must oppose gravity in mechanical FK space")) return 1;
        cfg.include_self_weight = false;
        cfg.include_foot_reaction = true;
        const auto reaction = computeLegGravityCompensation(geo.legGeometry[0], 0, 0, 0,
                                                           {0, 0, -1}, .1, cfg);
        if (!expect(reaction.delta_femur_rad < 0 && reaction.delta_tibia_rad < 0,
                    "upward foot reaction requires downward holding torque")) return 1;
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        cfg.enabled = false;
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        jt.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.1};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.1) < 1e-9,
                    "disabled should not change joints")) {
            return 1;
        }
    }

    // Independent virtual-work oracle: only descendants move when a pitch
    // joint is perturbed. The current sim's compound tibia mass is at 43 mm.
    for (const auto& leg : geo.legGeometry) {
        auto cfg = sampleConfig();
        cfg.include_self_weight = true;
        cfg.include_foot_reaction = false;
        for (int i = 0; i < 100; ++i) {
            const double q1 = -.7 + .014*i, q2 = -.6 + .009*i, q3 = -1.8 + .015*i;
            const Vec3 down{.3, -.4, -std::sqrt(.75)};
            const Vec3 g = legFromBodyFrame(leg) * down;
            auto potential = [&](double f, double t) {
                const double rF = leg.coxaLength.value + .5*leg.femurLength.value*std::cos(f);
                const double zF = .5*leg.femurLength.value*std::sin(f);
                const double com = leg.tibiaLength.value * (.043/.104);
                const double rT = leg.coxaLength.value + leg.femurLength.value*std::cos(f) + com*std::cos(f+t);
                const double zT = leg.femurLength.value*std::sin(f) + com*std::sin(f+t);
                return -9.80665 * (.070*(rF*(g.x*std::cos(q1)+g.y*std::sin(q1))+g.z*zF)
                                + .063*(rT*(g.x*std::cos(q1)+g.y*std::sin(q1))+g.z*zT));
            };
            const auto got = computeLegGravityCompensation(leg, q1, q2, q3, down, 0, cfg);
            const double gf = (potential(q2+1e-6,q3)-potential(q2-1e-6,q3))/2e-6;
            const double gt = (potential(q2,q3+1e-6)-potential(q2,q3-1e-6))/2e-6;
            if (!expect(std::abs(got.torque_femur_nm-gf) < 1e-8 && std::abs(got.torque_tibia_nm-gt) < 1e-8,
                        "self-weight holding torque must equal physical potential gradient")) return 1;
            auto longerCoxa = leg; longerCoxa.coxaLength.value += .1;
            const auto longer = computeLegGravityCompensation(longerCoxa, q1, q2, q3, down, 0, cfg);
            if (!expect(std::abs(got.torque_femur_nm-longer.torque_femur_nm) < 1e-12,
                        "proximal coxa must not contribute femur holding torque")) return 1;
        }
    }

    {
        auto cfg = sampleConfig();
        cfg.include_self_weight = true;
        cfg.include_foot_reaction = true;
        RobotState est = imuUpright();
        GaitState gait{}; gait.in_stance.fill(false);
        est.foot_contacts.fill(false);
        JointTargets targets{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            LegState mechanical{};
            mechanical.joint_state[FEMUR].pos_rad = AngleRad{.2};
            mechanical.joint_state[TIBIA].pos_rad = AngleRad{-.9};
            est.leg_states[leg] = geo.legGeometry[leg].servo.toServoAngles(mechanical);
            targets.leg_states[leg] = est.leg_states[leg];
        }
        const auto before = targets;
        resetJointAngleGravityFeedforwardState();
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, targets);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const auto want = computeLegGravityCompensation(geo.legGeometry[leg], 0,.2,-.9,{0,0,-1},0,cfg);
            const auto actual = geo.legGeometry[leg].servo.toJointAngles(targets.leg_states[leg]);
            if (!expect(std::abs(actual.joint_state[FEMUR].pos_rad.value-.2-want.delta_femur_rad) < 1e-9,
                        "airborne leg must get self weight but no foot reaction, including mirrored signs")) return 1;
        }
        const auto airborneResult = targets;
        // Reaction is nonzero on five OTHER legs; it must not leak onto leg 0.
        gait.in_stance.fill(true); gait.in_stance[0] = false;
        est.foot_contacts.fill(true); est.foot_contacts[0] = false;
        targets = before; resetJointAngleGravityFeedforwardState();
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, targets);
        if (!expect(std::abs(targets.leg_states[0].joint_state[FEMUR].pos_rad.value
                         - airborneResult.leg_states[0].joint_state[FEMUR].pos_rad.value) < 1e-12,
                    "other legs' support reactions must not enter airborne self-weight compensation")) return 1;
        // Bounded mode still refuses unreliable measurements on an airborne leg.
        targets = before; est.joint_state_quality[0].position_valid = false;
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, targets);
        if (!expect(targets.leg_states[0].joint_state[FEMUR].pos_rad.value == before.leg_states[0].joint_state[FEMUR].pos_rad.value,
                    "invalid airborne joint state must not be compensated")) return 1;
        targets = before; est.joint_state_quality[0].position_valid = true;
        est.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{std::numeric_limits<double>::quiet_NaN()};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, targets);
        if (!expect(targets.leg_states[0].joint_state[FEMUR].pos_rad.value == before.leg_states[0].joint_state[FEMUR].pos_rad.value,
                    "nonfinite feedback must not corrupt a finite command")) return 1;
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        RobotState est = imuUpright();
        est.imu.gyro_radps = {2.0, 0.0, 0.0};
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0,
                    "high gyro should skip feedforward")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        RobotState est = imuUpright();
        est.imu.accel_mps2 = {0.0, 0.0, 4.0};
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0,
                    "accel norm outside margin should skip feedforward")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        cfg.accel_norm_margin_mps2 = 0.0;
        RobotState est = imuUpright();
        est.imu.accel_mps2 = {0.0, 0.0, 4.0};
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        const LegState joint_target =
            geo.legGeometry[0].servo.toJointAngles(jt.leg_states[0]);
        const LegGravityCompensation want =
            computeLegGravityCompensation(
                geo.legGeometry[0],
                joint_target.joint_state[COXA].pos_rad.value,
                joint_target.joint_state[FEMUR].pos_rad.value,
                joint_target.joint_state[TIBIA].pos_rad.value,
                gravityDownFromUprightImu(), footReactionPerLeg(6), cfg);
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value - want.delta_femur_rad) < 1e-8 &&
                        std::abs(jt.leg_states[0].joint_state[TIBIA].pos_rad.value - want.delta_tibia_rad) < 1e-8,
                    "accel norm check disabled should match computeLegGravityCompensation")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        cfg.scale_femur = 0.0;
        cfg.scale_tibia = 0.0;
        cfg.scale_coxa = 0.0;
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        jt.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.05};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.05) < 1e-9,
                    "all scales zero should early-return without changing joints")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        gait.in_stance[0] = false;
        est.foot_contacts.fill(true);
        JointTargets jt{};
        const LegState joint_target =
            geo.legGeometry[1].servo.toJointAngles(jt.leg_states[1]);
        const LegGravityCompensation want_leg1 =
            computeLegGravityCompensation(
                geo.legGeometry[1],
                joint_target.joint_state[COXA].pos_rad.value,
                joint_target.joint_state[FEMUR].pos_rad.value,
                joint_target.joint_state[TIBIA].pos_rad.value,
                gravityDownFromUprightImu(), footReactionPerLeg(5), cfg);
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0,
                    "swing leg (not in_stance) should not get feedforward")) {
            return 1;
        }
        const double expected_servo_delta =
            -want_leg1.delta_femur_rad;
        if (!expect(std::abs(jt.leg_states[1].joint_state[FEMUR].pos_rad.value
                             - expected_servo_delta) < 1e-8,
                    "mirrored stance leg should receive the signed servo-space feedforward")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        est.foot_contacts[2] = false;
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[2].joint_state[FEMUR].pos_rad.value == 0.0,
                    "no foot contact should skip feedforward for that leg")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        cfg.mode = control_config::GravityFeedforwardMode::Off;
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        jt.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.1};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.1) < 1e-9,
                    "off mode should not change joints")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        RobotState est = imuUpright();
        est.joint_state_quality[0].source = JointStateSource::CommandEcho;
        est.joint_state_quality[0].confidence = 0.2;
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0,
                    "bounded mode should skip command-echo-only joint state")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        cfg.scale_femur = 5000.0;
        cfg.max_delta_femur_rad = 0.05;
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value)
                             - 0.05) < 1e-8,
                    "femur delta magnitude should clamp to max_delta_femur_rad")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        const Vec3 g = gravityDownFromUprightImu();
        const double F = footReactionPerLeg(6);
        const LegGravityCompensation a =
            computeLegGravityCompensation(
                geo.legGeometry[0], 0.0, 0.0, 0.0, g, 0.01 * F, cfg);
        const LegGravityCompensation b = computeLegGravityCompensation(
            geo.legGeometry[0], 0.0, 0.0, 0.0, g, 0.02 * F, cfg);
        if (!expect(std::abs(a.delta_femur_rad * 2.0 - b.delta_femur_rad) < 5e-7 &&
                        std::abs(a.delta_tibia_rad * 2.0 - b.delta_tibia_rad) < 5e-7,
                    "doubling foot reaction should double sag deltas (pre-saturation)")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        const Vec3 g = gravityDownFromUprightImu();
        const double F = footReactionPerLeg(6);
        const LegGeometry leg_pos = geo.legGeometry[0];
        LegGeometry leg_neg = leg_pos;
        leg_neg.mountAngle = AngleRad{-leg_pos.mountAngle.value};
        const LegGravityCompensation p =
            computeLegGravityCompensation(leg_pos, 0.2, 0.4, -0.1, g, F, cfg);
        const LegGravityCompensation n =
            computeLegGravityCompensation(leg_neg, 0.2, 0.4, -0.1, g, F, cfg);
        if (!expect(std::abs(std::abs(p.delta_femur_rad) - std::abs(n.delta_femur_rad)) < 1e-9 &&
                        std::abs(std::abs(p.delta_tibia_rad) - std::abs(n.delta_tibia_rad)) < 1e-9,
                    "mirrored mount angle should give equal femur/tibia sag magnitudes")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        const Vec3 g = gravityDownFromUprightImu();
        const double F = 0.01 * footReactionPerLeg(6);
        const LegGeometry& leg = geo.legGeometry[0];
        const LegGravityCompensation ext =
            computeLegGravityCompensation(leg, 0.0, 0.25, -0.05, g, F, cfg);
        const LegGravityCompensation flex =
            computeLegGravityCompensation(leg, 0.0, 1.05, -0.55, g, F, cfg);
        if (!expect(std::abs(flex.delta_femur_rad - ext.delta_femur_rad) > 1e-6,
                    "flexed vs extended femur pose should change predicted femur delta")) {
            return 1;
        }
    }

    return 0;
}
