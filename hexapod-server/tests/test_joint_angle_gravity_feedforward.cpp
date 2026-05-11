#include "joint_angle_gravity_feedforward.hpp"

#include "geometry_config.hpp"
#include "hexapod_dynamics_constants.hpp"

#include <cmath>
#include <iostream>

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
        const LegGravityCompensation want =
            computeLegGravityCompensation(geo.legGeometry[0], 0.0, 0.0, 0.0, gravityDownFromUprightImu(),
                                          footReactionPerLeg(6), cfg);
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
        const LegGravityCompensation want_leg1 =
            computeLegGravityCompensation(geo.legGeometry[1], 0.0, 0.0, 0.0, gravityDownFromUprightImu(),
                                          footReactionPerLeg(5), cfg);
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(jt.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0,
                    "swing leg (not in_stance) should not get feedforward")) {
            return 1;
        }
        if (!expect(std::abs(jt.leg_states[1].joint_state[FEMUR].pos_rad.value - want_leg1.delta_femur_rad) < 1e-8,
                    "stance leg should still get feedforward")) {
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
        cfg.scale_femur = 50.0;
        cfg.max_delta_femur_rad = 0.05;
        RobotState est = imuUpright();
        GaitState gait = allStanceContacts();
        est.foot_contacts.fill(true);
        JointTargets jt{};
        applyJointAngleGravityFeedforward(cfg, geo, est, gait, jt);
        if (!expect(std::abs(jt.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.05) < 1e-8,
                    "femur delta should clamp to max_delta_femur_rad")) {
            return 1;
        }
    }

    {
        control_config::GravityFeedforwardConfig cfg = sampleConfig();
        const Vec3 g = gravityDownFromUprightImu();
        const double F = footReactionPerLeg(6);
        const LegGravityCompensation a =
            computeLegGravityCompensation(geo.legGeometry[0], 0.0, 0.0, 0.0, g, 0.5 * F, cfg);
        const LegGravityCompensation b = computeLegGravityCompensation(geo.legGeometry[0], 0.0, 0.0, 0.0, g, F, cfg);
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
        const double F = footReactionPerLeg(6);
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
