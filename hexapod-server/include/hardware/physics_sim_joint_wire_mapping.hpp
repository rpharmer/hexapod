#pragma once

#include "physics_sim_protocol.hpp"
#include "types.hpp"

#include <cmath>

/// Maps one leg between minphys3d sim joint angles (wire order: coxa, femur, tibia) and
/// hexapod-server LegState in servo space (after ServoCalibration / IK conventions).
///
/// The built-in `hexapod-physics-sim --serve` rig reports zero wire angles at its assembled
/// relaxed pose, not at the server IK convention's mechanical zero. The coxa aligns at zero, but
/// the femur and tibia start pitched by `physics_sim::kWireZero*MechanicalRad`. The bridge must
/// translate between those conventions or the server's standing targets collapse nearly straight
/// under the body in the sim.
namespace physics_sim_joint_wire_mapping {

inline double normalizedServoSign(double sign) {
    if (!std::isfinite(sign)) {
        return 1.0;
    }
    return sign >= 0.0 ? 1.0 : -1.0;
}

inline double jointMechanicalFromSimWireAngle(int joint_index, float sim_angle) {
    switch (joint_index) {
        case COXA:
            return static_cast<double>(physics_sim::kWireZeroCoxaMechanicalRad + sim_angle);
        case FEMUR:
            return static_cast<double>(physics_sim::kWireZeroFemurMechanicalRad + sim_angle);
        case TIBIA:
            return static_cast<double>(physics_sim::kWireZeroTibiaMechanicalRad + sim_angle);
        default:
            return static_cast<double>(sim_angle);
    }
}

inline float simWireAngleFromJointMechanical(int joint_index, double joint_angle) {
    switch (joint_index) {
        case COXA:
            return static_cast<float>(joint_angle - physics_sim::kWireZeroCoxaMechanicalRad);
        case FEMUR:
            return static_cast<float>(joint_angle - physics_sim::kWireZeroFemurMechanicalRad);
        case TIBIA:
            return static_cast<float>(joint_angle - physics_sim::kWireZeroTibiaMechanicalRad);
        default:
            return static_cast<float>(joint_angle);
    }
}

inline LegState jointMechanicalFromServoLeg(
    const ServoCalibration& cal,
    const LegState& servo_leg) {
    LegState joint{};
    joint.joint_state[COXA].pos_rad = AngleRad{
        normalizedServoSign(cal.coxaSign) * (servo_leg.joint_state[COXA].pos_rad.value - cal.coxaOffset.value)};
    joint.joint_state[FEMUR].pos_rad = AngleRad{
        normalizedServoSign(cal.femurSign) * (servo_leg.joint_state[FEMUR].pos_rad.value - cal.femurOffset.value)};
    joint.joint_state[TIBIA].pos_rad = AngleRad{
        normalizedServoSign(cal.tibiaSign) * (servo_leg.joint_state[TIBIA].pos_rad.value - cal.tibiaOffset.value)};
    return joint;
}

inline void simWireTargetsFromJointMechanical(
    const ServoCalibration& cal,
    int leg_index,
    double jc,
    double jf,
    double jt,
    float& out_sim_c,
    float& out_sim_f,
    float& out_sim_t) {
    (void)cal;
    (void)leg_index;
    out_sim_c = simWireAngleFromJointMechanical(COXA, jc);
    out_sim_f = simWireAngleFromJointMechanical(FEMUR, jf);
    out_sim_t = simWireAngleFromJointMechanical(TIBIA, jt);
}

inline LegState servoLegFromSimWireAngles(
    const ServoCalibration& cal,
    int leg_index,
    float sim_c,
    float sim_f,
    float sim_t) {
    (void)leg_index;
    LegState joint{};
    joint.joint_state[COXA].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(COXA, sim_c)};
    joint.joint_state[FEMUR].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(FEMUR, sim_f)};
    joint.joint_state[TIBIA].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(TIBIA, sim_t)};
    return cal.toServoAngles(joint);
}

inline LegState jointMechanicalFromSimWireAngles(
    const ServoCalibration& cal,
    int leg_index,
    float sim_c,
    float sim_f,
    float sim_t) {
    (void)cal;
    (void)leg_index;
    LegState joint{};
    joint.joint_state[COXA].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(COXA, sim_c)};
    joint.joint_state[FEMUR].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(FEMUR, sim_f)};
    joint.joint_state[TIBIA].pos_rad = AngleRad{jointMechanicalFromSimWireAngle(TIBIA, sim_t)};
    return joint;
}

inline void simWireTargetsFromServoLeg(
    const ServoCalibration& cal,
    int leg_index,
    const LegState& servo_leg,
    float& out_sim_c,
    float& out_sim_f,
    float& out_sim_t) {
    const LegState joint_leg = jointMechanicalFromServoLeg(cal, servo_leg);
    simWireTargetsFromJointMechanical(
        cal,
        leg_index,
        joint_leg.joint_state[COXA].pos_rad.value,
        joint_leg.joint_state[FEMUR].pos_rad.value,
        joint_leg.joint_state[TIBIA].pos_rad.value,
        out_sim_c,
        out_sim_f,
        out_sim_t);
}

} // namespace physics_sim_joint_wire_mapping
