#include "body_controller.hpp"

#include <cmath>

#include "reach_envelope.hpp"

namespace {

constexpr double kDefaultBodyHeightM = 0.20;

} // namespace

BodyController::BodyController(control_config::MotionLimiterConfig config)
    : limiter_config_(config) {}

std::array<Vec3, kNumLegs> BodyController::nominalStance() const {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const double neutral_leg_x =
            leg_geo.coxaLength.value + 0.55 * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);
        const Vec3 neutral_leg_frame{neutral_leg_x, 0.0, -kDefaultBodyHeightM};
        const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
        nominal[leg] = leg_geo.bodyCoxaOffset + (body_from_leg * neutral_leg_frame);
    }
    return nominal;
}

Vec3 BodyController::clampToReachEnvelope(int leg, const Vec3& target_body) const {
    const LegGeometry& leg_geo = geometry_.legGeometry[leg];
    const Vec3 relative_to_coxa = target_body - leg_geo.bodyCoxaOffset;
    const Mat3 leg_from_body = Mat3::rotZ(-leg_geo.mountAngle.value);
    const Vec3 target_leg = leg_from_body * relative_to_coxa;
    const Vec3 clamped_leg = kinematics::clampFootToReachEnvelope(target_leg, leg_geo);
    const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
    return leg_geo.bodyCoxaOffset + (body_from_leg * clamped_leg);
}


void BodyController::setYawCommandSlewEnabled(bool enabled) {
    (void)enabled;
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    RuntimeGaitPolicy fallback_policy{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        fallback_policy.per_leg[leg].step_length_m = LengthM{0.06};
        fallback_policy.per_leg[leg].swing_height_m = LengthM{0.03};
        fallback_policy.per_leg[leg].duty_cycle = 0.5;
    }
    return update(est, intent, gait, fallback_policy, safety);
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const RuntimeGaitPolicy& policy,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();
    MotionLimiterTelemetry limiter_telemetry{};

    (void)est;
    (void)limiter_config_;

    const std::array<Vec3, kNumLegs> nominal = nominalStance();
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;
    (void)walking;

    const double roll_cmd = intent.body_pose_setpoint.orientation_rad.x;
    const double pitch_cmd = intent.body_pose_setpoint.orientation_rad.y;
    const double yaw_cmd_raw = policy.suppression.suppress_turning ? 0.0 : intent.body_pose_setpoint.orientation_rad.z;
    const double yaw_cmd = yaw_cmd_raw;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        intent.body_pose_setpoint.body_trans_m.x,
        intent.body_pose_setpoint.body_trans_m.y,
        0.0};

    double commanded_body_height_m = intent.body_pose_setpoint.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];

        target.z += commanded_body_height_m - kDefaultBodyHeightM;
        target = target - planar_body_offset;

        const PlannedFoothold foothold =
            foothold_planner_.plan(leg, target, intent, gait, policy, walking);

        target = body_rotation * foothold.pos_body_m;
        Vec3 target_vel = body_rotation * foothold.vel_body_mps;

        const Vec3 unclamped_reach_target = target;
        target = clampToReachEnvelope(leg, target);
        if (vecNorm(target - unclamped_reach_target) > 1e-9) {
            limiter_telemetry.hard_clamp_reach = true;
        }
        target_vel = target_vel + cross(intent.body_pose_setpoint.angular_velocity_radps, target);

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    last_motion_limiter_telemetry_ = limiter_telemetry;
    return out;
}
