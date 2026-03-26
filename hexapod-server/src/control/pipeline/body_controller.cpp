#include "body_controller.hpp"

#include <algorithm>
#include <cmath>

#include "reach_envelope.hpp"

namespace {

constexpr double kDefaultBodyHeightM = 0.20;

} // namespace

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

    (void)est;

    const std::array<Vec3, kNumLegs> nominal = nominalStance();
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;
    if (walking != previous_walking_) {
        transition_slew_steps_remaining_ = kLegTargetTransitionSlewSteps;
    } else if (transition_slew_steps_remaining_ > 0) {
        --transition_slew_steps_remaining_;
    }
    previous_walking_ = walking;

    const double roll_cmd = intent.twist.twist_pos_rad.x;
    const double pitch_cmd = intent.twist.twist_pos_rad.y;
    const double yaw_cmd_raw = policy.suppression.suppress_turning ? 0.0 : intent.twist.twist_pos_rad.z;
    const TimePointUs now = out.timestamp_us;
    const TimePointUs previous_update_ts = last_update_timestamp_;
    const bool has_positive_dt = !previous_update_ts.isZero() && now.value > previous_update_ts.value;
    const double update_dt_s =
        has_positive_dt ? static_cast<double>(now.value - previous_update_ts.value) / 1'000'000.0 : 0.0;
    if (!yaw_filter_initialized_ || !has_positive_dt) {
        filtered_yaw_cmd_rad_ = yaw_cmd_raw;
        yaw_filter_initialized_ = true;
    } else {
        const double max_step = kYawCommandSlewLimitRadPerSec * update_dt_s;
        const double delta = yaw_cmd_raw - filtered_yaw_cmd_rad_;
        filtered_yaw_cmd_rad_ += std::clamp(delta, -max_step, max_step);
    }
    last_update_timestamp_ = now;
    const double yaw_cmd = filtered_yaw_cmd_rad_;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    double commanded_body_height_m = intent.twist.body_trans_m.z;
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

        target = clampToReachEnvelope(leg, target);
        if (has_previous_targets_ && has_positive_dt) {
            const double speed_limit = transition_slew_steps_remaining_ > 0
                                           ? kLegTargetTransitionSlewLimitMps
                                           : kLegTargetSlewLimitMps;
            const double max_step_m = speed_limit * update_dt_s;
            const Vec3 previous = previous_targets_.feet[leg].pos_body_m;
            const Vec3 delta = target - previous;
            const double delta_mag = std::sqrt((delta.x * delta.x) + (delta.y * delta.y) + (delta.z * delta.z));
            if (delta_mag > max_step_m && delta_mag > 1e-9) {
                target = previous + (delta * (max_step_m / delta_mag));
            }
        }
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    previous_targets_ = out;
    has_previous_targets_ = true;
    return out;
}
