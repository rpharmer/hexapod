#include "body_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kStepLengthM = 0.06;
constexpr double kSwingHeightM = 0.03;
constexpr double kDefaultBodyHeightM = 0.05;
constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;

} // namespace

std::array<Vec3, kNumLegs> BodyController::nominalStance(double body_height_m) const {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const double femur_tibia_reach =
            std::max(0.0, leg_geo.femurLength.value + leg_geo.tibiaLength.value - kReachMarginM);
        const double desired_rho = kNominalReachFraction * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);

        // body_height_m is the requested body-center height above the contact plane.
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
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();

    (void)est;

    double commanded_body_height_m = intent.twist.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    const std::array<Vec3, kNumLegs> nominal = nominalStance(commanded_body_height_m);
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    const double heading = intent.heading_rad.value;
    const Vec3 step_dir{std::cos(heading), std::sin(heading), 0.0};
    const double roll_cmd = intent.twist.twist_pos_rad.x;
    const double pitch_cmd = intent.twist.twist_pos_rad.y;
    const double yaw_cmd = intent.twist.twist_pos_rad.z;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];
        Vec3 target_vel = Vec3{};

        target = target - planar_body_offset;
        target_vel = target_vel - intent.twist.body_trans_mps;

        if (walking) {
            const double phase = clamp01(gait.phase[leg]);
            const double phase_rate = std::max(gait.stride_phase_rate_hz.value, 0.0);
            if (gait.in_stance[leg]) {
                const double stance_alpha = clamp01(phase / 0.5);
                const double step_delta = (0.5 - stance_alpha) * kStepLengthM;
                const double step_vel = -2.0 * phase_rate * kStepLengthM;
                target = target + (step_dir * step_delta);
                target_vel = target_vel + (step_dir * step_vel);
            } else {
                const double swing_alpha = clamp01((phase - 0.5) / 0.5);
                const double step_delta = (swing_alpha - 0.5) * kStepLengthM;
                const double step_vel = 2.0 * phase_rate * kStepLengthM;
                const double swing_z_vel =
                    std::cos(kPi * swing_alpha) * kPi * 2.0 * phase_rate * kSwingHeightM;
                target = target + (step_dir * step_delta);
                target.z += std::sin(kPi * swing_alpha) * kSwingHeightM;
                target_vel = target_vel + (step_dir * step_vel);
                target_vel.z += swing_z_vel;
            }
        }

        // Apply commanded body orientation by rotating each foot target in body frame.
        target = body_rotation * target;
        target_vel = body_rotation * target_vel;
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    return out;
}
