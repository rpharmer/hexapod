#include "body_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kStepLengthM = 0.06;
constexpr double kSwingHeightM = 0.03;
constexpr double kDefaultBodyHeightM = 0.20;

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

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

LegTargets BodyController::update(const EstimatedState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();

    (void)est;

    const std::array<Vec3, kNumLegs> nominal = nominalStance();
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    const double heading = intent.heading_rad.value;
    const Vec3 step_dir{std::cos(heading), std::sin(heading), 0.0};
    const double yaw_cmd = intent.twist.twist_pos_rad.z;
    const Mat3 yaw_rotation = Mat3::rotZ(yaw_cmd);

    double commanded_body_height_m = intent.twist.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];

        // Apply commanded body-height offset around the nominal stance height.
        target.z += commanded_body_height_m - kDefaultBodyHeightM;

        if (walking) {
            const double phase = clamp01(gait.phase[leg]);
            if (gait.in_stance[leg]) {
                const double stance_alpha = clamp01(phase / 0.5);
                const double step_delta = (0.5 - stance_alpha) * kStepLengthM;
                target = target + (step_dir * step_delta);
            } else {
                const double swing_alpha = clamp01((phase - 0.5) / 0.5);
                const double step_delta = (swing_alpha - 0.5) * kStepLengthM;
                target = target + (step_dir * step_delta);
                target.z += std::sin(kPi * swing_alpha) * kSwingHeightM;
            }
        }

        // Apply commanded body yaw by rotating each foot target in body frame.
        target = yaw_rotation * target;

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = Vec3{};
    }

    return out;
}
