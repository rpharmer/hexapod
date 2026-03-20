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

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];
        if (walking) {
            const double phase = clamp01(gait.phase[leg]);
            if (gait.in_stance[leg]) {
                const double stance_alpha = clamp01(phase / 0.5);
                const double x_delta = (0.5 - stance_alpha) * kStepLengthM;
                target.x += x_delta;
            } else {
                const double swing_alpha = clamp01((phase - 0.5) / 0.5);
                const double x_delta = (swing_alpha - 0.5) * kStepLengthM;
                target.x += x_delta;
                target.z += std::sin(kPi * swing_alpha) * kSwingHeightM;
            }
        }

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = Vec3{};
    }

    return out;
}
