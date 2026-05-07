#include "body_pose_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kDefaultBodyHeightM = 0.14;
constexpr double kLeanVxRefMps = 0.20;
constexpr double kLeanYawRefRadps = 0.45;
constexpr double kLeanPitchPerVx = 0.22;
constexpr double kLeanRollPerYaw = 0.18;
constexpr double kLeanRollPerVy = 0.14;
constexpr double kMarginSoftM = 0.022;
constexpr double kMarginHardM = 0.004;
constexpr double kSlowStrideHz = 0.78;
constexpr double kSlowStrideLeanBoost = 1.18;
constexpr double kObliqueRollBlendLimit = 0.45;

} // namespace

BodyPoseSetpoint computeBodyPoseSetpoint(const MotionIntent& intent,
                                         const PlanarMotionCommand& cmd,
                                         const double static_stability_margin_m,
                                         const double stride_phase_rate_hz) {
    BodyPoseSetpoint out{};
    out.body_height_m = intent.twist.body_trans_m.z > 1e-6 ? intent.twist.body_trans_m.z : kDefaultBodyHeightM;
    out.roll_rad = intent.twist.twist_pos_rad.x;
    out.pitch_rad = intent.twist.twist_pos_rad.y;
    out.yaw_rad = intent.twist.twist_pos_rad.z;

    double margin_scale =
        std::clamp((static_stability_margin_m - kMarginHardM) / std::max(kMarginSoftM - kMarginHardM, 1e-6), 0.0, 1.0);
    if (stride_phase_rate_hz < kSlowStrideHz) {
        margin_scale = std::min(1.0, margin_scale * kSlowStrideLeanBoost);
    }

    const double vx_n = std::clamp(cmd.vx_mps / kLeanVxRefMps, -1.2, 1.2);
    const double vy_n = std::clamp(cmd.vy_mps / kLeanVxRefMps, -1.2, 1.2);
    const double yaw_n = std::clamp(cmd.yaw_rate_radps / kLeanYawRefRadps, -1.2, 1.2);

    out.pitch_rad += -kLeanPitchPerVx * vx_n * margin_scale;

    const double yaw_roll = kLeanRollPerYaw * yaw_n * margin_scale;
    const double lateral_roll = kLeanRollPerVy * vy_n * margin_scale;
    double combined_roll = yaw_roll + lateral_roll;
    const bool same_sign = (yaw_roll >= 0.0 && lateral_roll >= 0.0) || (yaw_roll <= 0.0 && lateral_roll <= 0.0);
    if (same_sign && std::abs(yaw_roll) > 1e-6 && std::abs(lateral_roll) > 1e-6) {
        const double oblique_coupling = std::clamp(std::min(std::abs(yaw_n), std::abs(vy_n)), 0.0, 1.0);
        const double dominant = std::max(std::abs(yaw_roll), std::abs(lateral_roll));
        const double residual = std::abs(combined_roll) - dominant;
        combined_roll = std::copysign(
            dominant + residual * (1.0 - kObliqueRollBlendLimit * oblique_coupling),
            combined_roll);
    }
    out.roll_rad += combined_roll;

    return out;
}
