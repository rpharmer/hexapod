#include "contact_foot_response.hpp"

#include <algorithm>
#include <cmath>

namespace contact_foot_response {
namespace {

constexpr double kLateTouchdownTau = 0.86;
constexpr double kMaxSwingDownExtensionM = 0.014;
constexpr double kLevelGainMPerRad = 0.055;
constexpr double kMaxLevelDzM = 0.014;
constexpr double kTiltDeadzoneRad = 0.004;

} // namespace

bool sensorsTrustedForContactResponse(const RobotState& est) {
    return est.valid || est.has_valid_flag;
}

void adjustSwingTauAndVerticalExtension(const bool in_swing,
                                        const bool foot_contact,
                                        const RobotState& est,
                                        const double tau01,
                                        double& tau_out,
                                        double& extra_down_z_m) {
    tau_out = std::clamp(tau01, 0.0, 1.0);
    extra_down_z_m = 0.0;

    if (!in_swing || !sensorsTrustedForContactResponse(est)) {
        return;
    }

    if (foot_contact) {
        tau_out = 1.0;
        return;
    }

    if (tau01 >= kLateTouchdownTau) {
        const double span = std::max(1.0 - kLateTouchdownTau, 1e-6);
        const double s = std::clamp((tau01 - kLateTouchdownTau) / span, 0.0, 1.0);
        extra_down_z_m = kMaxSwingDownExtensionM * s;
    }
}

double stanceTiltLevelingDeltaZ(const RobotState& est,
                               const MotionIntent& intent,
                               const double anchor_x,
                               const double anchor_y) {
    if (!est.has_body_twist_state) {
        return 0.0;
    }

    const double er = est.body_twist_state.twist_pos_rad.x - intent.twist.twist_pos_rad.x;
    const double ep = est.body_twist_state.twist_pos_rad.y - intent.twist.twist_pos_rad.y;
    if (std::abs(er) + std::abs(ep) < kTiltDeadzoneRad) {
        return 0.0;
    }

    const double dz =
        -kLevelGainMPerRad * (er * anchor_y + ep * anchor_x);
    return std::clamp(dz, -kMaxLevelDzM, kMaxLevelDzM);
}

} // namespace contact_foot_response
