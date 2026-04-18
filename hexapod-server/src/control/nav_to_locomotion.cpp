#include "nav_to_locomotion.hpp"

#include "types.hpp"

#include <cmath>

namespace {

// The navigation stack standardizes on +X forward / +Y left. The downstream locomotion
// pipeline still drives the live plant with the opposite sign on its X channel, so keep
// the contract correction isolated at this seam instead of spreading axis quirks through nav.
constexpr double kLocomotionForwardSignX = -1.0;

} // namespace

NavPose2d navPose2dFromRobotState(const RobotState& est) {
    if (!est.has_body_twist_state) {
        return NavPose2d{};
    }
    return NavPose2d{est.body_twist_state.body_trans_m.x,
                     est.body_twist_state.body_trans_m.y,
                     est.body_twist_state.twist_pos_rad.z};
}

void applyNavCommandToMotionIntent(const NavCommand& nav, MotionIntent& intent) {
    const double loco_vx_mps = kLocomotionForwardSignX * nav.vx_mps;
    intent.cmd_vx_mps = LinearRateMps{loco_vx_mps};
    intent.cmd_vy_mps = LinearRateMps{nav.vy_mps};
    intent.cmd_yaw_radps = AngularRateRadPerSec{nav.yaw_rate_radps};

    const double planar = std::hypot(loco_vx_mps, nav.vy_mps);
    if (planar > 1e-9) {
        intent.speed_mps = LinearRateMps{planar};
        intent.heading_rad = AngleRad{std::atan2(nav.vy_mps, loco_vx_mps)};
    } else {
        intent.speed_mps = LinearRateMps{};
        intent.heading_rad = AngleRad{};
    }
}
