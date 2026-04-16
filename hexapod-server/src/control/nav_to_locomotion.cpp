#include "nav_to_locomotion.hpp"

#include "types.hpp"

#include <cmath>

NavPose2d navPose2dFromRobotState(const RobotState& est) {
    if (!est.has_body_twist_state) {
        return NavPose2d{};
    }
    return NavPose2d{est.body_twist_state.body_trans_m.x,
                     est.body_twist_state.body_trans_m.y,
                     est.body_twist_state.twist_pos_rad.z};
}

void applyNavCommandToMotionIntent(const NavCommand& nav, MotionIntent& intent) {
    intent.cmd_vx_mps = LinearRateMps{nav.vx_mps};
    intent.cmd_vy_mps = LinearRateMps{nav.vy_mps};
    intent.cmd_yaw_radps = AngularRateRadPerSec{nav.yaw_rate_radps};

    const double planar = std::hypot(nav.vx_mps, nav.vy_mps);
    if (planar > 1e-9) {
        intent.speed_mps = LinearRateMps{planar};
        intent.heading_rad = AngleRad{std::atan2(nav.vy_mps, nav.vx_mps)};
    } else {
        intent.speed_mps = LinearRateMps{};
        intent.heading_rad = AngleRad{};
    }
}
