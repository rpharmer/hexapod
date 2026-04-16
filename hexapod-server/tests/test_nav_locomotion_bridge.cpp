#include "nav_locomotion_bridge.hpp"
#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-5) {
    return std::abs(a - b) <= eps;
}

void step_pose_from_motion_intent(NavPose2d& sim, const MotionIntent& m, const double dt_s) {
    const PlanarMotionCommand p = planarMotionCommand(m);
    const double c0 = std::cos(sim.yaw_rad);
    const double s0 = std::sin(sim.yaw_rad);
    sim.x_m += (c0 * p.vx_mps - s0 * p.vy_mps) * dt_s;
    sim.y_m += (s0 * p.vx_mps + c0 * p.vy_mps) * dt_s;
    sim.yaw_rad = navWrapAngleRad(sim.yaw_rad + p.yaw_rate_radps * dt_s);
}

void fill_state_xy_yaw(RobotState& est, const NavPose2d& sim) {
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.x = sim.x_m;
    est.body_twist_state.body_trans_m.y = sim.y_m;
    est.body_twist_state.body_trans_m.z = 0.0;
    est.body_twist_state.twist_pos_rad.z = sim.yaw_rad;
}

} // namespace

int main() {
    RobotState est{};
    fill_state_xy_yaw(est, NavPose2d{1.0, -0.5, 0.25});
    const NavPose2d pose = navPose2dFromRobotState(est);
    if (!nearlyEq(pose.x_m, 1.0) || !nearlyEq(pose.y_m, -0.5) || !nearlyEq(pose.yaw_rad, 0.25)) {
        std::cerr << "FAIL: navPose2dFromRobotState\n";
        return EXIT_FAILURE;
    }

    est.has_body_twist_state = false;
    const NavPose2d zero = navPose2dFromRobotState(est);
    if (!nearlyEq(zero.x_m, 0.0) || !nearlyEq(zero.y_m, 0.0) || !nearlyEq(zero.yaw_rad, 0.0)) {
        std::cerr << "FAIL: navPose2dFromRobotState without twist should be zero\n";
        return EXIT_FAILURE;
    }

    MotionIntent fallback = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.08);
    MotionIntent walk = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.08);

    NavLocomotionBridge bridge;
    bridge.startFollowWaypoints(
        walk, {NavPose2d{0.12, 0.0, 0.0}, NavPose2d{0.22, 0.04, 0.0}});

    NavPose2d sim{};
    const double dt = 0.02;
    MotionIntent last = fallback;
    for (int i = 0; i < 12000; ++i) {
        fill_state_xy_yaw(est, sim);
        last = bridge.mergeIntent(fallback, est, dt);
        if (!bridge.active()) {
            break;
        }
        step_pose_from_motion_intent(sim, last, dt);
    }

    if (bridge.active()) {
        std::cerr << "FAIL: navigation should complete\n";
        return EXIT_FAILURE;
    }
    if (!nearlyEq(sim.x_m, 0.22, 0.07) || !nearlyEq(sim.y_m, 0.04, 0.07)) {
        std::cerr << "FAIL: sim pose did not reach final waypoint xy=" << sim.x_m << "," << sim.y_m << "\n";
        return EXIT_FAILURE;
    }

    fill_state_xy_yaw(est, sim);
    const MotionIntent idle = bridge.mergeIntent(fallback, est, dt);
    if (idle.requested_mode != RobotMode::STAND) {
        std::cerr << "FAIL: mergeIntent should return fallback when inactive\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
