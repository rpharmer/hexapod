#include "nav_locomotion_bridge.hpp"
#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEq(double a, double b, double eps = 1e-5) {
    return std::abs(a - b) <= eps;
}

void step_pose_from_motion_intent(NavPose2d& sim, const MotionIntent& m, const double dt_s) {
    const PlanarMotionCommand p = planarMotionCommand(m);
    const double world_vx = p.vx_mps;
    const double world_vy = p.vy_mps;
    const double c0 = std::cos(sim.yaw_rad);
    const double s0 = std::sin(sim.yaw_rad);
    sim.x_m += (c0 * world_vx - s0 * world_vy) * dt_s;
    sim.y_m += (s0 * world_vx + c0 * world_vy) * dt_s;
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

    if (!expect(!bridge.active(), "navigation should complete")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEq(sim.x_m, 0.22, 0.07) && nearlyEq(sim.y_m, 0.04, 0.07),
                "sim pose should approach final waypoint")) {
        return EXIT_FAILURE;
    }
    const auto done_mon = bridge.monitor();
    if (!expect(done_mon.lifecycle == NavLocomotionBridge::LifecycleState::Completed,
                "monitor should report completed lifecycle")) {
        return EXIT_FAILURE;
    }

    // Optional body-frame I outer loop on planar command (Ki>0); default Ki=0 elsewhere.
    NavLocomotionBridge outer_bridge;
    outer_bridge.setBodyFramePositionIntegralGains(0.12, 0.12, 0.25);
    outer_bridge.startFollowWaypoints(walk, {NavPose2d{0.12, 0.0, 0.0}, NavPose2d{0.22, 0.04, 0.0}});
    NavPose2d sim_outer{};
    for (int i = 0; i < 12000; ++i) {
        fill_state_xy_yaw(est, sim_outer);
        last = outer_bridge.mergeIntent(fallback, est, dt);
        if (!outer_bridge.active()) {
            break;
        }
        const PlanarMotionCommand pl = planarMotionCommand(last);
        if (!expect(std::hypot(pl.vx_mps, pl.vy_mps) <= 0.56,
                    "outer loop should clamp planar speed")) {
            return EXIT_FAILURE;
        }
        step_pose_from_motion_intent(sim_outer, last, dt);
    }
    if (!expect(!outer_bridge.active(), "outer-loop nav should complete")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEq(sim_outer.x_m, 0.22, 0.08) && nearlyEq(sim_outer.y_m, 0.04, 0.08),
                "sim pose should approach final waypoint with outer loop enabled")) {
        return EXIT_FAILURE;
    }

    fill_state_xy_yaw(est, sim);
    const MotionIntent idle = bridge.mergeIntent(fallback, est, dt);
    if (!expect(idle.requested_mode == RobotMode::STAND,
                "mergeIntent should return fallback when inactive")) {
        return EXIT_FAILURE;
    }

    // Pause/resume/cancel behavior.
    NavLocomotionBridge ctrl_bridge;
    ctrl_bridge.startFollowWaypoints(walk, {NavPose2d{0.3, 0.0, 0.0}});
    fill_state_xy_yaw(est, NavPose2d{});
    ctrl_bridge.pause();
    const MotionIntent paused = ctrl_bridge.mergeIntent(fallback, est, dt);
    if (!expect(paused.requested_mode == RobotMode::STAND,
                "paused bridge should hold fallback intent")) {
        return EXIT_FAILURE;
    }
    if (!expect(ctrl_bridge.monitor().lifecycle == NavLocomotionBridge::LifecycleState::Paused,
                "monitor should show paused state")) {
        return EXIT_FAILURE;
    }
    ctrl_bridge.resume();
    const MotionIntent resumed = ctrl_bridge.mergeIntent(fallback, est, dt);
    if (!expect(resumed.requested_mode == RobotMode::WALK,
                "resumed bridge should emit walk/nav intent")) {
        return EXIT_FAILURE;
    }
    ctrl_bridge.cancel();
    if (!expect(ctrl_bridge.monitor().lifecycle == NavLocomotionBridge::LifecycleState::Cancelled,
                "cancel should set cancelled lifecycle")) {
        return EXIT_FAILURE;
    }

    // Fail policy: stop-on-fail should emit zeroed walk command.
    NavLocomotionBridge fail_bridge;
    FollowWaypoints::Params fp{};
    fp.stall_timeout_s = 0.01;
    fp.go_to.rotate_first = false;
    fail_bridge.setFailPolicy(NavLocomotionBridge::FailPolicy::FailStop);
    fail_bridge.startFollowWaypoints(walk, {NavPose2d{10.0, 0.0, 0.0}}, fp);
    fill_state_xy_yaw(est, NavPose2d{});
    MotionIntent failed_out = fallback;
    for (int i = 0; i < 8; ++i) {
        failed_out = fail_bridge.mergeIntent(fallback, est, dt);
    }
    const PlanarMotionCommand failed_planar = planarMotionCommand(failed_out);
    if (!expect(!fail_bridge.active(), "fail-stop should deactivate bridge")) {
        return EXIT_FAILURE;
    }
    if (!expect(fail_bridge.monitor().lifecycle == NavLocomotionBridge::LifecycleState::Failed,
                "fail-stop should report failed lifecycle")) {
        return EXIT_FAILURE;
    }
    if (!expect(fail_bridge.monitor().failure_reason == NavTaskFailureReason::StallTimeout,
                "fail-stop should report stall timeout reason")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEq(failed_planar.vx_mps, 0.0) && nearlyEq(failed_planar.vy_mps, 0.0) &&
                    nearlyEq(failed_planar.yaw_rate_radps, 0.0),
                "fail-stop output should command zero planar velocity")) {
        return EXIT_FAILURE;
    }

    // Retry policy should attempt restarts before declaring failure.
    NavLocomotionBridge retry_bridge;
    retry_bridge.setFailPolicy(NavLocomotionBridge::FailPolicy::FailRetryN, 2);
    retry_bridge.startFollowWaypoints(walk, {NavPose2d{10.0, 0.0, 0.0}}, fp);
    for (int i = 0; i < 40; ++i) {
        (void)retry_bridge.mergeIntent(fallback, est, dt);
    }
    if (!expect(retry_bridge.monitor().retry_count == 2,
                "retry policy should consume retry budget")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
