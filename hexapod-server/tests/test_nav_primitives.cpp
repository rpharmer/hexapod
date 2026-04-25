#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"

#include "math_types.hpp"
#include "motion_intent_utils.hpp"
#include "types.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-5) {
    return std::abs(a - b) <= eps;
}

void step_pose(NavPose2d& p, const NavCommand& c, const double dt_s) {
    const double c0 = std::cos(p.yaw_rad);
    const double s0 = std::sin(p.yaw_rad);
    p.x_m += (c0 * c.vx_mps - s0 * c.vy_mps) * dt_s;
    p.y_m += (s0 * c.vx_mps + c0 * c.vy_mps) * dt_s;
    p.yaw_rad = navWrapAngleRad(p.yaw_rad + c.yaw_rate_radps * dt_s);
}

} // namespace

int main() {
    if (!nearlyEq(navWrapAngleRad(4.0 * kPi + 0.1), 0.1, 1e-6) ||
        !nearlyEq(navWrapAngleRad(-kPi - 0.2), kPi - 0.2, 1e-3)) {
        std::cerr << "FAIL: navWrapAngleRad\n";
        return EXIT_FAILURE;
    }

    MotionIntent intent = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.08);
    NavCommand nav{};
    nav.vx_mps = 0.12;
    nav.vy_mps = -0.05;
    nav.yaw_rate_radps = 0.2;
    applyNavCommandToMotionIntent(nav, intent);
    if (!nearlyEq(intent.cmd_vx_mps.value, 0.12) || !nearlyEq(intent.cmd_vy_mps.value, -0.05) ||
        !nearlyEq(intent.cmd_yaw_radps.value, 0.2)) {
        std::cerr << "FAIL: applyNavCommandToMotionIntent cmd fields\n";
        return EXIT_FAILURE;
    }
    const PlanarMotionCommand planar = planarMotionCommand(intent);
    if (!nearlyEq(planar.vx_mps, 0.12) || !nearlyEq(planar.vy_mps, -0.05) ||
        !nearlyEq(planar.yaw_rate_radps, 0.2)) {
        std::cerr << "FAIL: planarMotionCommand after nav apply\n";
        return EXIT_FAILURE;
    }

    RotateToHeading::Params rp{};
    rp.error_threshold_rad = 0.08;
    rp.settle_cycles_required = 4;
    rp.yaw_rate_limit_radps = 0.9;
    rp.kp = 2.0;
    RotateToHeading rot(rp);
    rot.reset(kPi / 2.0);
    NavPose2d pose{};
    pose.yaw_rad = 0.0;
    const double dt = 0.02;
    NavTaskUpdate ru{};
    for (int i = 0; i < 8000; ++i) {
        ru = rot.update(pose.yaw_rad, dt);
        pose.yaw_rad = navWrapAngleRad(pose.yaw_rad + ru.cmd.yaw_rate_radps * dt);
        if (ru.status == NavTaskStatus::Completed) {
            break;
        }
    }
    if (ru.status != NavTaskStatus::Completed || !nearlyEq(pose.yaw_rad, kPi / 2.0, 0.12)) {
        std::cerr << "FAIL: RotateToHeading did not converge\n";
        return EXIT_FAILURE;
    }

    DriveDistance::Params dp{};
    dp.position_tol_m = 0.04;
    dp.settle_cycles_required = 3;
    dp.max_v_mps = 0.25;
    dp.position_gain = 1.1;
    DriveDistance drive(dp);
    NavPose2d start{};
    drive.reset(start, 0.35, 0.0);
    pose = start;
    NavTaskUpdate du{};
    for (int i = 0; i < 20000; ++i) {
        du = drive.update(pose, dt);
        step_pose(pose, du.cmd, dt);
        if (du.status == NavTaskStatus::Completed) {
            break;
        }
    }
    if (du.status != NavTaskStatus::Completed || !nearlyEq(pose.x_m, 0.35, 0.06) ||
        !nearlyEq(pose.y_m, 0.0, 0.06)) {
        std::cerr << "FAIL: DriveDistance did not reach goal\n";
        return EXIT_FAILURE;
    }

    GoToPose::Params gp{};
    gp.rotate_first = true;
    gp.rotate.error_threshold_rad = 0.12;
    gp.rotate.settle_cycles_required = 3;
    gp.drive.position_tol_m = 0.05;
    gp.drive.settle_cycles_required = 3;
    GoToPose gt(gp);
    NavPose2d g0{};
    gt.reset(g0, 0.25, 0.1, kPi / 4.0);
    pose = g0;
    NavTaskUpdate gu{};
    for (int i = 0; i < 50000; ++i) {
        gu = gt.update(pose, dt);
        step_pose(pose, gu.cmd, dt);
        if (gu.status == NavTaskStatus::Completed) {
            break;
        }
    }
    if (gu.status != NavTaskStatus::Completed || !nearlyEq(pose.x_m, 0.25, 0.08) ||
        !nearlyEq(pose.y_m, 0.1, 0.08) || !nearlyEq(pose.yaw_rad, kPi / 4.0, 0.15)) {
        std::cerr << "FAIL: GoToPose did not reach pose\n";
        return EXIT_FAILURE;
    }

    FollowWaypoints::Params fp{};
    fp.stall_timeout_s = 100.0;
    FollowWaypoints fw(fp);
    fw.reset({NavPose2d{0.15, 0.0, 0.0}, NavPose2d{0.3, 0.05, 0.0}});
    pose = {};
    NavTaskUpdate fu{};
    for (int i = 0; i < 80000; ++i) {
        fu = fw.update(pose, dt);
        step_pose(pose, fu.cmd, dt);
        if (fu.status == NavTaskStatus::Completed || fu.status == NavTaskStatus::Failed) {
            break;
        }
    }
    if (fu.status != NavTaskStatus::Completed || !nearlyEq(pose.x_m, 0.3, 0.09) ||
        !nearlyEq(pose.y_m, 0.05, 0.09)) {
        std::cerr << "FAIL: FollowWaypoints did not finish at final waypoint\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
