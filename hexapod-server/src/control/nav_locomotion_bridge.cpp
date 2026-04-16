#include "nav_locomotion_bridge.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <utility>

namespace {

MotionIntent stampIntentForControl(MotionIntent intent) {
    stampIntentStreamMotionFields(intent);
    return intent;
}

} // namespace

void NavLocomotionBridge::startFollowWaypoints(MotionIntent walk_base, std::vector<NavPose2d> waypoints) {
    startFollowWaypoints(std::move(walk_base), std::move(waypoints), FollowWaypoints::Params{});
}

void NavLocomotionBridge::startFollowWaypoints(MotionIntent walk_base,
                                              std::vector<NavPose2d> waypoints,
                                              const FollowWaypoints::Params& follow_params) {
    walk_base_ = std::move(walk_base);
    follow_ = FollowWaypoints(follow_params);
    follow_.reset(std::move(waypoints));
    active_ = true;
    cmd_slew_have_prev_ = false;
}

void NavLocomotionBridge::deactivate() {
    active_ = false;
    cmd_slew_have_prev_ = false;
}

void NavLocomotionBridge::setPlanarCommandSlew01(const double alpha) {
    cmd_slew_alpha_ = std::clamp(alpha, 0.0, 1.0);
    cmd_slew_have_prev_ = false;
}

MotionIntent NavLocomotionBridge::mergeIntent(const MotionIntent& fallback, const RobotState& est, const double dt_s) {
    if (!active_) {
        return stampIntentForControl(fallback);
    }

    const NavPose2d pose = navPose2dFromRobotState(est);
    NavTaskUpdate u = follow_.update(pose, dt_s);

    if (u.status == NavTaskStatus::Failed) {
        active_ = false;
        cmd_slew_have_prev_ = false;
        return stampIntentForControl(fallback);
    }

    NavCommand cmd = u.cmd;
    if (cmd_slew_alpha_ > 0.0 && cmd_slew_have_prev_) {
        const double a = cmd_slew_alpha_;
        cmd.vx_mps = prev_nav_vx_ * (1.0 - a) + cmd.vx_mps * a;
        cmd.vy_mps = prev_nav_vy_ * (1.0 - a) + cmd.vy_mps * a;
        cmd.yaw_rate_radps = prev_nav_w_ * (1.0 - a) + cmd.yaw_rate_radps * a;
    }
    prev_nav_vx_ = cmd.vx_mps;
    prev_nav_vy_ = cmd.vy_mps;
    prev_nav_w_ = cmd.yaw_rate_radps;
    cmd_slew_have_prev_ = true;

    MotionIntent out = walk_base_;
    applyNavCommandToMotionIntent(cmd, out);

    if (u.status == NavTaskStatus::Completed) {
        active_ = false;
    }

    return stampIntentForControl(out);
}
