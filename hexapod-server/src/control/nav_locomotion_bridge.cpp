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
    waypoints_ = waypoints;
    follow_params_ = follow_params;
    follow_ = FollowWaypoints(follow_params);
    follow_.reset(std::move(waypoints));
    active_ = true;
    paused_ = false;
    cmd_slew_have_prev_ = false;
    monitor_ = MonitorSnapshot{};
    monitor_.active = true;
    monitor_.lifecycle = LifecycleState::Running;
    monitor_.waypoint_count = waypoints_.size();
    monitor_.retry_budget = (fail_policy_ == FailPolicy::FailRetryN) ? retry_budget_ : 0;
    updateMonitorFromFollow();
}

void NavLocomotionBridge::deactivate() {
    active_ = false;
    paused_ = false;
    cmd_slew_have_prev_ = false;
    monitor_.active = false;
    monitor_.paused = false;
    monitor_.lifecycle = LifecycleState::Idle;
}

void NavLocomotionBridge::setPlanarCommandSlew01(const double alpha) {
    cmd_slew_alpha_ = std::clamp(alpha, 0.0, 1.0);
    cmd_slew_have_prev_ = false;
}

void NavLocomotionBridge::pause() {
    if (!active_) {
        return;
    }
    paused_ = true;
    monitor_.paused = true;
    monitor_.lifecycle = LifecycleState::Paused;
}

void NavLocomotionBridge::resume() {
    if (!active_) {
        return;
    }
    paused_ = false;
    monitor_.paused = false;
    monitor_.lifecycle = LifecycleState::Running;
}

void NavLocomotionBridge::cancel() {
    active_ = false;
    paused_ = false;
    cmd_slew_have_prev_ = false;
    monitor_.active = false;
    monitor_.paused = false;
    monitor_.lifecycle = LifecycleState::Cancelled;
}

void NavLocomotionBridge::setFailPolicy(const FailPolicy policy, const int retry_budget) {
    fail_policy_ = policy;
    retry_budget_ = std::max(retry_budget, 0);
    monitor_.retry_budget = (policy == FailPolicy::FailRetryN) ? retry_budget_ : 0;
}

MotionIntent NavLocomotionBridge::mergeIntent(const MotionIntent& fallback, const RobotState& est, const double dt_s) {
    if (!active_) {
        return stampIntentForControl(fallback);
    }
    if (paused_) {
        monitor_.elapsed_s += std::max(dt_s, 0.0);
        updateMonitorFromFollow();
        monitor_.paused = true;
        monitor_.lifecycle = LifecycleState::Paused;
        return stampIntentForControl(fallback);
    }

    monitor_.elapsed_s += std::max(dt_s, 0.0);

    const NavPose2d pose = navPose2dFromRobotState(est);
    NavTaskUpdate u = follow_.update(pose, dt_s);
    updateMonitorFromFollow();

    if (u.status == NavTaskStatus::Failed) {
        monitor_.failure_reason = u.failure_reason;
        return onFailedTask(fallback);
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
        paused_ = false;
        monitor_.active = false;
        monitor_.paused = false;
        monitor_.lifecycle = LifecycleState::Completed;
    }

    return stampIntentForControl(out);
}

MotionIntent NavLocomotionBridge::makeStopIntentFromWalkBase() const {
    MotionIntent out = walk_base_;
    applyNavCommandToMotionIntent(NavCommand{}, out);
    return out;
}

MotionIntent NavLocomotionBridge::onFailedTask(const MotionIntent& fallback) {
    cmd_slew_have_prev_ = false;
    if (fail_policy_ == FailPolicy::FailRetryN && monitor_.retry_count < monitor_.retry_budget && !waypoints_.empty()) {
        ++monitor_.retry_count;
        follow_ = FollowWaypoints(follow_params_);
        follow_.reset(waypoints_);
        monitor_.lifecycle = LifecycleState::Running;
        updateMonitorFromFollow();
        return stampIntentForControl(fallback);
    }

    active_ = false;
    paused_ = false;
    monitor_.active = false;
    monitor_.paused = false;
    monitor_.lifecycle = LifecycleState::Failed;

    if (fail_policy_ == FailPolicy::FailStop || fail_policy_ == FailPolicy::FailRetryN) {
        return stampIntentForControl(makeStopIntentFromWalkBase());
    }
    return stampIntentForControl(fallback);
}

void NavLocomotionBridge::updateMonitorFromFollow() {
    const FollowWaypointsProgress p = follow_.progress();
    monitor_.active = active_;
    monitor_.paused = paused_;
    monitor_.active_waypoint_index = p.waypoint_index;
    monitor_.waypoint_count = p.waypoint_count;
    monitor_.has_active_waypoint = p.has_active_waypoint;
    monitor_.distance_to_active_waypoint_m = p.distance_to_active_waypoint_m;
    monitor_.best_distance_to_active_waypoint_m = p.best_distance_to_active_waypoint_m;
    monitor_.stall_timer_s = p.stall_timer_s;
}
