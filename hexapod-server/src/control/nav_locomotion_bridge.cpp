#include "nav_locomotion_bridge.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
    outer_i_fwd_ = 0.0;
    outer_i_lat_ = 0.0;
    outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
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
    outer_i_fwd_ = 0.0;
    outer_i_lat_ = 0.0;
    outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
    monitor_.active = false;
    monitor_.paused = false;
    monitor_.lifecycle = LifecycleState::Idle;
}

void NavLocomotionBridge::setPlanarCommandSlew01(const double alpha) {
    cmd_slew_alpha_ = std::clamp(alpha, 0.0, 1.0);
    cmd_slew_have_prev_ = false;
}

void NavLocomotionBridge::setBodyFramePositionIntegralGains(const double ki_fwd_per_s,
                                                          const double ki_lat_per_s,
                                                          const double integral_abs_cap_m_s) {
    outer_ki_fwd_ = std::max(0.0, ki_fwd_per_s);
    outer_ki_lat_ = std::max(0.0, ki_lat_per_s);
    outer_i_cap_ = std::max(0.0, integral_abs_cap_m_s);
    outer_i_fwd_ = 0.0;
    outer_i_lat_ = 0.0;
    outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
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
    outer_i_fwd_ = 0.0;
    outer_i_lat_ = 0.0;
    outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
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
    applyBodyFramePositionIntegralOuterLoop(pose, dt_s, cmd);
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
        outer_i_fwd_ = 0.0;
        outer_i_lat_ = 0.0;
        outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
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
        outer_i_fwd_ = 0.0;
        outer_i_lat_ = 0.0;
        outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
        monitor_.lifecycle = LifecycleState::Running;
        updateMonitorFromFollow();
        return stampIntentForControl(fallback);
    }

    active_ = false;
    paused_ = false;
    outer_i_fwd_ = 0.0;
    outer_i_lat_ = 0.0;
    outer_last_wp_index_ = std::numeric_limits<std::size_t>::max();
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

void NavLocomotionBridge::applyBodyFramePositionIntegralOuterLoop(const NavPose2d& pose,
                                                                  const double dt_s,
                                                                  NavCommand& cmd) {
    if (outer_ki_fwd_ <= 0.0 && outer_ki_lat_ <= 0.0) {
        return;
    }
    if (dt_s <= 0.0 || waypoints_.empty()) {
        return;
    }
    const std::size_t idx = monitor_.active_waypoint_index;
    if (idx >= waypoints_.size()) {
        return;
    }
    if (idx != outer_last_wp_index_) {
        outer_i_fwd_ = 0.0;
        outer_i_lat_ = 0.0;
        outer_last_wp_index_ = idx;
    }

    const NavPose2d& wp = waypoints_[idx];
    const double ex = wp.x_m - pose.x_m;
    const double ey = wp.y_m - pose.y_m;
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    const double e_fwd = c * ex + s * ey;
    const double e_lat = -s * ex + c * ey;

    outer_i_fwd_ += e_fwd * dt_s;
    outer_i_lat_ += e_lat * dt_s;
    if (outer_i_cap_ > 0.0) {
        outer_i_fwd_ = std::clamp(outer_i_fwd_, -outer_i_cap_, outer_i_cap_);
        outer_i_lat_ = std::clamp(outer_i_lat_, -outer_i_cap_, outer_i_cap_);
    }

    cmd.vx_mps += outer_ki_fwd_ * outer_i_fwd_;
    cmd.vy_mps += outer_ki_lat_ * outer_i_lat_;

    constexpr double kMaxPlanarMps = 0.55;
    const double h = std::hypot(cmd.vx_mps, cmd.vy_mps);
    if (h > kMaxPlanarMps && h > 1e-9) {
        const double sc = kMaxPlanarMps / h;
        cmd.vx_mps *= sc;
        cmd.vy_mps *= sc;
    }
}
