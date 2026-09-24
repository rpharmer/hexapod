#include "navigation_manager.hpp"

#include "motion_intent_utils.hpp"
#include "nav_to_locomotion.hpp"

#include <cmath>
#include <utility>

namespace {

double secondsBetween(const TimePointUs newer, const TimePointUs older) {
    if (older.isZero() || newer.value <= older.value) {
        return 0.0;
    }
    return static_cast<double>(newer.value - older.value) * 1e-6;
}

double pathLength(const std::vector<NavPose2d>& waypoints) {
    double total = 0.0;
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        total += std::hypot(waypoints[i].x_m - waypoints[i - 1].x_m,
                            waypoints[i].y_m - waypoints[i - 1].y_m);
    }
    return total;
}

} // namespace

NavigationManager::NavigationManager(LocalMapConfig local_map_config,
                                     LocalPlannerConfig local_planner_config,
                                     control_config::NavBridgeConfig nav_bridge_config,
                                     std::unique_ptr<ILocalPlanner> planner)
    : local_map_builder_(std::move(local_map_config)),
      planner_config_(std::move(local_planner_config)),
      nav_bridge_config_(std::move(nav_bridge_config)),
      planner_(std::move(planner)) {
    bridge_.setFailPolicy(NavLocomotionBridge::FailPolicy::FailStop);
    bridge_.setPlanarCommandSlew01(0.10);
    bridge_.setBodyFramePositionIntegralGains(nav_bridge_config_.body_frame_integral_ki_fwd_per_s,
                                              nav_bridge_config_.body_frame_integral_ki_lat_per_s,
                                              nav_bridge_config_.body_frame_integral_abs_cap_m_s);
}

void NavigationManager::addObservationSource(std::shared_ptr<ILocalMapObservationSource> source) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (source) {
        observation_sources_.push_back(std::move(source));
    }
}

void NavigationManager::clearObservationSources() {
    const std::lock_guard<std::mutex> lock(mutex_);
    observation_sources_.clear();
}

void NavigationManager::startNavigateToPose(MotionIntent walk_base,
                                            NavPose2d goal_pose,
                                            FollowWaypoints::Params follow_params) {
    const std::lock_guard<std::mutex> lock(mutex_);
    walk_base_ = std::move(walk_base);
    goal_pose_ = goal_pose;
    follow_params_ = follow_params;
    map_aware_mode_ = true;
    raw_waypoints_.clear();
    active_segment_.clear();
    blocked_since_ = TimePointUs{};
    last_merge_timestamp_ = TimePointUs{};
    local_map_builder_.reset();
    bridge_.deactivate();
    monitor_ = NavigationMonitorSnapshot{};
    monitor_.active = true;
    monitor_.lifecycle = NavigationLifecycleState::Running;
    monitor_.has_goal = true;
    monitor_.goal = goal_pose_;
}

void NavigationManager::startRawFollowWaypoints(MotionIntent walk_base,
                                                std::vector<NavPose2d> waypoints,
                                                FollowWaypoints::Params follow_params) {
    const std::lock_guard<std::mutex> lock(mutex_);
    walk_base_ = std::move(walk_base);
    follow_params_ = follow_params;
    raw_waypoints_ = std::move(waypoints);
    map_aware_mode_ = false;
    active_segment_ = raw_waypoints_;
    blocked_since_ = TimePointUs{};
    last_merge_timestamp_ = TimePointUs{};
    bridge_.startFollowWaypoints(walk_base_, raw_waypoints_, follow_params_);
    monitor_ = NavigationMonitorSnapshot{};
    monitor_.active = true;
    monitor_.lifecycle = NavigationLifecycleState::Running;
    monitor_.planner_status = LocalPlanStatus::Ready;
    monitor_.active_segment_waypoint_count = active_segment_.size();
    monitor_.active_segment_length_m = pathLength(active_segment_);
    refreshMonitorFromBridge();
}

void NavigationManager::pause() {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!monitor_.active) {
        return;
    }
    monitor_.paused = true;
    monitor_.lifecycle = NavigationLifecycleState::Paused;
    bridge_.pause();
}

void NavigationManager::resume() {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!monitor_.active) {
        return;
    }
    monitor_.paused = false;
    monitor_.lifecycle = NavigationLifecycleState::Running;
    bridge_.resume();
}

void NavigationManager::cancel() {
    const std::lock_guard<std::mutex> lock(mutex_);
    monitor_.active = false;
    monitor_.paused = false;
    monitor_.lifecycle = NavigationLifecycleState::Cancelled;
    active_segment_.clear();
    bridge_.cancel();
}

void NavigationManager::deactivate() {
    const std::lock_guard<std::mutex> lock(mutex_);
    monitor_ = NavigationMonitorSnapshot{};
    bridge_.deactivate();
    active_segment_.clear();
    blocked_since_ = TimePointUs{};
}

void NavigationManager::reset() {
    const std::lock_guard<std::mutex> lock(mutex_);
    local_map_builder_.reset();
    cached_map_snapshot_ = LocalMapSnapshot{};
    blocked_since_ = TimePointUs{};
    last_merge_timestamp_ = TimePointUs{};
    last_terrain_refresh_timestamp_ = TimePointUs{};
    last_terrain_refresh_sample_id_ = 0;
    active_segment_.clear();
    monitor_.block_reason = PlannerBlockReason::None;
    monitor_.blocked_elapsed_s = 0.0;
    monitor_.map_fresh = false;
    monitor_.nearest_obstacle_distance_m = -1.0;

    const bool should_resume_raw_bridge = !map_aware_mode_ && !raw_waypoints_.empty() && monitor_.active;
    if (should_resume_raw_bridge) {
        bridge_.deactivate();
        active_segment_ = raw_waypoints_;
        bridge_.startFollowWaypoints(walk_base_, raw_waypoints_, follow_params_);
        if (monitor_.paused) {
            bridge_.pause();
        }
        refreshMonitorFromBridge();
        monitor_.planner_status = LocalPlanStatus::Ready;
        monitor_.block_reason = PlannerBlockReason::None;
        monitor_.lifecycle = monitor_.paused ? NavigationLifecycleState::Paused : NavigationLifecycleState::Running;
        monitor_.active = true;
    } else {
        bridge_.deactivate();
        monitor_.active_segment_waypoint_count = 0;
        monitor_.active_segment_length_m = 0.0;
        monitor_.planner_status = LocalPlanStatus::Ready;
        monitor_.bridge = bridge_.monitor();
        if (monitor_.active) {
            monitor_.lifecycle = monitor_.paused ? NavigationLifecycleState::Paused
                                                 : NavigationLifecycleState::Running;
        }
    }
}

void NavigationManager::refreshTerrainSnapshot(const RobotState& est, const TimePointUs now) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const NavPose2d pose = navPose2dFromRobotState(est);
    (void)refreshTerrainSnapshotLocked(pose, est, now);
}

MotionIntent NavigationManager::mergeIntent(const MotionIntent& fallback,
                                            const RobotState& est,
                                            const TimePointUs now) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!monitor_.active) {
        return fallback;
    }
    if (monitor_.paused) {
        MotionIntent paused_intent = fallback;
        stampIntentStreamMotionFields(paused_intent);
        return paused_intent;
    }

    const NavPose2d pose = navPose2dFromRobotState(est);
    const double dt_s = secondsBetween(now, last_merge_timestamp_);
    last_merge_timestamp_ = now;

    if (!map_aware_mode_) {
        MotionIntent out = bridge_.mergeIntent(fallback, est, dt_s);
        refreshMonitorFromBridge();
        if (!bridge_.active() &&
            monitor_.bridge.lifecycle == NavLocomotionBridge::LifecycleState::Completed) {
            monitor_.active = false;
            monitor_.lifecycle = NavigationLifecycleState::Completed;
        } else if (!bridge_.active() &&
                   monitor_.bridge.lifecycle == NavLocomotionBridge::LifecycleState::Failed) {
            monitor_.active = false;
            monitor_.lifecycle = NavigationLifecycleState::Failed;
        }
        stampIntentStreamMotionFields(out);
        return out;
    }

    const LocalMapSnapshot snapshot = refreshTerrainSnapshotLocked(pose, est, now);

    if (snapshot.has_primary_observations && !snapshot.fresh) {
        monitor_.lifecycle = NavigationLifecycleState::MapUnavailable;
        monitor_.planner_status = LocalPlanStatus::MapUnavailable;
        bridge_.deactivate();
        MotionIntent out = makeStopIntent();
        stampIntentStreamMotionFields(out);
        return out;
    }

    const bool need_replan =
        !bridge_.active() ||
        monitor_.bridge.lifecycle == NavLocomotionBridge::LifecycleState::Failed ||
        activeSegmentBlocked(snapshot, pose) ||
        (bridge_.active() &&
         monitor_.bridge.stall_timer_s >=
             std::max(0.25, follow_params_.stall_timeout_s * 0.5));

    if (need_replan) {
        planOrBlock(pose, snapshot, now);
    }

    if (monitor_.lifecycle == NavigationLifecycleState::Blocked) {
        monitor_.blocked_elapsed_s = secondsBetween(now, blocked_since_);
        if (monitor_.blocked_elapsed_s >= planner_config_.blocked_timeout_s) {
            monitor_.lifecycle = NavigationLifecycleState::Failed;
            monitor_.active = false;
        }
        MotionIntent out = makeStopIntent();
        stampIntentStreamMotionFields(out);
        return out;
    }

    if (monitor_.lifecycle == NavigationLifecycleState::Completed) {
        MotionIntent out = makeStopIntent();
        stampIntentStreamMotionFields(out);
        return out;
    }

    MotionIntent out = bridge_.mergeIntent(fallback, est, dt_s);
    refreshMonitorFromBridge();
    if (!snapshot.has_observations && terminalGoalReached(pose)) {
        bridge_.deactivate();
        monitor_.active = false;
        monitor_.lifecycle = NavigationLifecycleState::Completed;
        out = makeStopIntent();
    } else if (!bridge_.active() &&
               monitor_.bridge.lifecycle == NavLocomotionBridge::LifecycleState::Completed &&
               terminalGoalReached(pose)) {
        monitor_.active = false;
        monitor_.lifecycle = NavigationLifecycleState::Completed;
        out = makeStopIntent();
    } else if (!bridge_.active() &&
               monitor_.bridge.lifecycle == NavLocomotionBridge::LifecycleState::Failed) {
        planOrBlock(pose, snapshot, now);
        if (monitor_.lifecycle == NavigationLifecycleState::Blocked) {
            out = makeStopIntent();
        }
    }

    stampIntentStreamMotionFields(out);
    return out;
}

MotionIntent NavigationManager::makeStopIntent() const {
    MotionIntent out = walk_base_;
    applyNavCommandToMotionIntent(NavCommand{}, out);
    return out;
}

std::vector<LocalMapObservation> NavigationManager::collectObservations(const NavPose2d& pose,
                                                                        const RobotState& est,
                                                                        const TimePointUs now) const {
    std::vector<LocalMapObservation> out{};
    out.reserve(observation_sources_.size());
    for (const std::shared_ptr<ILocalMapObservationSource>& source : observation_sources_) {
        if (!source) {
            continue;
        }
        out.push_back(source->collect(pose, est, now));
    }
    return out;
}

bool NavigationManager::activeSegmentBlocked(const LocalMapSnapshot& snapshot,
                                             const NavPose2d& current_pose) const {
    if (active_segment_.empty() || snapshot.inflated.empty()) {
        return false;
    }

    const double sample_step_m = std::max(0.02, snapshot.inflated.resolution_m * 0.5);
    const std::size_t first = std::min(monitor_.bridge.active_waypoint_index, active_segment_.size());
    for (std::size_t i = first; i < active_segment_.size(); ++i) {
        // The remaining route begins at the robot, not the preceding path
        // node. This also checks a direct route with just one destination.
        const NavPose2d& a = i == first ? current_pose : active_segment_[i - 1];
        const NavPose2d& b = active_segment_[i];
        const double length = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_step_m)));
        for (int s = 0; s <= samples; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(samples);
            const double x = a.x_m + (b.x_m - a.x_m) * t;
            const double y = a.y_m + (b.y_m - a.y_m) * t;
            int cell_x = 0;
            int cell_y = 0;
            if (!snapshot.inflated.worldToCell(x, y, cell_x, cell_y)) {
                continue;
            }
            if (snapshot.inflated.stateAtCell(cell_x, cell_y) == LocalMapCellState::Occupied) {
                return true;
            }
        }
    }

    return false;
}

bool NavigationManager::terminalGoalReached(const NavPose2d& pose) const {
    const double position_tol_m = std::max(0.03, local_map_builder_.config().resolution_m);
    const double yaw_tol_rad = std::max(0.06, follow_params_.go_to.rotate.error_threshold_rad);
    return std::hypot(goal_pose_.x_m - pose.x_m, goal_pose_.y_m - pose.y_m) <= position_tol_m &&
           std::abs(navWrapAngleRad(goal_pose_.yaw_rad - pose.yaw_rad)) <= yaw_tol_rad;
}

void NavigationManager::planOrBlock(const NavPose2d& pose,
                                    const LocalMapSnapshot& snapshot,
                                    const TimePointUs now) {
    if (!planner_) {
        monitor_.lifecycle = NavigationLifecycleState::Failed;
        monitor_.active = false;
        return;
    }

    const LocalPlanResult plan = planner_->plan(LocalPlanRequest{pose, goal_pose_, snapshot});
    // Count a newly blocked route as a replan, but not every control tick
    // spent retrying the same blocked route.
    if (plan.status == LocalPlanStatus::Ready ||
        monitor_.lifecycle != NavigationLifecycleState::Blocked) {
        monitor_.replan_count += 1;
    }
    monitor_.planner_status = plan.status;
    monitor_.block_reason = plan.block_reason;
    monitor_.last_plan_timestamp = now;

    if (plan.status != LocalPlanStatus::Blocked &&
        plan.status != LocalPlanStatus::MapUnavailable && terminalGoalReached(pose)) {
        bridge_.deactivate();
        active_segment_.clear();
        monitor_.active = false;
        monitor_.lifecycle = NavigationLifecycleState::Completed;
        return;
    }

    if (plan.status == LocalPlanStatus::GoalReached) {
        // A* is an XY planner. At the requested position, the pose goal still
        // needs its final heading; do not report completion from XY alone.
        int cell_x = 0;
        int cell_y = 0;
        if (!snapshot.inflated.worldToCell(pose.x_m, pose.y_m, cell_x, cell_y) ||
            snapshot.inflated.stateAtCell(cell_x, cell_y) == LocalMapCellState::Occupied) {
            bridge_.deactivate();
            active_segment_.clear();
            if (blocked_since_.isZero()) blocked_since_ = now;
            monitor_.planner_status = LocalPlanStatus::Blocked;
            monitor_.block_reason = PlannerBlockReason::StartOccupied;
            monitor_.lifecycle = NavigationLifecycleState::Blocked;
            monitor_.active = true;
            return;
        }
        active_segment_ = {NavPose2d{pose.x_m, pose.y_m, goal_pose_.yaw_rad}};
        bridge_.startFollowWaypoints(walk_base_, active_segment_, follow_params_);
        refreshMonitorFromBridge();
        blocked_since_ = TimePointUs{};
        monitor_.active = true;
        monitor_.paused = false;
        monitor_.lifecycle = NavigationLifecycleState::Running;
        return;
    }

    if (plan.status != LocalPlanStatus::Ready || plan.waypoints.empty()) {
        bridge_.deactivate();
        active_segment_.clear();
        if (blocked_since_.isZero()) {
            blocked_since_ = now;
        }
        monitor_.lifecycle = NavigationLifecycleState::Blocked;
        monitor_.active = true;
        monitor_.paused = false;
        monitor_.blocked_elapsed_s = secondsBetween(now, blocked_since_);
        return;
    }

    blocked_since_ = TimePointUs{};
    startBridgeFromPlan(plan, pose);
    monitor_.active = true;
    monitor_.paused = false;
    monitor_.lifecycle = NavigationLifecycleState::Running;
}

void NavigationManager::startBridgeFromPlan(const LocalPlanResult& plan,
                                            const NavPose2d& current_pose) {
    active_segment_ = plan.waypoints;
    // A* includes the start cell as a path node. Its yaw points toward the next
    // cell, so following it as a pose goal can demand a 180-degree turn before
    // a perfectly valid backwards/sideways translation. It is already reached.
    if (active_segment_.size() > 1) {
        const NavPose2d& first = active_segment_.front();
        const double start_cell_radius_m = 0.75 * local_map_builder_.config().resolution_m;
        if (std::hypot(first.x_m - current_pose.x_m, first.y_m - current_pose.y_m) <=
            start_cell_radius_m) {
            active_segment_.erase(active_segment_.begin());
        }
    }
    if (!follow_params_.go_to.rotate_first && !active_segment_.empty()) {
        // A holonomic command does not need to face every path tangent. Keep
        // the current heading at intermediate nodes and reserve the requested
        // yaw for the actual terminal goal, not a horizon-limited segment end.
        const NavPose2d& last = active_segment_.back();
        const double goal_cell_radius_m = 0.75 * local_map_builder_.config().resolution_m;
        const bool reaches_terminal_cell =
            std::hypot(last.x_m - goal_pose_.x_m, last.y_m - goal_pose_.y_m) <= goal_cell_radius_m;
        for (std::size_t i = 0; i < active_segment_.size(); ++i) {
            if (i + 1 < active_segment_.size() || !reaches_terminal_cell) {
                active_segment_[i].yaw_rad = current_pose.yaw_rad;
            }
        }
    }
    bridge_.startFollowWaypoints(walk_base_, active_segment_, follow_params_);
    monitor_.active_segment_waypoint_count = active_segment_.size();
    monitor_.active_segment_length_m = pathLength(active_segment_);
    refreshMonitorFromBridge();
}

void NavigationManager::refreshMonitorFromBridge() {
    monitor_.bridge = bridge_.monitor();
    monitor_.active_segment_waypoint_count = active_segment_.size();
    monitor_.active_segment_length_m = pathLength(active_segment_);
    monitor_.active_segment = active_segment_;
    if (map_aware_mode_) {
        monitor_.has_goal = monitor_.active;
        monitor_.goal = goal_pose_;
    } else if (!active_segment_.empty()) {
        monitor_.has_goal = monitor_.active;
        monitor_.goal = active_segment_.back();
    } else {
        monitor_.has_goal = false;
        monitor_.goal = {};
    }
}

bool NavigationManager::shouldRefreshTerrainSnapshot(const RobotState& est, const TimePointUs now) const {
    if (est.sample_id != 0) {
        return est.sample_id != last_terrain_refresh_sample_id_;
    }
    return now.value != last_terrain_refresh_timestamp_.value;
}

LocalMapSnapshot NavigationManager::refreshTerrainSnapshotLocked(const NavPose2d& pose,
                                                                const RobotState& est,
                                                                const TimePointUs now) {
    if (shouldRefreshTerrainSnapshot(est, now)) {
        const std::vector<LocalMapObservation> observations = collectObservations(pose, est, now);
        local_map_builder_.update(pose, now, observations);
        last_terrain_refresh_sample_id_ = est.sample_id;
        last_terrain_refresh_timestamp_ = now;
    }

    const LocalMapSnapshot snapshot = local_map_builder_.snapshot(now);
    cached_map_snapshot_ = snapshot;
    monitor_.map_fresh = snapshot.has_primary_observations ? snapshot.fresh : true;
    monitor_.nearest_obstacle_distance_m = snapshot.nearest_obstacle_distance_m;
    return snapshot;
}

NavigationMonitorSnapshot NavigationManager::monitor() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return monitor_;
}

bool NavigationManager::active() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.active;
}

bool NavigationManager::paused() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return monitor_.paused;
}

LocalMapSnapshot NavigationManager::latestMapSnapshot(const TimePointUs now) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return local_map_builder_.snapshot(now);
}

namespace {

bool primaryObservationFresh(const LocalMapSnapshot& snap, const TimePointUs now, const double timeout_s) {
    if (!snap.has_primary_observations || snap.last_primary_observation_timestamp.isZero()) {
        return false;
    }
    if (now.value <= snap.last_primary_observation_timestamp.value) {
        return true;
    }
    const double age_s =
        static_cast<double>(now.value - snap.last_primary_observation_timestamp.value) * 1e-6;
    return age_s <= timeout_s;
}

} // namespace

const LocalMapSnapshot* NavigationManager::footTerrainSnapshot(const TimePointUs now) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    const double timeout_s = local_map_builder_.config().observation_timeout_s;
    if (!primaryObservationFresh(cached_map_snapshot_, now, timeout_s)) {
        return nullptr;
    }
    if (!cached_map_snapshot_.elevation_has_data && !cached_map_snapshot_.ground_elevation_has_data) {
        return nullptr;
    }
    return &cached_map_snapshot_;
}
