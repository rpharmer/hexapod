#pragma once

#include "local_map.hpp"
#include "local_planner.hpp"
#include "control_config.hpp"
#include "nav_locomotion_bridge.hpp"

#include <memory>
#include <mutex>
#include <vector>

enum class NavigationLifecycleState {
    Idle,
    Running,
    Paused,
    Blocked,
    MapUnavailable,
    Completed,
    Failed,
    Cancelled,
};

struct NavigationMonitorSnapshot {
    NavigationLifecycleState lifecycle{NavigationLifecycleState::Idle};
    LocalPlanStatus planner_status{LocalPlanStatus::Blocked};
    PlannerBlockReason block_reason{PlannerBlockReason::None};
    bool active{false};
    bool paused{false};
    bool map_fresh{false};
    std::size_t replan_count{0};
    std::size_t active_segment_waypoint_count{0};
    double active_segment_length_m{0.0};
    double nearest_obstacle_distance_m{-1.0};
    double blocked_elapsed_s{0.0};
    TimePointUs last_plan_timestamp{};
    NavLocomotionBridge::MonitorSnapshot bridge{};
};

class NavigationManager {
public:
    NavigationManager(LocalMapConfig local_map_config = {},
                      LocalPlannerConfig local_planner_config = {},
                      control_config::NavBridgeConfig nav_bridge_config = {},
                      std::unique_ptr<ILocalPlanner> planner = std::make_unique<AStarLocalPlanner>());

    void addObservationSource(std::shared_ptr<ILocalMapObservationSource> source);
    void clearObservationSources();

    void startNavigateToPose(MotionIntent walk_base,
                             NavPose2d goal_pose,
                             FollowWaypoints::Params follow_params = {});
    void startRawFollowWaypoints(MotionIntent walk_base,
                                 std::vector<NavPose2d> waypoints,
                                 FollowWaypoints::Params follow_params = {});

    void pause();
    void resume();
    void cancel();
    void deactivate();

    [[nodiscard]] bool active() const;
    [[nodiscard]] bool paused() const;
    [[nodiscard]] NavigationMonitorSnapshot monitor() const;
    [[nodiscard]] LocalMapSnapshot latestMapSnapshot(TimePointUs now) const;

    MotionIntent mergeIntent(const MotionIntent& fallback, const RobotState& est, TimePointUs now);

private:
    [[nodiscard]] MotionIntent makeStopIntent() const;
    [[nodiscard]] std::vector<LocalMapObservation> collectObservations(const NavPose2d& pose, TimePointUs now) const;
    [[nodiscard]] bool activeSegmentBlocked(const LocalMapSnapshot& snapshot) const;
    [[nodiscard]] bool terminalGoalReached(const NavPose2d& pose) const;
    void planOrBlock(const NavPose2d& pose, const LocalMapSnapshot& snapshot, TimePointUs now);
    void startBridgeFromPlan(const LocalPlanResult& plan);
    void refreshMonitorFromBridge();

    LocalMapBuilder local_map_builder_{};
    LocalPlannerConfig planner_config_{};
    control_config::NavBridgeConfig nav_bridge_config_{};
    std::unique_ptr<ILocalPlanner> planner_{};
    std::vector<std::shared_ptr<ILocalMapObservationSource>> observation_sources_{};

    MotionIntent walk_base_{};
    FollowWaypoints::Params follow_params_{};
    NavPose2d goal_pose_{};
    std::vector<NavPose2d> raw_waypoints_{};
    std::vector<NavPose2d> active_segment_{};
    NavLocomotionBridge bridge_{};
    NavigationMonitorSnapshot monitor_{};
    mutable std::mutex mutex_{};

    bool map_aware_mode_{false};
    TimePointUs last_merge_timestamp_{};
    TimePointUs blocked_since_{};
};
