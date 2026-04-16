#pragma once

#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"
#include "types.hpp"

#include <cstddef>
#include <vector>

/**
 * Connects navigation output (`NavCommand`) to the existing locomotion stack via `MotionIntent`.
 * Owns no gait/leg/IK state: callers supply a WALK `MotionIntent` template (gait, height, mode);
 * this type only overwrites planar velocity fields each tick from `FollowWaypoints`.
 */
class NavLocomotionBridge {
public:
    enum class FailPolicy {
        FailHold,
        FailStop,
        FailRetryN,
    };

    enum class LifecycleState {
        Idle,
        Running,
        Paused,
        Completed,
        Failed,
        Cancelled,
    };

    struct MonitorSnapshot {
        LifecycleState lifecycle{LifecycleState::Idle};
        NavTaskFailureReason failure_reason{NavTaskFailureReason::None};
        bool active{false};
        bool paused{false};
        std::size_t active_waypoint_index{0};
        std::size_t waypoint_count{0};
        bool has_active_waypoint{false};
        double distance_to_active_waypoint_m{0.0};
        double best_distance_to_active_waypoint_m{0.0};
        double stall_timer_s{0.0};
        double elapsed_s{0.0};
        int retry_count{0};
        int retry_budget{0};
    };

    /** Begin following waypoints; `walk_base` must request WALK and desired gait / body height. */
    void startFollowWaypoints(MotionIntent walk_base, std::vector<NavPose2d> waypoints);
    void startFollowWaypoints(MotionIntent walk_base,
                              std::vector<NavPose2d> waypoints,
                              const FollowWaypoints::Params& follow_params);

    void deactivate();
    void pause();
    void resume();
    void cancel();
    bool active() const { return active_; }
    bool paused() const { return paused_; }

    void setFailPolicy(FailPolicy policy, int retry_budget = 0);
    [[nodiscard]] MonitorSnapshot monitor() const { return monitor_; }

    /**
     * Blend each new `NavCommand` toward the previous one: `cmd = prev * (1-a) + raw * a`.
     * Use a small positive `alpha` (e.g. 0.15–0.35) to limit jerk when closing loops on a stiff sim.
     * `alpha == 0` disables (default).
     */
    void setPlanarCommandSlew01(double alpha);

    /**
     * When `active()`, merges planar commands from navigation into `walk_base_` and returns the
     * result. When inactive, returns `fallback` unchanged. Uses `est` for pose via
     * `navPose2dFromRobotState`.
     */
    MotionIntent mergeIntent(const MotionIntent& fallback, const RobotState& est, double dt_s);

private:
    MotionIntent makeStopIntentFromWalkBase() const;
    MotionIntent onFailedTask(const MotionIntent& fallback);
    void updateMonitorFromFollow();

    bool active_{false};
    bool paused_{false};
    MotionIntent walk_base_{};
    std::vector<NavPose2d> waypoints_{};
    FollowWaypoints::Params follow_params_{};
    FollowWaypoints follow_{};
    MonitorSnapshot monitor_{};
    FailPolicy fail_policy_{FailPolicy::FailHold};
    int retry_budget_{0};

    double cmd_slew_alpha_{0.0};
    bool cmd_slew_have_prev_{false};
    double prev_nav_vx_{0.0};
    double prev_nav_vy_{0.0};
    double prev_nav_w_{0.0};
};
