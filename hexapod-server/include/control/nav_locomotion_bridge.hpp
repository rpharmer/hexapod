#pragma once

#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"
#include "types.hpp"

#include <vector>

/**
 * Connects navigation output (`NavCommand`) to the existing locomotion stack via `MotionIntent`.
 * Owns no gait/leg/IK state: callers supply a WALK `MotionIntent` template (gait, height, mode);
 * this type only overwrites planar velocity fields each tick from `FollowWaypoints`.
 */
class NavLocomotionBridge {
public:
    /** Begin following waypoints; `walk_base` must request WALK and desired gait / body height. */
    void startFollowWaypoints(MotionIntent walk_base, std::vector<NavPose2d> waypoints);
    void startFollowWaypoints(MotionIntent walk_base,
                              std::vector<NavPose2d> waypoints,
                              const FollowWaypoints::Params& follow_params);

    void deactivate();
    bool active() const { return active_; }

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
    bool active_{false};
    MotionIntent walk_base_{};
    FollowWaypoints follow_{};

    double cmd_slew_alpha_{0.0};
    bool cmd_slew_have_prev_{false};
    double prev_nav_vx_{0.0};
    double prev_nav_vy_{0.0};
    double prev_nav_w_{0.0};
};
