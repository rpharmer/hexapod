#pragma once

#include "nav_command.hpp"

#include <cmath>
#include <vector>

/** Wrap angle to (-pi, pi]. */
double navWrapAngleRad(double yaw_rad);

enum class NavTaskStatus {
    Running,
    Completed,
    Failed,
};

enum class NavTaskFailureReason {
    None,
    StallTimeout,
};

struct NavTaskUpdate {
    NavCommand cmd{};
    NavTaskStatus status{NavTaskStatus::Running};
    NavTaskFailureReason failure_reason{NavTaskFailureReason::None};
};

struct FollowWaypointsProgress {
    std::size_t waypoint_index{0};
    std::size_t waypoint_count{0};
    bool has_active_waypoint{false};
    double distance_to_active_waypoint_m{0.0};
    double best_distance_to_active_waypoint_m{0.0};
    double stall_timer_s{0.0};
};

/** In-place yaw regulation; success when |error| < threshold for N consecutive updates. */
class RotateToHeading {
public:
    struct Params {
        Params() = default;
        double yaw_rate_limit_radps{0.55};
        double kp{1.5};
        double error_threshold_rad{0.06};
        int settle_cycles_required{6};
    };

    RotateToHeading();
    explicit RotateToHeading(Params params);

    void reset(double target_yaw_rad);
    NavTaskUpdate update(double current_yaw_rad, double dt_s);

private:
    Params params_{};
    double target_yaw_rad_{};
    int settle_cycles_{0};
};

/**
 * Move in the horizontal plane to a goal computed once from the start pose and a body-frame
 * displacement (+x forward, +y left at start).
 */
class DriveDistance {
public:
    struct Params {
        Params() = default;
        double max_v_mps{0.22};
        double position_gain{0.9};
        /** Optional yaw hold relative to start heading during the move. */
        double yaw_hold_kp{0.6};
        double position_tol_m{0.025};
        int settle_cycles_required{5};
    };

    DriveDistance();
    explicit DriveDistance(Params params);

    void reset(NavPose2d start, double delta_forward_m, double delta_lateral_m);
    NavTaskUpdate update(NavPose2d current, double dt_s);

private:
    Params params_{};
    double goal_x_m_{};
    double goal_y_m_{};
    double start_yaw_rad_{};
    int settle_cycles_{0};
};

/**
 * Go to (x, y, yaw): optional initial turn toward the goal point, translate, then final yaw.
 * Caller supplies `NavPose2d` from odometry / SLAM / sim each tick.
 */
class GoToPose {
public:
    struct Params {
        Params() = default;
        RotateToHeading::Params rotate{};
        DriveDistance::Params drive{};
        bool rotate_first{true};
    };

    GoToPose();
    explicit GoToPose(Params params);

    void reset(NavPose2d start, double goal_x_m, double goal_y_m, double goal_yaw_rad);
    NavTaskUpdate update(NavPose2d current, double dt_s);

private:
    enum class Phase {
        PreTurn,
        Translate,
        PostTurn,
        Done,
    };

    void begin_translate(NavPose2d current);

    Params params_{};
    Phase phase_{Phase::Done};
    double goal_x_m_{};
    double goal_y_m_{};
    double goal_yaw_rad_{};
    RotateToHeading rotate_{};
    DriveDistance drive_{};
};

/** Visit poses in order; optional stall detection on progress toward the active goal. */
class FollowWaypoints {
public:
    struct Params {
        Params() = default;
        GoToPose::Params go_to{};
        double stall_timeout_s{4.0};
    };

    FollowWaypoints();
    explicit FollowWaypoints(Params params);

    void reset(std::vector<NavPose2d> waypoints);
    NavTaskUpdate update(NavPose2d current, double dt_s);
    FollowWaypointsProgress progress() const;

private:
    Params params_{};
    std::vector<NavPose2d> waypoints_{};
    std::size_t index_{0};
    GoToPose inner_{};
    double stall_timer_s_{0.0};
    double best_distance_m_{1e9};
    double current_distance_m_{1e9};
    bool need_inner_reset_{true};
};
