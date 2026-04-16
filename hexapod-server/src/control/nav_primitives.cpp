#include "nav_primitives.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

double clampAbs(double v, double lim) {
    return std::clamp(v, -lim, lim);
}

} // namespace

double navWrapAngleRad(const double yaw_rad) {
    double a = yaw_rad;
    while (a > kPi) {
        a -= 2.0 * kPi;
    }
    while (a < -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

RotateToHeading::RotateToHeading()
    : RotateToHeading(Params{}) {}

RotateToHeading::RotateToHeading(Params params)
    : params_(params) {}

void RotateToHeading::reset(const double target_yaw_rad) {
    target_yaw_rad_ = navWrapAngleRad(target_yaw_rad);
    settle_cycles_ = 0;
}

NavTaskUpdate RotateToHeading::update(const double current_yaw_rad, const double /*dt_s*/) {
    const double err = navWrapAngleRad(target_yaw_rad_ - navWrapAngleRad(current_yaw_rad));
    NavTaskUpdate out{};

    if (std::abs(err) < params_.error_threshold_rad) {
        ++settle_cycles_;
    } else {
        settle_cycles_ = 0;
    }

    if (settle_cycles_ >= params_.settle_cycles_required) {
        out.cmd = NavCommand{};
        out.cmd.stop_at_goal = true;
        out.status = NavTaskStatus::Completed;
        return out;
    }

    out.cmd.yaw_rate_radps = clampAbs(params_.kp * err, params_.yaw_rate_limit_radps);
    out.status = NavTaskStatus::Running;
    return out;
}

DriveDistance::DriveDistance()
    : DriveDistance(Params{}) {}

DriveDistance::DriveDistance(Params params)
    : params_(params) {}

void DriveDistance::reset(const NavPose2d start, const double delta_forward_m, const double delta_lateral_m) {
    start_yaw_rad_ = start.yaw_rad;
    settle_cycles_ = 0;
    const double c = std::cos(start.yaw_rad);
    const double s = std::sin(start.yaw_rad);
    goal_x_m_ = start.x_m + c * delta_forward_m - s * delta_lateral_m;
    goal_y_m_ = start.y_m + s * delta_forward_m + c * delta_lateral_m;
}

NavTaskUpdate DriveDistance::update(const NavPose2d current, const double /*dt_s*/) {
    NavTaskUpdate out{};
    const double ex = goal_x_m_ - current.x_m;
    const double ey = goal_y_m_ - current.y_m;
    const double c = std::cos(current.yaw_rad);
    const double s = std::sin(current.yaw_rad);
    const double e_fwd = c * ex + s * ey;
    const double e_lat = -s * ex + c * ey;

    if (std::hypot(e_fwd, e_lat) < params_.position_tol_m) {
        ++settle_cycles_;
    } else {
        settle_cycles_ = 0;
    }

    if (settle_cycles_ >= params_.settle_cycles_required) {
        out.cmd = NavCommand{};
        out.cmd.stop_at_goal = true;
        out.status = NavTaskStatus::Completed;
        return out;
    }

    out.cmd.vx_mps = clampAbs(params_.position_gain * e_fwd, params_.max_v_mps);
    out.cmd.vy_mps = clampAbs(params_.position_gain * e_lat, params_.max_v_mps);
    out.cmd.yaw_rate_radps =
        clampAbs(params_.yaw_hold_kp * navWrapAngleRad(start_yaw_rad_ - current.yaw_rad), 0.38);
    out.status = NavTaskStatus::Running;
    return out;
}

GoToPose::GoToPose()
    : GoToPose(Params{}) {}

GoToPose::GoToPose(Params params)
    : params_(params),
      rotate_(params.rotate),
      drive_(params.drive) {}

void GoToPose::reset(const NavPose2d start, const double goal_x_m, const double goal_y_m, const double goal_yaw_rad) {
    goal_x_m_ = goal_x_m;
    goal_y_m_ = goal_y_m;
    goal_yaw_rad_ = navWrapAngleRad(goal_yaw_rad);

    if (params_.rotate_first) {
        const double pre_turn_target_yaw_rad =
            std::atan2(goal_y_m_ - start.y_m, goal_x_m_ - start.x_m);
        rotate_.reset(pre_turn_target_yaw_rad);
        phase_ = Phase::PreTurn;
    } else {
        const double dx = goal_x_m_ - start.x_m;
        const double dy = goal_y_m_ - start.y_m;
        const double c = std::cos(start.yaw_rad);
        const double s = std::sin(start.yaw_rad);
        const double df = c * dx + s * dy;
        const double dl = -s * dx + c * dy;
        drive_.reset(start, df, dl);
        phase_ = Phase::Translate;
    }
}

void GoToPose::begin_translate(const NavPose2d current) {
    const double dx = goal_x_m_ - current.x_m;
    const double dy = goal_y_m_ - current.y_m;
    const double c = std::cos(current.yaw_rad);
    const double s = std::sin(current.yaw_rad);
    const double df = c * dx + s * dy;
    const double dl = -s * dx + c * dy;
    drive_.reset(current, df, dl);
}

NavTaskUpdate GoToPose::update(const NavPose2d current, const double dt_s) {
    switch (phase_) {
    case Phase::PreTurn: {
        NavTaskUpdate u = rotate_.update(current.yaw_rad, dt_s);
        if (u.status == NavTaskStatus::Completed) {
            begin_translate(current);
            phase_ = Phase::Translate;
            return drive_.update(current, dt_s);
        }
        return u;
    }
    case Phase::Translate: {
        NavTaskUpdate u = drive_.update(current, dt_s);
        if (u.status == NavTaskStatus::Completed) {
            rotate_.reset(goal_yaw_rad_);
            phase_ = Phase::PostTurn;
            return rotate_.update(current.yaw_rad, dt_s);
        }
        return u;
    }
    case Phase::PostTurn: {
        NavTaskUpdate u = rotate_.update(current.yaw_rad, dt_s);
        if (u.status == NavTaskStatus::Completed) {
            phase_ = Phase::Done;
        }
        return u;
    }
    case Phase::Done: {
        NavTaskUpdate u{};
        u.cmd.stop_at_goal = true;
        u.status = NavTaskStatus::Completed;
        return u;
    }
    }
    return NavTaskUpdate{};
}

FollowWaypoints::FollowWaypoints()
    : FollowWaypoints(Params{}) {}

FollowWaypoints::FollowWaypoints(Params params)
    : params_(params),
      inner_(params.go_to) {}

void FollowWaypoints::reset(std::vector<NavPose2d> waypoints) {
    waypoints_ = std::move(waypoints);
    index_ = 0;
    stall_timer_s_ = 0.0;
    best_distance_m_ = 1e9;
    need_inner_reset_ = true;
}

NavTaskUpdate FollowWaypoints::update(const NavPose2d current, const double dt_s) {
    if (waypoints_.empty()) {
        NavTaskUpdate u{};
        u.status = NavTaskStatus::Completed;
        u.cmd.stop_at_goal = true;
        return u;
    }

    if (need_inner_reset_) {
        if (index_ >= waypoints_.size()) {
            NavTaskUpdate u{};
            u.status = NavTaskStatus::Completed;
            u.cmd.stop_at_goal = true;
            return u;
        }
        const NavPose2d& wp = waypoints_[index_];
        inner_.reset(current, wp.x_m, wp.y_m, wp.yaw_rad);
        need_inner_reset_ = false;
        const double dx = wp.x_m - current.x_m;
        const double dy = wp.y_m - current.y_m;
        best_distance_m_ = std::hypot(dx, dy);
        stall_timer_s_ = 0.0;
    }

    const NavPose2d& wp = waypoints_[index_];
    const double dist = std::hypot(wp.x_m - current.x_m, wp.y_m - current.y_m);
    // Real gaits often do not shrink Euclidean range-to-goal every tick (slip, heading wobble).
    // Near the running best, accrue stall slowly; only accrue at full rate when clearly regressing.
    constexpr double kStallDistHysteresisM = 0.008;
    if (dist < best_distance_m_ - 1e-9) {
        best_distance_m_ = dist;
        stall_timer_s_ = 0.0;
    } else if (dist <= best_distance_m_ + kStallDistHysteresisM) {
        stall_timer_s_ += dt_s * 0.08;
    } else {
        stall_timer_s_ += dt_s;
    }
    if (stall_timer_s_ > params_.stall_timeout_s) {
        NavTaskUpdate u{};
        u.status = NavTaskStatus::Failed;
        return u;
    }

    NavTaskUpdate u = inner_.update(current, dt_s);
    if (u.status == NavTaskStatus::Completed) {
        ++index_;
        need_inner_reset_ = true;
        if (index_ >= waypoints_.size()) {
            u.cmd = NavCommand{};
            u.cmd.stop_at_goal = true;
            return u;
        }
        return update(current, dt_s);
    }
    return u;
}
