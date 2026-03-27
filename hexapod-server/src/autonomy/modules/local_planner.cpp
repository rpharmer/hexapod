#include "autonomy/modules/local_planner.hpp"

#include <cmath>

namespace autonomy {
Waypoint LocalPlannerModuleShell::selectShortHorizonTarget(const GlobalPlan& global_plan) const {
    if (global_plan.route.empty()) {
        return global_plan.target;
    }

    Waypoint selected = global_plan.route.front();
    double cumulative_distance = 0.0;
    double previous_x = 0.0;
    double previous_y = 0.0;
    for (const auto& waypoint : global_plan.route) {
        cumulative_distance += std::hypot(waypoint.x_m - previous_x, waypoint.y_m - previous_y);
        previous_x = waypoint.x_m;
        previous_y = waypoint.y_m;
        selected = waypoint;
        if (cumulative_distance >= policy_.execution_horizon_m) {
            break;
        }
    }
    return selected;
}

bool LocalPlannerModuleShell::trajectoryFeasible(const GlobalPlan& global_plan) const {
    if (global_plan.route.empty()) {
        return false;
    }
    double previous_x = 0.0;
    double previous_y = 0.0;
    for (const auto& waypoint : global_plan.route) {
        const double step = std::hypot(waypoint.x_m - previous_x, waypoint.y_m - previous_y);
        if (step > policy_.max_feasible_step_m) {
            return false;
        }
        previous_x = waypoint.x_m;
        previous_y = waypoint.y_m;
    }
    return true;
}

Waypoint LocalPlannerModuleShell::blendFallbackTarget(const Waypoint& stale_target,
                                                      const Waypoint& fallback_target) const {
    return Waypoint{
        .frame_id = fallback_target.frame_id.empty() ? stale_target.frame_id : fallback_target.frame_id,
        .x_m = fallback_target.x_m + (stale_target.x_m - fallback_target.x_m) * policy_.fallback_blend_gain,
        .y_m = fallback_target.y_m + (stale_target.y_m - fallback_target.y_m) * policy_.fallback_blend_gain,
        .yaw_rad = fallback_target.yaw_rad + (stale_target.yaw_rad - fallback_target.yaw_rad) * policy_.fallback_blend_gain,
    };
}

LocalPlannerModuleShell::LocalPlannerModuleShell(LocalPlannerPolicyConfig policy)
    : AutonomyModuleStub("local_planner"),
      policy_(policy) {}

LocalPlan LocalPlannerModuleShell::plan(const GlobalPlan& global_plan,
                                        bool blocked,
                                        uint64_t now_ms,
                                        ContractEnvelope envelope) {
    (void)envelope;
    if (!global_plan.has_plan) {
        plan_ = LocalPlan{
            .has_command = false,
            .target = {},
            .status = global_plan.status == PlannerStatus::UnsafePlan ? PlannerStatus::UnsafePlan : PlannerStatus::NoPlan,
            .reason = global_plan.reason.empty() ? "no-global-plan" : global_plan.reason,
            .fallback_active = false,
        };
        return plan_;
    }

    if (blocked) {
        plan_ = LocalPlan{
            .has_command = false,
            .target = {},
            .status = PlannerStatus::UnsafePlan,
            .reason = "path-blocked",
            .fallback_active = false,
        };
        return plan_;
    }

    if (now_ms > global_plan.source_timestamp_ms + policy_.stale_plan_timeout_ms) {
        const bool have_fallback_target = last_executable_target_valid_;
        const auto fallback_target = have_fallback_target
                                         ? blendFallbackTarget(global_plan.target, last_executable_target_)
                                         : global_plan.target;
        plan_ = LocalPlan{
            .has_command = have_fallback_target,
            .target = fallback_target,
            .status = PlannerStatus::StalePlan,
            .reason = have_fallback_target ? "stale-global-plan-fallback-smoothed" : "stale-global-plan-no-fallback",
            .fallback_active = true,
        };
        return plan_;
    }

    if (!trajectoryFeasible(global_plan)) {
        plan_ = LocalPlan{
            .has_command = false,
            .target = {},
            .status = PlannerStatus::UnsafePlan,
            .reason = "infeasible-short-horizon-trajectory",
            .fallback_active = false,
        };
        return plan_;
    }

    const Waypoint short_horizon_target = selectShortHorizonTarget(global_plan);
    plan_ = LocalPlan{
        .has_command = true,
        .target = short_horizon_target,
        .status = global_plan.status == PlannerStatus::Degraded ? PlannerStatus::Degraded : PlannerStatus::Ready,
        .reason = global_plan.status == PlannerStatus::Degraded ? "degraded-short-horizon" : "short-horizon-ready",
        .fallback_active = false,
    };
    last_executable_target_ = short_horizon_target;
    last_executable_target_valid_ = true;
    return plan_;
}

LocalPlan LocalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
