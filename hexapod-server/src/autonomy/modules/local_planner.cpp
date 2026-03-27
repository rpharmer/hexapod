#include "autonomy/modules/local_planner.hpp"

#include <cmath>

namespace autonomy {
namespace {

struct StalePlanPolicy {
    uint64_t timeout_ms{1000};
};

constexpr StalePlanPolicy kDefaultStalePlanPolicy{};
constexpr double kExecutionHorizonMeters = 1.0;

Waypoint selectShortHorizonTarget(const GlobalPlan& global_plan) {
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
        if (cumulative_distance >= kExecutionHorizonMeters) {
            break;
        }
    }
    return selected;
}

} // namespace

LocalPlannerModuleShell::LocalPlannerModuleShell()
    : AutonomyModuleStub("local_planner") {}

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

    if (now_ms > global_plan.source_timestamp_ms + kDefaultStalePlanPolicy.timeout_ms) {
        const bool have_fallback_target = last_executable_target_valid_;
        plan_ = LocalPlan{
            .has_command = have_fallback_target,
            .target = have_fallback_target ? last_executable_target_ : global_plan.target,
            .status = PlannerStatus::StalePlan,
            .reason = have_fallback_target ? "stale-global-plan-fallback-last-command" : "stale-global-plan-no-fallback",
            .fallback_active = true,
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
