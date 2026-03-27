#include "autonomy/modules/local_planner.hpp"

namespace autonomy {
namespace {

constexpr uint64_t kPlanStaleAfterMs = 1000;

} // namespace

LocalPlannerModuleShell::LocalPlannerModuleShell()
    : AutonomyModuleStub("local_planner") {}

LocalPlan LocalPlannerModuleShell::plan(const GlobalPlan& global_plan,
                                        bool blocked,
                                        uint64_t now_ms) {
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

    if (now_ms > global_plan.source_timestamp_ms + kPlanStaleAfterMs) {
        const bool have_fallback_target = plan_.has_command;
        const Waypoint fallback_target = plan_.target;
        plan_ = LocalPlan{
            .has_command = have_fallback_target,
            .target = have_fallback_target ? fallback_target : global_plan.target,
            .status = PlannerStatus::StalePlan,
            .reason = have_fallback_target ? "stale-global-plan-fallback-last-command" : "stale-global-plan-no-fallback",
            .fallback_active = true,
        };
        return plan_;
    }

    const auto& route = global_plan.route;
    const Waypoint short_horizon_target = route.empty() ? global_plan.target : route.front();
    plan_ = LocalPlan{
        .has_command = true,
        .target = short_horizon_target,
        .status = global_plan.status == PlannerStatus::Degraded ? PlannerStatus::Degraded : PlannerStatus::Ready,
        .reason = global_plan.status == PlannerStatus::Degraded ? "degraded-short-horizon" : "short-horizon-ready",
        .fallback_active = false,
    };
    return plan_;
}

LocalPlan LocalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
