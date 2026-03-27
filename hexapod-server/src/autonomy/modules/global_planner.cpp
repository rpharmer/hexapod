#include "autonomy/modules/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autonomy {
namespace {

constexpr double kDegradedCostThreshold = 0.65;

std::vector<Waypoint> buildGridRoute(const Waypoint& target, double traversability_cost) {
    const int step_count = std::clamp(2 + static_cast<int>(std::lround(traversability_cost * 6.0)), 2, 8);
    std::vector<Waypoint> route;
    route.reserve(static_cast<size_t>(step_count));

    for (int i = 1; i <= step_count; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(step_count);
        route.push_back(Waypoint{
            .frame_id = target.frame_id,
            .x_m = target.x_m * alpha,
            .y_m = target.y_m * alpha,
            .yaw_rad = target.yaw_rad,
        });
    }
    return route;
}

} // namespace

GlobalPlannerModuleShell::GlobalPlannerModuleShell()
    : AutonomyModuleStub("global_planner") {}

GlobalPlan GlobalPlannerModuleShell::plan(const NavigationUpdate& navigation_update,
                                          const TraversabilityReport& traversability_report,
                                          ContractEnvelope envelope) {
    (void)envelope;
    if (!navigation_update.has_intent) {
        plan_ = GlobalPlan{
            .has_plan = false,
            .target = {},
            .route = {},
            .cost = 0.0,
            .status = PlannerStatus::NoPlan,
            .reason = "no-navigation-intent",
            .source_timestamp_ms = traversability_report.timestamp_ms,
        };
        return plan_;
    }

    if (!traversability_report.traversable) {
        plan_ = GlobalPlan{
            .has_plan = false,
            .target = navigation_update.intent.target,
            .route = {},
            .cost = traversability_report.cost,
            .status = PlannerStatus::UnsafePlan,
            .reason = "unsafe-traversability:" + traversability_report.reason,
            .source_timestamp_ms = traversability_report.timestamp_ms,
        };
        return plan_;
    }

    const auto& target = navigation_update.intent.target;
    auto route = buildGridRoute(target, traversability_report.cost);
    const double path_length = std::hypot(target.x_m, target.y_m);
    const double weighted_cost = path_length * (1.0 + traversability_report.cost) +
                                 static_cast<double>(route.size()) * 0.05;

    plan_ = GlobalPlan{
        .has_plan = true,
        .target = target,
        .route = std::move(route),
        .cost = weighted_cost,
        .status = traversability_report.cost >= kDegradedCostThreshold ? PlannerStatus::Degraded : PlannerStatus::Ready,
        .reason = traversability_report.cost >= kDegradedCostThreshold ? "high-cost-route" : "route-ready",
        .source_timestamp_ms = traversability_report.timestamp_ms,
    };
    return plan_;
}

GlobalPlan GlobalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
