#include "autonomy/modules/global_planner.hpp"

#include "global_planner_algorithms.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace autonomy {
namespace {

constexpr double kDegradedCostThreshold = 0.65;
constexpr double kUnsafeCostThreshold = 0.95;
constexpr double kUnsafeRiskThreshold = 0.85;
constexpr double kReplanCostDeltaThreshold = 0.2;
constexpr double kReplanRiskDeltaThreshold = 0.2;

bool shouldTriggerReplan(bool had_previous_report,
                         bool previous_traversable,
                         double previous_cost,
                         double previous_risk,
                         const TraversabilityReport& report) {
    if (!had_previous_report) {
        return true;
    }
    if (previous_traversable != report.traversable) {
        return true;
    }
    return std::abs(previous_cost - report.cost) >= kReplanCostDeltaThreshold ||
           std::abs(previous_risk - report.risk) >= kReplanRiskDeltaThreshold;
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

    const bool unsafe_by_constraints = !traversability_report.traversable ||
                                       traversability_report.cost >= kUnsafeCostThreshold ||
                                       traversability_report.risk >= kUnsafeRiskThreshold;
    if (unsafe_by_constraints) {
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

    const bool replan_triggered = shouldTriggerReplan(had_traversability_report_,
                                                      previous_traversable_,
                                                      previous_cost_,
                                                      previous_risk_,
                                                      traversability_report);
    const auto& target = navigation_update.intent.target;
    if (replan_triggered || !plan_.has_plan) {
        plan_.route = global_planner::algorithms::buildConstraintAwareRoute(target, traversability_report.cost, traversability_report.risk);
    }
    auto route = plan_.route;
    if (route.empty()) {
        plan_ = GlobalPlan{
            .has_plan = false,
            .target = target,
            .route = {},
            .cost = traversability_report.cost,
            .status = PlannerStatus::UnsafePlan,
            .reason = "unsafe-traversability",
            .source_timestamp_ms = traversability_report.timestamp_ms,
        };
        had_traversability_report_ = true;
        previous_traversable_ = traversability_report.traversable;
        previous_cost_ = traversability_report.cost;
        previous_risk_ = traversability_report.risk;
        return plan_;
    }

    const double path_length = std::hypot(target.x_m, target.y_m);
    const double replan_penalty = replan_triggered ? 0.1 : 0.0;
    const double weighted_cost = path_length * (1.0 + traversability_report.cost) +
                                 static_cast<double>(route.size()) * 0.05 + replan_penalty;

    plan_ = GlobalPlan{
        .has_plan = true,
        .target = target,
        .route = std::move(route),
        .cost = weighted_cost,
        .status = traversability_report.cost >= kDegradedCostThreshold ? PlannerStatus::Degraded : PlannerStatus::Ready,
        .reason = traversability_report.cost >= kDegradedCostThreshold ? "high-cost-route" : "route-ready",
        .source_timestamp_ms = traversability_report.timestamp_ms,
    };
    had_traversability_report_ = true;
    previous_traversable_ = traversability_report.traversable;
    previous_cost_ = traversability_report.cost;
    previous_risk_ = traversability_report.risk;
    return plan_;
}

GlobalPlan GlobalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
