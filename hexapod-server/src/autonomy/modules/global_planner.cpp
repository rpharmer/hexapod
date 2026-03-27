#include "autonomy/modules/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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

std::vector<Waypoint> buildConstraintAwareRoute(const Waypoint& target,
                                                double traversability_cost,
                                                double traversability_risk) {
    const int step_count = std::clamp(3 + static_cast<int>(std::lround(traversability_cost * 8.0)), 3, 10);
    const double lateral_bias = std::clamp(traversability_cost * 0.75 + traversability_risk * 0.5, 0.0, 1.0);
    std::vector<Waypoint> route;
    route.reserve(static_cast<size_t>(step_count));

    double previous_x = 0.0;
    double previous_y = 0.0;
    for (int i = 1; i <= step_count; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(step_count);
        const double offset_gain = lateral_bias * std::sin(alpha * 3.14159265358979323846);
        const double normal_x = target.y_m;
        const double normal_y = -target.x_m;
        const double normal_norm = std::hypot(normal_x, normal_y);
        const double unit_normal_x = normal_norm > std::numeric_limits<double>::epsilon() ? normal_x / normal_norm : 0.0;
        const double unit_normal_y = normal_norm > std::numeric_limits<double>::epsilon() ? normal_y / normal_norm : 0.0;
        const double x = target.x_m * alpha + unit_normal_x * offset_gain;
        const double y = target.y_m * alpha + unit_normal_y * offset_gain;
        const double segment_length = std::hypot(x - previous_x, y - previous_y);
        previous_x = x;
        previous_y = y;

        const double cell_risk_proxy = std::clamp(traversability_cost * (0.35 + alpha * 0.65) +
                                                      traversability_risk * 0.45 +
                                                      segment_length * 0.2,
                                                  0.0,
                                                  1.2);
        if (cell_risk_proxy >= 1.0) {
            continue;
        }
        route.push_back(Waypoint{
            .frame_id = target.frame_id,
            .x_m = x,
            .y_m = y,
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
            .reason = "unsafe-traversability",
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
        plan_.route = buildConstraintAwareRoute(target, traversability_report.cost, traversability_report.risk);
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
