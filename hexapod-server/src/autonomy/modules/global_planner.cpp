#include "autonomy/modules/global_planner.hpp"

namespace autonomy {

GlobalPlannerModuleShell::GlobalPlannerModuleShell()
    : AutonomyModuleStub("global_planner") {}

GlobalPlan GlobalPlannerModuleShell::plan(const NavigationUpdate& navigation_update,
                                          const TraversabilityReport& traversability_report) {
    if (navigation_update.has_intent && traversability_report.traversable) {
        plan_.has_plan = true;
        plan_.target = navigation_update.intent.target;
        plan_.cost = traversability_report.cost;
        return plan_;
    }

    plan_ = {};
    return plan_;
}

GlobalPlan GlobalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
