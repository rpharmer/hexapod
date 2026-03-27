#include "autonomy/modules/local_planner.hpp"

namespace autonomy {

LocalPlannerModuleShell::LocalPlannerModuleShell()
    : AutonomyModuleStub("local_planner") {}

LocalPlan LocalPlannerModuleShell::plan(const GlobalPlan& global_plan,
                                        bool blocked) {
    if (global_plan.has_plan && !blocked) {
        plan_.has_command = true;
        plan_.target = global_plan.target;
        return plan_;
    }

    plan_ = {};
    return plan_;
}

LocalPlan LocalPlannerModuleShell::currentPlan() const {
    return plan_;
}

} // namespace autonomy
