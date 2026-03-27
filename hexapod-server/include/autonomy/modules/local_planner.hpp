#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

class LocalPlannerModuleShell : public AutonomyModuleStub {
public:
    LocalPlannerModuleShell();

    LocalPlan plan(const GlobalPlan& global_plan);
    [[nodiscard]] LocalPlan currentPlan() const;

private:
    LocalPlan plan_{};
};

} // namespace autonomy
