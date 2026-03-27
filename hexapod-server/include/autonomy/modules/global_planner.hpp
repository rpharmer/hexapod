#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_data.hpp"
#include "autonomy/navigation_types.hpp"

namespace autonomy {

class GlobalPlannerModuleShell : public AutonomyModuleStub {
public:
    GlobalPlannerModuleShell();

    GlobalPlan plan(const NavigationUpdate& navigation_update,
                    const TraversabilityReport& traversability_report);
    [[nodiscard]] GlobalPlan currentPlan() const;

private:
    GlobalPlan plan_{};
};

} // namespace autonomy
