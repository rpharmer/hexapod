#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"
#include "autonomy/navigation_types.hpp"

namespace autonomy {

class GlobalPlannerModuleShell : public AutonomyModuleStub {
public:
    GlobalPlannerModuleShell();

    GlobalPlan plan(const NavigationUpdate& navigation_update,
                    const TraversabilityReport& traversability_report,
                    ContractEnvelope envelope = {});
    [[nodiscard]] GlobalPlan currentPlan() const;

private:
    GlobalPlan plan_{};
    bool had_traversability_report_{false};
    bool previous_traversable_{true};
    double previous_cost_{0.0};
    double previous_risk_{0.0};
};

} // namespace autonomy
