#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/navigation_manager.hpp"

namespace autonomy {

// Thin façade shell: delegates to NavigationManager and snapshots the last output.
class NavigationManagerModuleShell : public AutonomyModuleStub {
public:
    NavigationManagerModuleShell();

    NavigationUpdate computeIntent(const WaypointMission& mission,
                                   uint64_t completed_waypoints,
                                   bool blocked,
                                   ContractEnvelope envelope = {});
    [[nodiscard]] NavigationUpdate lastUpdate() const;

private:
    NavigationManager navigation_manager_{};
    NavigationUpdate last_update_{};
};

} // namespace autonomy
