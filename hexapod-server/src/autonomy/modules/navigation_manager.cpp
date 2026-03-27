#include "autonomy/modules/navigation_manager.hpp"
#include "autonomy/modules/shell_utils.hpp"

namespace autonomy {

NavigationManagerModuleShell::NavigationManagerModuleShell()
    : AutonomyModuleStub("navigation_manager") {}

NavigationUpdate NavigationManagerModuleShell::computeIntent(const WaypointMission& mission,
                                                             uint64_t completed_waypoints,
                                                             bool blocked,
                                                             ContractEnvelope envelope) {
    (void)envelope;
    return modules::storeLast(last_update_,
                              navigation_manager_.planNextIntent(mission, completed_waypoints, blocked));
}

NavigationUpdate NavigationManagerModuleShell::lastUpdate() const {
    return last_update_;
}

} // namespace autonomy
