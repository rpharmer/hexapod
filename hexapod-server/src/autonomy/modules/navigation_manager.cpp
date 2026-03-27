#include "autonomy/modules/navigation_manager.hpp"

namespace autonomy {

NavigationManagerModuleShell::NavigationManagerModuleShell()
    : AutonomyModuleStub("navigation_manager") {}

NavigationUpdate NavigationManagerModuleShell::computeIntent(const WaypointMission& mission,
                                                             uint64_t completed_waypoints,
                                                             bool blocked,
                                                             ContractEnvelope envelope) {
    (void)envelope;
    last_update_ = navigation_manager_.planNextIntent(mission, completed_waypoints, blocked);
    return last_update_;
}

NavigationUpdate NavigationManagerModuleShell::lastUpdate() const {
    return last_update_;
}

} // namespace autonomy
