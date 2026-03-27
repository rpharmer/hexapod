#include "autonomy/navigation_manager.hpp"

namespace autonomy {

NavigationUpdate NavigationManager::planNextIntent(const WaypointMission& mission,
                                                   uint64_t completed_waypoints,
                                                   bool blocked) const {
    if (mission.mission_id.empty() || mission.waypoints.empty()) {
        return NavigationUpdate{
            .has_intent = false,
            .intent = {},
            .status = NavigationStatus::Idle,
            .reason = "no active mission",
        };
    }

    if (completed_waypoints >= mission.waypoints.size()) {
        return NavigationUpdate{
            .has_intent = false,
            .intent = {},
            .status = NavigationStatus::Complete,
            .reason = {},
        };
    }

    if (blocked) {
        return NavigationUpdate{
            .has_intent = false,
            .intent = {},
            .status = NavigationStatus::Blocked,
            .reason = "navigation blocked",
        };
    }

    return NavigationUpdate{
        .has_intent = true,
        .intent = NavigationIntent{
            .mission_id = mission.mission_id,
            .waypoint_index = completed_waypoints,
            .target = mission.waypoints[completed_waypoints],
        },
        .status = NavigationStatus::Active,
        .reason = {},
    };
}

} // namespace autonomy
