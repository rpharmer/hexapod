#pragma once

#include "autonomy/mission_types.hpp"
#include "autonomy/navigation_types.hpp"

#include <cstdint>

namespace autonomy {

class NavigationManager {
public:
    NavigationManager() = default;

    NavigationUpdate planNextIntent(const WaypointMission& mission,
                                    uint64_t completed_waypoints,
                                    bool blocked) const;
};

} // namespace autonomy
