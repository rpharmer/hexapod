#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {

struct Waypoint {
    std::string frame_id{"map"};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
};

struct WaypointMission {
    std::string mission_id{};
    std::vector<Waypoint> waypoints{};
};

struct MissionProgress {
    std::string mission_id{};
    uint64_t completed_waypoints{0};
    uint64_t total_waypoints{0};
};

} // namespace autonomy
