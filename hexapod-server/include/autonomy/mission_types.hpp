#pragma once

#include "core/math_types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {

struct Waypoint {
    std::string frame_id{"map"};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};

    PositionM3 position_m() const {
        return PositionM3{x_m, y_m, 0.0};
    }

    AngleRad yaw() const {
        return AngleRad{yaw_rad};
    }

    void set_position_m(const PositionM3& position) {
        x_m = position.x;
        y_m = position.y;
    }

    void set_yaw(AngleRad heading) {
        yaw_rad = heading.value;
    }
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
