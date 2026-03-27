#pragma once

#include "autonomy/mission_types.hpp"

#include <cstdint>
#include <string>

namespace autonomy {

struct LocalizationEstimate {
    bool valid{false};
    std::string frame_id{"map"};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
    uint64_t timestamp_ms{0};
};

struct WorldModelSnapshot {
    bool has_map{false};
    double obstacle_risk{0.0};
    uint64_t timestamp_ms{0};
};

struct TraversabilityReport {
    bool traversable{true};
    double cost{0.0};
    uint64_t timestamp_ms{0};
};

struct GlobalPlan {
    bool has_plan{false};
    Waypoint target{};
    double cost{0.0};
};

struct LocalPlan {
    bool has_command{false};
    Waypoint target{};
};

struct LocomotionCommand {
    bool sent{false};
    Waypoint target{};
    std::string reason{};
};

} // namespace autonomy
