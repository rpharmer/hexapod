#pragma once

#include "autonomy/mission_types.hpp"

#include <cstdint>
#include <string>

namespace autonomy {

struct LocalizationEstimate {
    bool valid{false};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
    uint64_t timestamp_ms{0};
};

struct MapSliceInput {
    bool has_occupancy{false};
    bool has_elevation{false};
    bool has_risk_confidence{false};
    double occupancy{0.0};
    double elevation_m{0.0};
    double risk_confidence{1.0};
};

struct WorldModelSnapshot {
    bool has_map{false};
    double occupancy{0.0};
    double elevation_m{0.0};
    double terrain_gradient{0.0};
    double risk_confidence{1.0};
    uint64_t timestamp_ms{0};
};

struct TraversabilityReport {
    bool traversable{true};
    double cost{0.0};
    double risk{0.0};
    double confidence{1.0};
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
