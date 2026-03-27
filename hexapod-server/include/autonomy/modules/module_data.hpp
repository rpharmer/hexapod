#pragma once

#include "autonomy/mission_types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {

enum class PlannerStatus {
    None,
    Ready,
    NoPlan,
    UnsafePlan,
    StalePlan,
    Degraded,
};

struct LocalizationEstimate {
    bool valid{false};
    std::string frame_id{"map"};
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
    std::vector<Waypoint> route{};
    double cost{0.0};
    PlannerStatus status{PlannerStatus::None};
    std::string reason{};
    uint64_t source_timestamp_ms{0};
};

struct LocalPlan {
    bool has_command{false};
    Waypoint target{};
    PlannerStatus status{PlannerStatus::None};
    std::string reason{};
    bool fallback_active{false};
};

struct LocomotionCommand {
    enum class DispatchStatus {
        Suppressed,
        Dispatched,
        DispatchFailed,
    };

    DispatchStatus status{DispatchStatus::Suppressed};
    bool sent{false};
    bool write_ok{false};
    Waypoint target{};
    std::string reason{};
};

} // namespace autonomy
