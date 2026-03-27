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

enum class MapDataProvenance {
    Unknown,
    Sensor,
    Map,
    Fused,
    Inferred,
    SafetyOverride,
};

struct LocalizationEstimate {
    bool valid{false};
    std::string frame_id{"map"};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
    TimestampMs timestamp_ms{};

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

struct MapSliceInput {
    std::string frame_id{"map"};
    bool has_timestamp{false};
    TimestampMs timestamp_ms{};
    bool has_occupancy{false};
    bool has_elevation{false};
    bool has_risk_confidence{false};
    double occupancy{0.0};
    double elevation_m{0.0};
    double risk_confidence{1.0};
    MapDataProvenance occupancy_provenance{MapDataProvenance::Unknown};
    MapDataProvenance elevation_provenance{MapDataProvenance::Unknown};
    MapDataProvenance risk_confidence_provenance{MapDataProvenance::Unknown};
};

struct WorldModelSnapshot {
    bool has_map{false};
    std::string frame_id{"map"};
    bool stale_input_rejected{false};
    bool frame_mismatch{false};
    bool non_monotonic_timestamp{false};
    double occupancy{0.0};
    double elevation_m{0.0};
    double terrain_gradient{0.0};
    double risk_confidence{1.0};
    MapDataProvenance occupancy_provenance{MapDataProvenance::Unknown};
    MapDataProvenance elevation_provenance{MapDataProvenance::Unknown};
    MapDataProvenance risk_confidence_provenance{MapDataProvenance::Unknown};
    double obstacle_band_near{0.0};
    double obstacle_band_mid{0.0};
    double obstacle_band_far{0.0};
    double slope_band_low{0.0};
    double slope_band_mid{0.0};
    double slope_band_high{0.0};
    double confidence_zone_nominal{1.0};
    double confidence_zone_degraded{0.0};
    double confidence_zone_unknown{0.0};
    TimestampMs timestamp_ms{};
};

struct TraversabilityReport {
    bool traversable{true};
    double cost{0.0};
    double risk{0.0};
    double confidence{1.0};
    TimestampMs timestamp_ms{};
};

struct GlobalPlan {
    bool has_plan{false};
    Waypoint target{};
    std::vector<Waypoint> route{};
    double cost{0.0};
    PlannerStatus status{PlannerStatus::None};
    std::string reason{};
    TimestampMs source_timestamp_ms{};
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
