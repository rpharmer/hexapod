#pragma once

#include "types.hpp"

#include <array>

namespace telemetry {

struct LocomotionDebugSnapshot {
    bool valid{false};
    std::array<Vec3, kNumLegs> measured_foot_body_m{};
    std::array<Vec3, kNumLegs> measured_foot_world_m{};
    std::array<Vec3, kNumLegs> commanded_foot_body_m{};
    std::array<Vec3, kNumLegs> commanded_foot_world_m{};
    std::array<Vec3, kNumLegs> contact_anchor_world_m{};
    std::array<double, kNumLegs> contact_anchor_drift_m{};
    std::array<double, kNumLegs> contact_anchor_max_drift_m{};
    std::array<double, kNumLegs> commanded_tracking_error_m{};
    std::array<bool, kNumLegs> contact_anchor_valid{};
    double min_measured_foot_world_z_m{0.0};
    double min_commanded_foot_world_z_m{0.0};
    double max_commanded_tracking_error_m{0.0};
};

} // namespace telemetry
