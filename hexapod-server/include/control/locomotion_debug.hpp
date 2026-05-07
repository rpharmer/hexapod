#pragma once

#include "types.hpp"

#include <array>
#include <cstdint>

namespace telemetry {

struct LocomotionDebugSnapshot {
    bool valid{false};
    std::array<bool, kNumLegs> planned_stance{};
    std::array<bool, kNumLegs> hold_stance{};
    std::array<bool, kNumLegs> raw_contact{};
    std::array<bool, kNumLegs> fused_load_bearing{};
    std::array<bool, kNumLegs> fusion_phase_active{};
    // Backward-compatible alias for fused load-bearing support.
    std::array<bool, kNumLegs> fused_support{};
    std::array<std::uint8_t, kNumLegs> fused_contact_phase{};
    std::array<double, kNumLegs> fused_contact_confidence{};
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
