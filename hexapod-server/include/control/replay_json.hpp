#pragma once

#include "local_map.hpp"
#include "telemetry_publisher.hpp"
#include "types.hpp"

#include <cstdint>
#include <string>

namespace replay_json {

inline constexpr int kSchemaVersion = 4;

struct ReplayTransitionDiagnostics {
    double body_height_m{0.0};
    int stance_leg_count{0};
    int contact_leg_count{0};
    int stance_contact_mismatch_count{0};
    std::array<double, kNumLegs> joint_tracking_rms_error_rad{};
    std::array<double, kNumLegs> joint_tracking_max_abs_error_rad{};
};

struct ReplayTelemetryRecord {
    TimePointUs timestamp_us{};
    uint64_t sample_id{0};
    ControlStatus status{};
    CommandGovernorState governor{};
    RobotState estimated_state{};
    LegTargets leg_targets{};
    GaitState gait_state{};
    JointTargets joint_targets{};
    LocalMapSnapshot terrain_snapshot{};
    ReplayTransitionDiagnostics transition_diagnostics{};
};

std::string serializeReplayTelemetryRecord(const ReplayTelemetryRecord& record);

} // namespace replay_json
