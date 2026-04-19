#pragma once

#include "local_map.hpp"
#include "telemetry_publisher.hpp"
#include "types.hpp"

#include <cstdint>
#include <string>

namespace replay_json {

inline constexpr int kSchemaVersion = 1;

struct ReplayTelemetryRecord {
    TimePointUs timestamp_us{};
    uint64_t sample_id{0};
    ControlStatus status{};
    RobotState estimated_state{};
    LocalMapSnapshot terrain_snapshot{};
};

std::string serializeReplayTelemetryRecord(const ReplayTelemetryRecord& record);

} // namespace replay_json
