#pragma once

#include <cstdint>
#include <string>

#include "geometry_config.hpp"
#include "telemetry_publisher.hpp"
#include "types.hpp"

namespace telemetry_json {

inline constexpr int kSchemaVersion = 1;

std::string serializeVisualiserJointsPacket(const HexapodGeometry& geometry,
                                            const JointTargets& joints,
                                            uint64_t timestamp_ms);

std::string serializeControlStepPacket(const telemetry::ControlStepTelemetry& telemetry);

} // namespace telemetry_json
