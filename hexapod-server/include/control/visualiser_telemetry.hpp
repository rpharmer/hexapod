#pragma once

#include "types.hpp"

#include <array>
#include <optional>
#include <string>

namespace visualiser_telemetry {

inline constexpr int kSchemaVersion = 1;
inline constexpr double kMetersToMillimeters = 1000.0;

using JointAnglesDeg = std::array<double, kJointsPerLeg>;

struct VisualiserGeometry {
    double coxa_mm{0.0};
    double femur_mm{0.0};
    double tibia_mm{0.0};
    double body_radius_mm{0.0};
};

std::optional<std::string> legIdToVisualiserKey(LegID leg_id);
JointAnglesDeg jointAnglesRadToDeg(const LegState& leg_state);
VisualiserGeometry geometryToVisualiserUnits(const HexapodGeometry& geometry);

std::string serializeGeometryPayload(const HexapodGeometry& geometry);
std::string serializeJointPayload(const RobotState& robot_state);

} // namespace visualiser_telemetry
