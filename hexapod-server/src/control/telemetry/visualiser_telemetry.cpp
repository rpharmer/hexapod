#include "visualiser_telemetry.hpp"

#include "geometry_config.hpp"

#include <cmath>
#include <sstream>

namespace visualiser_telemetry {
namespace {

std::string escapeJsonString(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        if (ch == '\\' || ch == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(ch);
    }
    return escaped;
}

} // namespace

std::optional<std::string> legIdToVisualiserKey(const LegID leg_id) {
    switch (leg_id) {
        case LegID::L1:
            return "LF";
        case LegID::L2:
            return "LM";
        case LegID::L3:
            return "LR";
        case LegID::R1:
            return "RF";
        case LegID::R2:
            return "RM";
        case LegID::R3:
            return "RR";
    }
    return std::nullopt;
}

JointAnglesDeg jointAnglesRadToDeg(const LegState& leg_state) {
    return {
        rad2deg(leg_state.joint_state[COXA].pos_rad),
        rad2deg(leg_state.joint_state[FEMUR].pos_rad),
        rad2deg(leg_state.joint_state[TIBIA].pos_rad),
    };
}

VisualiserGeometry geometryToVisualiserUnits(const HexapodGeometry& geometry) {
    const LegGeometry& first_leg = geometry.legGeometry.front();
    const double max_radius_m = [] (const HexapodGeometry& source) {
        double radius_m = 0.0;
        for (const auto& leg : source.legGeometry) {
            const double leg_radius = std::sqrt(
                (leg.bodyCoxaOffset.x * leg.bodyCoxaOffset.x) +
                (leg.bodyCoxaOffset.y * leg.bodyCoxaOffset.y));
            if (leg_radius > radius_m) {
                radius_m = leg_radius;
            }
        }
        return radius_m;
    }(geometry);

    return VisualiserGeometry{
        first_leg.coxaLength.value * kMetersToMillimeters,
        first_leg.femurLength.value * kMetersToMillimeters,
        first_leg.tibiaLength.value * kMetersToMillimeters,
        max_radius_m * kMetersToMillimeters,
    };
}

std::string serializeGeometryPayload(const HexapodGeometry& geometry) {
    const VisualiserGeometry visualiser_geometry = geometryToVisualiserUnits(geometry);

    std::ostringstream stream;
    stream << '{'
           << "\"schema_version\":" << kSchemaVersion << ','
           << "\"type\":\"geometry\","
           << "\"geometry\":{"
           << "\"coxa\":" << visualiser_geometry.coxa_mm << ','
           << "\"femur\":" << visualiser_geometry.femur_mm << ','
           << "\"tibia\":" << visualiser_geometry.tibia_mm << ','
           << "\"body_radius\":" << visualiser_geometry.body_radius_mm
           << "}}";
    return stream.str();
}

std::string serializeJointPayload(const RobotState& robot_state) {
    std::ostringstream stream;
    stream << '{'
           << "\"schema_version\":" << kSchemaVersion << ','
           << "\"type\":\"joints\","
           << "\"timestamp_ms\":" << (robot_state.timestamp_us.value / 1000) << ','
           << "\"angles_deg\":{";

    bool first_leg = true;
    for (const auto& leg_geometry : geometry_config::activeHexapodGeometry().legGeometry) {
        const std::optional<std::string> leg_key = legIdToVisualiserKey(leg_geometry.legID);
        if (!leg_key.has_value()) {
            continue;
        }

        if (!first_leg) {
            stream << ',';
        }
        first_leg = false;

        const std::size_t leg_index = static_cast<std::size_t>(leg_geometry.legID);
        const JointAnglesDeg joints_deg = jointAnglesRadToDeg(robot_state.leg_states[leg_index]);
        stream << '\"' << escapeJsonString(*leg_key) << "\":["
               << joints_deg[COXA] << ','
               << joints_deg[FEMUR] << ','
               << joints_deg[TIBIA] << ']';
    }

    stream << "}}";
    return stream.str();
}

} // namespace visualiser_telemetry
