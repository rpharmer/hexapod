#include "telemetry_json.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace telemetry_json {
namespace {

constexpr std::array<const char*, kNumLegs> kLegOrder = {"LF", "LM", "LR", "RF", "RM", "RR"};
constexpr double kMetersToMillimeters = 1000.0;

std::string formatNumber(const double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    std::string text = out.str();
    const auto trailing = text.find_last_not_of('0');
    if (trailing != std::string::npos && text[trailing] == '.') {
        text.erase(trailing);
    } else if (trailing != std::string::npos) {
        text.erase(trailing + 1);
    }
    if (text.empty()) {
        return "0";
    }
    return text;
}

double averageBodyRadiusM(const HexapodGeometry& geometry)
{
    double sum = 0.0;
    for (const LegGeometry& leg : geometry.legGeometry) {
        sum += std::hypot(leg.bodyCoxaOffset.x, leg.bodyCoxaOffset.y);
    }
    return sum / static_cast<double>(kNumLegs);
}

} // namespace

std::string serializeVisualiserJointsPacket(const HexapodGeometry& geometry,
                                            const JointTargets& joints,
                                            const uint64_t timestamp_ms)
{
    std::ostringstream out;
    out << '{';
    out << "\"type\":\"joints\",";
    out << "\"schema_version\":" << kSchemaVersion << ',';
    out << "\"timestamp_ms\":" << timestamp_ms << ',';

    out << "\"geometry\":{";
    out << "\"coxa\":" << formatNumber(geometry.legGeometry[0].coxaLength.value * kMetersToMillimeters) << ',';
    out << "\"femur\":" << formatNumber(geometry.legGeometry[0].femurLength.value * kMetersToMillimeters)
        << ',';
    out << "\"tibia\":" << formatNumber(geometry.legGeometry[0].tibiaLength.value * kMetersToMillimeters)
        << ',';
    out << "\"body_radius\":" << formatNumber(averageBodyRadiusM(geometry) * kMetersToMillimeters);
    out << "},";

    out << "\"angles_deg\":{";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg > 0) {
            out << ',';
        }
        out << '\"' << kLegOrder[leg] << "\":[";
        out << formatNumber(rad2deg(joints.leg_states[leg].joint_state[COXA].pos_rad)) << ',';
        out << formatNumber(rad2deg(joints.leg_states[leg].joint_state[FEMUR].pos_rad)) << ',';
        out << formatNumber(rad2deg(joints.leg_states[leg].joint_state[TIBIA].pos_rad));
        out << ']';
    }
    out << "}";

    out << '}';
    return out.str();
}

} // namespace telemetry_json
