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

template <typename VecLike>
void appendVec3Json(std::ostringstream& payload, const VecLike& vec) {
    payload << '[' << vec.x << ',' << vec.y << ',' << vec.z << ']';
}

void appendFusionJson(std::ostringstream& payload, const telemetry::FusionTelemetrySnapshot& fusion) {
    payload << "\"fusion\":{"
            << "\"has_data\":" << (fusion.has_data ? "true" : "false") << ","
            << "\"model_trust\":" << fusion.diagnostics.model_trust << ","
            << "\"resync_requested\":" << (fusion.diagnostics.resync_requested ? "true" : "false") << ","
            << "\"hard_reset_requested\":" << (fusion.diagnostics.hard_reset_requested ? "true" : "false") << ","
            << "\"predictive_mode\":" << (fusion.diagnostics.predictive_mode ? "true" : "false") << ","
            << "\"residuals\":{"
            << "\"body_position_error_m\":";
    appendVec3Json(payload, fusion.diagnostics.residuals.body_position_error_m);
    payload << ",\"body_velocity_error_mps\":";
    appendVec3Json(payload, fusion.diagnostics.residuals.body_velocity_error_mps);
    payload << ",\"body_orientation_error_rad\":";
    appendVec3Json(payload, fusion.diagnostics.residuals.body_orientation_error_rad);
    payload << ",\"foot_contact_error\":[";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << fusion.diagnostics.residuals.foot_contact_error[static_cast<std::size_t>(leg)];
    }
    payload << "],"
            << "\"max_body_position_error_m\":" << fusion.diagnostics.residuals.max_body_position_error_m << ","
            << "\"max_body_orientation_error_rad\":"
            << fusion.diagnostics.residuals.max_body_orientation_error_rad << ","
            << "\"contact_mismatch_ratio\":" << fusion.diagnostics.residuals.contact_mismatch_ratio << ","
            << "\"terrain_residual_m\":" << fusion.diagnostics.residuals.terrain_residual_m
            << "},\"feet\":[";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        const FootContactFusion& foot = fusion.foot_contact_fusion[static_cast<std::size_t>(leg)];
        payload << "{\"phase\":" << static_cast<int>(foot.phase)
                << ",\"confidence\":" << foot.confidence
                << ",\"touchdown_window_start_us\":" << foot.touchdown_window_start_us.value
                << ",\"touchdown_window_end_us\":" << foot.touchdown_window_end_us.value
                << ",\"last_transition_us\":" << foot.last_transition_us.value
                << "}";
    }
    payload << "],\"correction\":{"
            << "\"has_data\":" << (fusion.correction.has_data ? "true" : "false") << ","
            << "\"mode\":" << static_cast<int>(fusion.correction.mode) << ","
            << "\"sample_id\":" << fusion.correction.sample_id << ","
            << "\"timestamp_us\":" << fusion.correction.timestamp_us.value << ","
            << "\"correction_strength\":" << fusion.correction.correction_strength << ","
            << "\"residuals\":{"
            << "\"body_position_error_m\":";
    appendVec3Json(payload, fusion.correction.residuals.body_position_error_m);
    payload << ",\"body_velocity_error_mps\":";
    appendVec3Json(payload, fusion.correction.residuals.body_velocity_error_mps);
    payload << ",\"body_orientation_error_rad\":";
    appendVec3Json(payload, fusion.correction.residuals.body_orientation_error_rad);
    payload << ",\"foot_contact_error\":[";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << fusion.correction.residuals.foot_contact_error[static_cast<std::size_t>(leg)];
    }
    payload << "],"
            << "\"max_body_position_error_m\":"
            << fusion.correction.residuals.max_body_position_error_m << ","
            << "\"max_body_orientation_error_rad\":"
            << fusion.correction.residuals.max_body_orientation_error_rad << ","
            << "\"contact_mismatch_ratio\":" << fusion.correction.residuals.contact_mismatch_ratio << ","
            << "\"terrain_residual_m\":" << fusion.correction.residuals.terrain_residual_m
            << "}}";
    payload << '}';
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

std::string serializeControlStepPacket(const telemetry::ControlStepTelemetry& telemetry)
{
    std::ostringstream payload;
    payload << "{\"type\":\"joints\",\"schema_version\":" << kSchemaVersion << ','
            << "\"timestamp_ms\":" << (telemetry.timestamp_us.value / 1000ULL) << ','
            << "\"loop_counter\":" << telemetry.status.loop_counter << ','
            << "\"mode\":" << static_cast<int>(telemetry.status.active_mode) << ','
            << "\"bus_ok\":" << (telemetry.status.bus_ok ? "true" : "false") << ','
            << "\"estimator_valid\":" << (telemetry.status.estimator_valid ? "true" : "false") << ','
            << "\"angles_deg\":{";

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg != 0) {
            payload << ",";
        }
        payload << "\"" << kLegOrder[leg] << "\":["
                << formatNumber(rad2deg(telemetry.joint_targets.leg_states[leg].joint_state[COXA].pos_rad)) << ','
                << formatNumber(rad2deg(telemetry.joint_targets.leg_states[leg].joint_state[FEMUR].pos_rad)) << ','
                << formatNumber(rad2deg(telemetry.joint_targets.leg_states[leg].joint_state[TIBIA].pos_rad))
                << "]";
    }

    payload << "}";
    if (telemetry.navigation.has_value()) {
        const NavigationMonitorSnapshot& nav = telemetry.navigation.value();
        payload << ",\"nav\":{"
                << "\"lifecycle\":" << static_cast<int>(nav.lifecycle) << ','
                << "\"planner_status\":" << static_cast<int>(nav.planner_status) << ','
                << "\"block_reason\":" << static_cast<int>(nav.block_reason) << ','
                << "\"map_fresh\":" << (nav.map_fresh ? "true" : "false") << ','
                << "\"replan_count\":" << nav.replan_count << ','
                << "\"active_segment_waypoint_count\":" << nav.active_segment_waypoint_count << ','
                << "\"active_segment_length_m\":" << nav.active_segment_length_m << ','
                << "\"nearest_obstacle_distance_m\":" << nav.nearest_obstacle_distance_m
                << "}";
    }
    telemetry::FusionTelemetrySnapshot fusion = telemetry.fusion;
    if (!fusion.has_data && telemetry.estimated_state.has_fusion_diagnostics) {
        fusion.has_data = true;
        fusion.diagnostics = telemetry.estimated_state.fusion;
        fusion.foot_contact_fusion = telemetry.estimated_state.foot_contact_fusion;
    }
    payload << ',';
    appendFusionJson(payload, fusion);
    payload << '}';
    return payload.str();
}

} // namespace telemetry_json
