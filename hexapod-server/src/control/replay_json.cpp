#include "replay_json.hpp"

#include "command_governor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

namespace replay_json {
namespace {

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

void appendVec3(std::ostringstream& payload, const Vec3& vec)
{
    payload << '[' << formatNumber(vec.x) << ',' << formatNumber(vec.y) << ',' << formatNumber(vec.z)
            << ']';
}

void appendEuler(std::ostringstream& payload, const EulerAnglesRad3& vec)
{
    payload << '[' << formatNumber(vec.x) << ',' << formatNumber(vec.y) << ',' << formatNumber(vec.z)
            << ']';
}

void appendNavPose(std::ostringstream& payload, const NavPose2d& pose)
{
    payload << "{\"x_m\":" << formatNumber(pose.x_m)
            << ",\"y_m\":" << formatNumber(pose.y_m)
            << ",\"yaw_rad\":" << formatNumber(pose.yaw_rad)
            << '}';
}

void appendBodyTwist(std::ostringstream& payload, const BodyTwistState& twist)
{
    payload << "{\"twist_pos_rad\":";
    appendEuler(payload, twist.twist_pos_rad);
    payload << ",\"twist_vel_radps\":";
    appendVec3(payload, Vec3{twist.twist_vel_radps.x, twist.twist_vel_radps.y, twist.twist_vel_radps.z});
    payload << ",\"body_trans_m\":";
    appendVec3(payload, twist.body_trans_m.raw());
    payload << ",\"body_trans_mps\":";
    appendVec3(payload, twist.body_trans_mps.raw());
    payload << '}';
}

void appendGridOrigin(std::ostringstream& payload, const LocalOccupancyGrid& grid)
{
    if (grid.empty()) {
        payload << "null";
        return;
    }

    const double half_w = static_cast<double>(grid.width_cells - 1) * 0.5;
    const double half_h = static_cast<double>(grid.height_cells - 1) * 0.5;
    const double origin_x = grid.center_pose.x_m - half_w * grid.resolution_m;
    const double origin_y = grid.center_pose.y_m - half_h * grid.resolution_m;
    payload << '[' << formatNumber(origin_x) << ',' << formatNumber(origin_y) << ']';
}

void appendOccupancyGrid(std::ostringstream& payload, const LocalOccupancyGrid& grid)
{
    payload << "{\"width_cells\":" << grid.width_cells
            << ",\"height_cells\":" << grid.height_cells
            << ",\"resolution_m\":" << formatNumber(grid.resolution_m)
            << ",\"center_pose\":";
    appendNavPose(payload, grid.center_pose);
    payload << ",\"grid_origin_xy\":";
    appendGridOrigin(payload, grid);
    payload << ",\"cells\":[";
    for (std::size_t i = 0; i < grid.cells.size(); ++i) {
        if (i > 0) {
            payload << ',';
        }
        payload << static_cast<int>(grid.cells[i]);
    }
    payload << "]}";
}

void appendElevationGrid(std::ostringstream& payload, const LocalElevationGrid& grid)
{
    payload << "{\"width_cells\":" << grid.width_cells
            << ",\"height_cells\":" << grid.height_cells
            << ",\"resolution_m\":" << formatNumber(grid.resolution_m)
            << ",\"center_pose\":";
    appendNavPose(payload, grid.center_pose);
    payload << ",\"values\":[";
    for (std::size_t i = 0; i < grid.max_hit_z_m.size(); ++i) {
        if (i > 0) {
            payload << ',';
        }
        const double z = grid.max_hit_z_m[i];
        if (std::isfinite(z)) {
            payload << formatNumber(z);
        } else {
            payload << "null";
        }
    }
    payload << "]}";
}

void appendMatrixLidar(std::ostringstream& payload, const RobotState& state)
{
    payload << "{\"has_matrix_lidar\":" << (state.has_matrix_lidar ? "true" : "false")
            << ",\"valid\":" << (state.matrix_lidar.valid ? "true" : "false")
            << ",\"timestamp_us\":" << state.matrix_lidar.timestamp_us.value
            << ",\"model\":" << static_cast<int>(state.matrix_lidar.model)
            << ",\"cols\":" << static_cast<int>(state.matrix_lidar.cols)
            << ",\"rows\":" << static_cast<int>(state.matrix_lidar.rows)
            << ",\"ranges_mm\":[";
    const std::size_t cell_count =
        std::min<std::size_t>(static_cast<std::size_t>(state.matrix_lidar.cols) *
                                  static_cast<std::size_t>(state.matrix_lidar.rows),
                              state.matrix_lidar.ranges_mm.size());
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (i > 0) {
            payload << ',';
        }
        payload << state.matrix_lidar.ranges_mm[i];
    }
    payload << "]}";
}

void appendEstimatedState(std::ostringstream& payload, const RobotState& state)
{
    payload << "\"estimated_state\":{"
            << "\"sample_id\":" << state.sample_id << ','
            << "\"timestamp_us\":" << state.timestamp_us.value << ','
            << "\"valid\":" << (state.valid ? "true" : "false") << ','
            << "\"bus_ok\":" << (state.bus_ok ? "true" : "false") << ','
            << "\"has_body_twist_state\":" << (state.has_body_twist_state ? "true" : "false")
            << ",\"foot_contacts\":[";
    for (std::size_t leg = 0; leg < state.foot_contacts.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << (state.foot_contacts[leg] ? "true" : "false");
    }
    payload << "],\"foot_contact_phase\":[";
    for (std::size_t leg = 0; leg < state.foot_contact_fusion.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << static_cast<int>(state.foot_contact_fusion[leg].phase);
    }
    payload << "],\"foot_contact_confidence\":[";
    for (std::size_t leg = 0; leg < state.foot_contact_fusion.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << formatNumber(state.foot_contact_fusion[leg].confidence);
    }
    payload << ']';
    if (state.has_body_twist_state) {
        payload << ",\"body_twist_state\":";
        appendBodyTwist(payload, state.body_twist_state);
    }
    if (state.has_imu) {
        payload << ",\"imu\":{"
                << "\"timestamp_us\":" << state.imu.timestamp_us.value << ','
                << "\"valid\":" << (state.imu.valid ? "true" : "false") << ",\"gyro_radps\":";
        appendVec3(payload, Vec3{state.imu.gyro_radps.x, state.imu.gyro_radps.y, state.imu.gyro_radps.z});
        payload << ",\"accel_mps2\":";
        appendVec3(payload, state.imu.accel_mps2);
        payload << '}';
    }
    if (state.has_matrix_lidar) {
        payload << ",\"matrix_lidar\":";
        appendMatrixLidar(payload, state);
    }
    payload << '}';
}

void appendFootTargets(std::ostringstream& payload, const LegTargets& targets)
{
    payload << "\"leg_targets\":{\"timestamp_us\":" << targets.timestamp_us.value << ",\"feet\":[";
    for (std::size_t leg = 0; leg < targets.feet.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << "{\"pos_body_m\":";
        appendVec3(payload, targets.feet[leg].pos_body_m.raw());
        payload << ",\"vel_body_mps\":";
        appendVec3(payload, targets.feet[leg].vel_body_mps.raw());
        payload << '}';
    }
    payload << "]}";
}

void appendGaitState(std::ostringstream& payload, const GaitState& gait)
{
    payload << "\"gait_state\":{\"timestamp_us\":" << gait.timestamp_us.value
            << ",\"stride_phase_rate_hz\":" << formatNumber(gait.stride_phase_rate_hz.value)
            << ",\"duty_factor\":" << formatNumber(gait.duty_factor)
            << ",\"step_length_m\":" << formatNumber(gait.step_length_m)
            << ",\"swing_height_m\":" << formatNumber(gait.swing_height_m)
            << ",\"swing_time_ease_01\":" << formatNumber(gait.swing_time_ease_01)
            << ",\"static_stability_margin_m\":" << formatNumber(gait.static_stability_margin_m)
            << ",\"phase\":[";
    for (std::size_t leg = 0; leg < gait.phase.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << formatNumber(gait.phase[leg]);
    }
    payload << "],\"in_stance\":[";
    for (std::size_t leg = 0; leg < gait.in_stance.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << (gait.in_stance[leg] ? "true" : "false");
    }
    payload << "],\"hold_stance\":[";
    for (std::size_t leg = 0; leg < gait.stability_hold_stance.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << (gait.stability_hold_stance[leg] ? "true" : "false");
    }
    payload << "],\"safe_to_lift\":[";
    for (std::size_t leg = 0; leg < gait.support_liftoff_safe_to_lift.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << (gait.support_liftoff_safe_to_lift[leg] ? "true" : "false");
    }
    payload << "],\"support_liftoff_clearance_m\":[";
    for (std::size_t leg = 0; leg < gait.support_liftoff_clearance_m.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << formatNumber(gait.support_liftoff_clearance_m[leg]);
    }
    payload << "]}";
}

void appendGovernorReplay(std::ostringstream& payload, const CommandGovernorState& g)
{
    payload << "\"governor\":{"
            << "\"severity\":" << formatNumber(g.severity) << ','
            << "\"body_height_delta_m\":" << formatNumber(g.body_height_delta_m) << ','
            << "\"command_scale\":" << formatNumber(g.command_scale) << ','
            << "\"cadence_scale\":" << formatNumber(g.cadence_scale) << ','
            << "\"reasons\":\"0x" << std::hex << std::uppercase
            << static_cast<std::uint32_t>(g.reasons) << std::dec << "\"}";
}

void appendJointTargets(std::ostringstream& payload, const JointTargets& joints)
{
    payload << "\"joint_targets\":[";
    for (std::size_t leg = 0; leg < joints.leg_states.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << '[';
        for (std::size_t joint = 0; joint < joints.leg_states[leg].joint_state.size(); ++joint) {
            if (joint > 0) {
                payload << ',';
            }
            payload << formatNumber(joints.leg_states[leg].joint_state[joint].pos_rad.value);
        }
        payload << ']';
    }
    payload << ']';
}

void appendTransitionDiagnostics(std::ostringstream& payload,
                                 const ReplayTransitionDiagnostics& diagnostics)
{
    payload << "\"transition_diagnostics\":{\"body_height_m\":"
            << formatNumber(diagnostics.body_height_m)
            << ",\"stance_leg_count\":" << diagnostics.stance_leg_count
            << ",\"contact_leg_count\":" << diagnostics.contact_leg_count
            << ",\"stance_contact_mismatch_count\":" << diagnostics.stance_contact_mismatch_count
            << ",\"joint_tracking_rms_error_rad\":[";
    for (std::size_t leg = 0; leg < diagnostics.joint_tracking_rms_error_rad.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << formatNumber(diagnostics.joint_tracking_rms_error_rad[leg]);
    }
    payload << "],\"joint_tracking_max_abs_error_rad\":[";
    for (std::size_t leg = 0; leg < diagnostics.joint_tracking_max_abs_error_rad.size(); ++leg) {
        if (leg > 0) {
            payload << ',';
        }
        payload << formatNumber(diagnostics.joint_tracking_max_abs_error_rad[leg]);
    }
    payload << "]}";
}

} // namespace

std::string serializeReplayTelemetryRecord(const ReplayTelemetryRecord& record)
{
    std::ostringstream payload;
    payload << "{\"type\":\"replay\",\"schema_version\":" << kSchemaVersion << ','
            << "\"timestamp_us\":" << record.timestamp_us.value << ','
            << "\"sample_id\":" << record.sample_id << ','
            << "\"status\":{"
            << "\"active_mode\":" << static_cast<int>(record.status.active_mode) << ','
            << "\"estimator_valid\":" << (record.status.estimator_valid ? "true" : "false") << ','
            << "\"bus_ok\":" << (record.status.bus_ok ? "true" : "false") << ','
            << "\"active_fault\":" << static_cast<int>(record.status.active_fault) << ','
            << "\"loop_counter\":" << record.status.loop_counter << "},";
    appendEstimatedState(payload, record.estimated_state);
    payload << ',';
    appendFootTargets(payload, record.leg_targets);
    payload << ',';
    appendGaitState(payload, record.gait_state);
    payload << ',';
    appendJointTargets(payload, record.joint_targets);
    payload << ',';
    appendGovernorReplay(payload, record.governor);
    payload << ',';
    appendTransitionDiagnostics(payload, record.transition_diagnostics);
    payload << ",\"terrain_patch\":{"
            << "\"fresh\":" << (record.terrain_snapshot.fresh ? "true" : "false") << ','
            << "\"has_observations\":" << (record.terrain_snapshot.has_observations ? "true" : "false") << ','
            << "\"has_primary_observations\":" << (record.terrain_snapshot.has_primary_observations ? "true" : "false") << ','
            << "\"last_observation_timestamp_us\":" << record.terrain_snapshot.last_observation_timestamp.value << ','
            << "\"last_primary_observation_timestamp_us\":" << record.terrain_snapshot.last_primary_observation_timestamp.value << ','
            << "\"nearest_obstacle_distance_m\":" << formatNumber(record.terrain_snapshot.nearest_obstacle_distance_m)
            << ",\"elevation_has_data\":" << (record.terrain_snapshot.elevation_has_data ? "true" : "false")
            << ",\"ground_elevation_has_data\":" << (record.terrain_snapshot.ground_elevation_has_data ? "true" : "false")
            << ",\"raw\":";
    appendOccupancyGrid(payload, record.terrain_snapshot.raw);
    payload << ",\"inflated\":";
    appendOccupancyGrid(payload, record.terrain_snapshot.inflated);
    payload << ",\"elevation_max_hit_z\":";
    appendElevationGrid(payload, record.terrain_snapshot.elevation_max_hit_z);
    payload << ",\"elevation_ground_mean_z\":";
    appendElevationGrid(payload, record.terrain_snapshot.elevation_ground_mean_z);
    payload << "}}";
    return payload.str();
}

} // namespace replay_json
