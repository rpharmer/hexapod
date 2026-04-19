#include "replay_json.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
            << "\"has_body_twist_state\":" << (state.has_body_twist_state ? "true" : "false");
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
