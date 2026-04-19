#include "replay_json.hpp"
#include "replay_logger.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>

namespace {

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

std::string readText(const std::string& path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

replay_json::ReplayTelemetryRecord makeSampleRecord()
{
    replay_json::ReplayTelemetryRecord record{};
    record.timestamp_us = TimePointUs{123'456};
    record.sample_id = 77;
    record.status.active_mode = RobotMode::WALK;
    record.status.estimator_valid = true;
    record.status.bus_ok = true;
    record.status.active_fault = FaultCode::NONE;
    record.status.loop_counter = 9;

    record.estimated_state.sample_id = 77;
    record.estimated_state.timestamp_us = TimePointUs{123'450};
    record.estimated_state.valid = true;
    record.estimated_state.bus_ok = true;
    record.estimated_state.has_body_twist_state = true;
    record.estimated_state.body_twist_state.body_trans_m.x = 0.12;
    record.estimated_state.body_twist_state.body_trans_m.y = -0.34;
    record.estimated_state.body_twist_state.body_trans_m.z = 0.56;
    record.estimated_state.body_twist_state.twist_pos_rad.x = 0.01;
    record.estimated_state.body_twist_state.twist_pos_rad.y = -0.02;
    record.estimated_state.body_twist_state.twist_pos_rad.z = 0.03;
    record.estimated_state.has_matrix_lidar = true;
    record.estimated_state.matrix_lidar.valid = true;
    record.estimated_state.matrix_lidar.timestamp_us = TimePointUs{123'300};
    record.estimated_state.matrix_lidar.model = 1;
    record.estimated_state.matrix_lidar.cols = 2;
    record.estimated_state.matrix_lidar.rows = 2;
    record.estimated_state.matrix_lidar.ranges_mm[0] = 1000;
    record.estimated_state.matrix_lidar.ranges_mm[1] = 1100;
    record.estimated_state.matrix_lidar.ranges_mm[2] = 1200;
    record.estimated_state.matrix_lidar.ranges_mm[3] = 1300;

    record.terrain_snapshot.raw.width_cells = 2;
    record.terrain_snapshot.raw.height_cells = 2;
    record.terrain_snapshot.raw.resolution_m = 0.05;
    record.terrain_snapshot.raw.center_pose = NavPose2d{1.0, 2.0, 0.25};
    record.terrain_snapshot.raw.cells = {
        LocalMapCellState::Unknown,
        LocalMapCellState::Free,
        LocalMapCellState::Occupied,
        LocalMapCellState::Occupied,
    };
    record.terrain_snapshot.inflated = record.terrain_snapshot.raw;
    record.terrain_snapshot.elevation_max_hit_z.width_cells = 2;
    record.terrain_snapshot.elevation_max_hit_z.height_cells = 2;
    record.terrain_snapshot.elevation_max_hit_z.resolution_m = 0.05;
    record.terrain_snapshot.elevation_max_hit_z.center_pose = NavPose2d{1.0, 2.0, 0.25};
    record.terrain_snapshot.elevation_max_hit_z.max_hit_z_m = {0.1, 0.2, 0.3, 0.4};
    record.terrain_snapshot.elevation_ground_mean_z = record.terrain_snapshot.elevation_max_hit_z;
    record.terrain_snapshot.fresh = true;
    record.terrain_snapshot.has_observations = true;
    record.terrain_snapshot.has_primary_observations = true;
    record.terrain_snapshot.last_observation_timestamp = TimePointUs{123'000};
    record.terrain_snapshot.last_primary_observation_timestamp = TimePointUs{122'500};
    record.terrain_snapshot.elevation_has_data = true;
    record.terrain_snapshot.ground_elevation_has_data = true;
    record.terrain_snapshot.nearest_obstacle_distance_m = 0.18;
    return record;
}

bool testSerializerIncludesKeySections()
{
    const auto payload = replay_json::serializeReplayTelemetryRecord(makeSampleRecord());
    return expect(payload.find("\"type\":\"replay\"") != std::string::npos,
                  "replay payload should include type") &&
           expect(payload.find("\"terrain_patch\":{\"fresh\":true") != std::string::npos,
                  "replay payload should include terrain patch summary") &&
           expect(payload.find("\"grid_origin_xy\"") != std::string::npos,
                  "replay payload should include grid origin") &&
           expect(payload.find("\"matrix_lidar\":{\"has_matrix_lidar\":true") != std::string::npos,
                  "replay payload should include lidar frame") &&
           expect(payload.find("\"sample_id\":77") != std::string::npos,
                  "replay payload should include sample id");
}

bool testFileLoggerWritesNdjson()
{
    const std::string path = "/tmp/hexapod_replay_logging_test.ndjson";
    std::filesystem::remove(path);
    {
        auto logger = replay::makeFileReplayLogger(path, nullptr);
        logger->write(makeSampleRecord());
    }
    const std::string contents = readText(path);
    return expect(contents.find("\"type\":\"replay\"") != std::string::npos,
                  "file logger should write replay json") &&
           expect(contents.find("\"terrain_patch\"") != std::string::npos,
                  "file logger output should include terrain patch") &&
           expect(contents.find('\n') != std::string::npos,
                  "file logger should end records with newline");
}

} // namespace

int main()
{
    if (!testSerializerIncludesKeySections()) {
        return EXIT_FAILURE;
    }
    if (!testFileLoggerWritesNdjson()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
