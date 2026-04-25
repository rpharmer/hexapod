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
    record.estimated_state.foot_contacts = {true, false, true, false, true, false};
    record.estimated_state.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
    record.estimated_state.foot_contact_fusion[1].phase = ContactPhase::Swing;
    record.estimated_state.foot_contact_fusion[0].confidence = 0.95f;
    record.estimated_state.foot_contact_fusion[1].confidence = 0.15f;
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

    record.leg_targets.timestamp_us = TimePointUs{123'451};
    record.leg_targets.feet[0].pos_body_m = PositionM3{0.21, -0.11, -0.17};
    record.leg_targets.feet[0].vel_body_mps = VelocityMps3{0.08, 0.02, 0.01};
    record.leg_targets.feet[1].pos_body_m = PositionM3{0.19, 0.13, -0.16};
    record.leg_targets.feet[1].vel_body_mps = VelocityMps3{0.07, -0.01, 0.0};

    record.gait_state.timestamp_us = TimePointUs{123'452};
    record.gait_state.stride_phase_rate_hz = FrequencyHz{1.25};
    record.gait_state.duty_factor = 0.62;
    record.gait_state.step_length_m = 0.14;
    record.gait_state.swing_height_m = 0.035;
    record.gait_state.swing_time_ease_01 = 0.4;
    record.gait_state.static_stability_margin_m = 0.028;
    record.gait_state.phase = {0.1, 0.6, 0.2, 0.7, 0.3, 0.8};
    record.gait_state.in_stance = {true, false, true, false, true, false};
    record.gait_state.stability_hold_stance = {false, false, false, false, false, false};
    record.gait_state.support_liftoff_clearance_m = {0.015, -0.01, 0.02, -0.03, 0.025, -0.005};
    record.gait_state.support_liftoff_safe_to_lift = {true, true, true, true, true, true};

    record.joint_targets.leg_states[0].joint_state[0].pos_rad = AngleRad{0.11};
    record.joint_targets.leg_states[0].joint_state[1].pos_rad = AngleRad{-0.22};
    record.joint_targets.leg_states[0].joint_state[2].pos_rad = AngleRad{0.33};
    record.joint_targets.leg_states[1].joint_state[0].pos_rad = AngleRad{0.44};
    record.joint_targets.leg_states[1].joint_state[1].pos_rad = AngleRad{-0.55};
    record.joint_targets.leg_states[1].joint_state[2].pos_rad = AngleRad{0.66};

    record.transition_diagnostics.body_height_m = 0.56;
    record.transition_diagnostics.stance_leg_count = 3;
    record.transition_diagnostics.contact_leg_count = 3;
    record.transition_diagnostics.stance_contact_mismatch_count = 1;
    record.transition_diagnostics.joint_tracking_rms_error_rad = {0.03, 0.05, 0.02, 0.01, 0.04, 0.02};
    record.transition_diagnostics.joint_tracking_max_abs_error_rad = {0.06, 0.08, 0.04, 0.02, 0.07, 0.03};

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
           expect(payload.find("\"schema_version\":3") != std::string::npos,
                  "replay payload should include schema version 3") &&
           expect(payload.find("\"terrain_patch\":{\"fresh\":true") != std::string::npos,
                  "replay payload should include terrain patch summary") &&
           expect(payload.find("\"grid_origin_xy\"") != std::string::npos,
                  "replay payload should include grid origin") &&
            expect(payload.find("\"matrix_lidar\":{\"has_matrix_lidar\":true") != std::string::npos,
                  "replay payload should include lidar frame") &&
           expect(payload.find("\"foot_contacts\":[true,false,true,false,true,false]") != std::string::npos,
                  "replay payload should include fused foot contacts") &&
           expect(payload.find("\"leg_targets\":{\"timestamp_us\":123451") != std::string::npos,
                  "replay payload should include leg target telemetry") &&
           expect(payload.find("\"gait_state\":{\"timestamp_us\":123452") != std::string::npos,
                  "replay payload should include gait state telemetry") &&
           expect(payload.find("\"support_liftoff_clearance_m\":[0.015,-0.01,0.02,-0.03,0.025,-0.005]") != std::string::npos,
                  "replay payload should include per-leg support clearances") &&
           expect(payload.find("\"joint_targets\":[[0.11,-0.22,0.33],[0.44,-0.55,0.66]") != std::string::npos,
                  "replay payload should include joint target telemetry") &&
           expect(payload.find("\"transition_diagnostics\":{\"body_height_m\":0.56") != std::string::npos,
                  "replay payload should include walk-entry diagnostics") &&
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
