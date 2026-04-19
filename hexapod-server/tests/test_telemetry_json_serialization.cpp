#include "telemetry_json.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(const bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool containsInOrder(const std::string& text,
                     const std::string& first,
                     const std::string& second,
                     const std::string& third,
                     const std::string& fourth,
                     const std::string& fifth,
                     const std::string& sixth)
{
    const std::size_t i0 = text.find(first);
    const std::size_t i1 = text.find(second);
    const std::size_t i2 = text.find(third);
    const std::size_t i3 = text.find(fourth);
    const std::size_t i4 = text.find(fifth);
    const std::size_t i5 = text.find(sixth);
    return i0 != std::string::npos && i1 != std::string::npos && i2 != std::string::npos &&
           i3 != std::string::npos && i4 != std::string::npos && i5 != std::string::npos &&
           i0 < i1 && i1 < i2 && i2 < i3 && i3 < i4 && i4 < i5;
}

bool test_exact_keys_and_schema_version()
{
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    JointTargets joints{};

    const std::string payload = telemetry_json::serializeVisualiserJointsPacket(geometry, joints, 12345);

    return expect(payload.find("\"type\":\"joints\"") != std::string::npos,
                  "payload should include joints type key") &&
           expect(payload.find("\"schema_version\":1") != std::string::npos,
                  "payload should include fixed schema version") &&
           expect(payload.find("\"timestamp_ms\":12345") != std::string::npos,
                  "payload should include timestamp_ms key") &&
           expect(payload.find("\"geometry\":{") != std::string::npos,
                  "payload should include geometry key") &&
           expect(payload.find("\"angles_deg\":{") != std::string::npos,
                  "payload should include angles_deg key");
}

bool test_geometry_packet_includes_leg_metadata()
{
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    const std::string payload = telemetry_json::serializeGeometryPacket(geometry);

    return expect(payload.find("\"type\":\"geometry\"") != std::string::npos,
                  "geometry packet should include geometry type") &&
           expect(payload.find("\"legs\":[") != std::string::npos,
                  "geometry packet should include per-leg metadata") &&
           expect(payload.find("\"key\":\"LF\"") != std::string::npos,
                  "geometry packet should include canonical leg keys");
}

bool test_leg_order_is_stable_and_canonical()
{
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    JointTargets joints{};

    const std::string payload = telemetry_json::serializeVisualiserJointsPacket(geometry, joints, 100);

    return expect(containsInOrder(payload,
                                  "\"LF\":[",
                                  "\"LM\":[",
                                  "\"LR\":[",
                                  "\"RF\":[",
                                  "\"RM\":[",
                                  "\"RR\":["),
                  "angles leg key order should be LF, LM, LR, RF, RM, RR");
}

bool test_geometry_unit_conversion_meters_to_mm()
{
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        geometry.legGeometry[leg].coxaLength = LengthM{0.035};
        geometry.legGeometry[leg].femurLength = LengthM{0.070};
        geometry.legGeometry[leg].tibiaLength = LengthM{0.110};
        geometry.legGeometry[leg].bodyCoxaOffset = PositionM3{0.06, 0.0, 0.0};
    }
    JointTargets joints{};

    const std::string payload = telemetry_json::serializeVisualiserJointsPacket(geometry, joints, 100);

    return expect(payload.find("\"coxa\":35") != std::string::npos,
                  "coxa length should be serialized in millimeters") &&
           expect(payload.find("\"femur\":70") != std::string::npos,
                  "femur length should be serialized in millimeters") &&
           expect(payload.find("\"tibia\":110") != std::string::npos,
                  "tibia length should be serialized in millimeters") &&
           expect(payload.find("\"body_radius\":60") != std::string::npos,
                  "body radius should be serialized in millimeters");
}

bool test_joint_unit_conversion_radians_to_degrees()
{
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    JointTargets joints{};
    joints.leg_states[0].joint_state[COXA].pos_rad = AngleRad{deg2rad(15.0)};
    joints.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{deg2rad(-20.0)};
    joints.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{deg2rad(35.0)};

    const std::string payload = telemetry_json::serializeVisualiserJointsPacket(geometry, joints, 100);

    return expect(payload.find("\"LF\":[15,-20,35]") != std::string::npos,
                  "angles should be serialized in degrees");
}

bool test_control_step_packet_includes_fusion_diagnostics()
{
    telemetry::ControlStepTelemetry telemetry_sample{};
    telemetry_sample.timestamp_us = TimePointUs{123'000};
    telemetry_sample.status.loop_counter = 7;
    telemetry_sample.status.active_mode = RobotMode::WALK;
    telemetry_sample.status.active_fault = FaultCode::ESTOP;
    telemetry_sample.status.bus_ok = true;
    telemetry_sample.status.estimator_valid = true;
    telemetry_sample.estimated_state.voltage = 11.7f;
    telemetry_sample.estimated_state.current = 1.9f;
    telemetry_sample.joint_targets.leg_states[0].joint_state[COXA].pos_rad = AngleRad{deg2rad(5.0)};
    telemetry_sample.joint_targets.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{deg2rad(-10.0)};
    telemetry_sample.joint_targets.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{deg2rad(15.0)};
    telemetry_sample.fusion.has_data = true;
    telemetry_sample.fusion.diagnostics.model_trust = 0.72;
    telemetry_sample.fusion.diagnostics.resync_requested = true;
    telemetry_sample.fusion.diagnostics.hard_reset_requested = false;
    telemetry_sample.fusion.diagnostics.predictive_mode = true;
    telemetry_sample.fusion.diagnostics.residuals.max_body_position_error_m = 0.04;
    telemetry_sample.fusion.diagnostics.residuals.max_body_orientation_error_rad = 0.08;
    telemetry_sample.fusion.diagnostics.residuals.contact_mismatch_ratio = 0.17;
    telemetry_sample.fusion.diagnostics.residuals.terrain_residual_m = 0.02;
    telemetry_sample.fusion.diagnostics.residuals.body_position_error_m = Vec3{0.01, -0.02, 0.03};
    telemetry_sample.fusion.diagnostics.residuals.body_velocity_error_mps = Vec3{0.1, -0.2, 0.3};
    telemetry_sample.fusion.diagnostics.residuals.body_orientation_error_rad = EulerAnglesRad3{0.01, 0.02, 0.03};
    telemetry_sample.fusion.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
    telemetry_sample.fusion.foot_contact_fusion[0].confidence = 0.91f;
    telemetry_sample.fusion.foot_contact_fusion[0].touchdown_window_start_us = TimePointUs{100'000};
    telemetry_sample.fusion.foot_contact_fusion[0].touchdown_window_end_us = TimePointUs{130'000};
    telemetry_sample.fusion.foot_contact_fusion[0].last_transition_us = TimePointUs{110'000};
    telemetry_sample.fusion.correction.has_data = true;
    telemetry_sample.fusion.correction.mode = telemetry::FusionCorrectionMode::Strong;
    telemetry_sample.fusion.correction.sample_id = 42;
    telemetry_sample.fusion.correction.timestamp_us = TimePointUs{121'000};
    telemetry_sample.fusion.correction.correction_strength = 0.77;
    telemetry_sample.fusion.correction.residuals.max_body_position_error_m = 0.12;
    telemetry_sample.fusion.correction.residuals.max_body_orientation_error_rad = 0.33;
    telemetry_sample.fusion.correction.residuals.contact_mismatch_ratio = 0.26;
    telemetry_sample.fusion.correction.residuals.terrain_residual_m = 0.04;

    const std::string payload = telemetry_json::serializeControlStepPacket(telemetry_sample);

    return expect(payload.find("\"type\":\"joints\"") != std::string::npos,
                  "control step payload should use the joints schema type") &&
           expect(payload.find("\"timestamp_ms\":123") != std::string::npos,
                  "control step payload should include timestamp_ms") &&
           expect(payload.find("\"loop_counter\":7") != std::string::npos,
                  "control step payload should include loop counter") &&
           expect(payload.find("\"active_mode\":\"WALK\"") != std::string::npos,
                  "control step payload should include human readable mode") &&
           expect(payload.find("\"active_fault\":\"ESTOP\"") != std::string::npos,
                  "control step payload should include human readable fault") &&
           expect(payload.find("\"voltage\":11.7") != std::string::npos,
                  "control step payload should include voltage") &&
           expect(payload.find("\"current\":1.9") != std::string::npos,
                  "control step payload should include current") &&
           expect(payload.find("\"fusion\":{") != std::string::npos,
                  "control step payload should include fusion object") &&
           expect(payload.find("\"model_trust\":0.72") != std::string::npos,
                  "fusion payload should include model trust") &&
           expect(payload.find("\"resync_requested\":true") != std::string::npos,
                  "fusion payload should include resync flag") &&
           expect(payload.find("\"predictive_mode\":true") != std::string::npos,
                  "fusion payload should include predictive mode") &&
           expect(payload.find("\"contact_mismatch_ratio\":0.17") != std::string::npos,
                  "fusion payload should include contact mismatch ratio") &&
           expect(payload.find("\"phase\":3") != std::string::npos,
                  "fusion payload should include confirmed stance phase") &&
           expect(payload.find("\"confidence\":0.91") != std::string::npos,
                  "fusion payload should include per-foot confidence") &&
           expect(payload.find("\"correction\":{\"has_data\":true,\"mode\":2") != std::string::npos,
                  "correction payload should include strong mode") &&
           expect(payload.find("\"sample_id\":42") != std::string::npos,
                  "correction payload should include sample id") &&
           expect(payload.find("\"timestamp_us\":121000") != std::string::npos,
                  "correction payload should include timestamp") &&
           expect(payload.find("\"correction_strength\":0.77") != std::string::npos,
                  "correction payload should include correction strength") &&
           expect(payload.find("\"terrain_residual_m\":0.04") != std::string::npos,
                  "correction payload should include terrain residual");
}

} // namespace

int main()
{
    if (!test_exact_keys_and_schema_version()) {
        return EXIT_FAILURE;
    }
    if (!test_geometry_packet_includes_leg_metadata()) {
        return EXIT_FAILURE;
    }
    if (!test_leg_order_is_stable_and_canonical()) {
        return EXIT_FAILURE;
    }
    if (!test_geometry_unit_conversion_meters_to_mm()) {
        return EXIT_FAILURE;
    }
    if (!test_joint_unit_conversion_radians_to_degrees()) {
        return EXIT_FAILURE;
    }
    if (!test_control_step_packet_includes_fusion_diagnostics()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
