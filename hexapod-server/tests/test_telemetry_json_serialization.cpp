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

} // namespace

int main()
{
    if (!test_exact_keys_and_schema_version()) {
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

    return EXIT_SUCCESS;
}
