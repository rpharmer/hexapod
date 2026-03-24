#include "visualiser_telemetry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testLegIdMappingIsDeterministic() {
    return expect(visualiser_telemetry::legIdToVisualiserKey(LegID::L1) == std::optional<std::string>{"LF"},
                  "L1 should map to LF") &&
           expect(visualiser_telemetry::legIdToVisualiserKey(LegID::L2) == std::optional<std::string>{"LM"},
                  "L2 should map to LM") &&
           expect(visualiser_telemetry::legIdToVisualiserKey(LegID::L3) == std::optional<std::string>{"LR"},
                  "L3 should map to LR") &&
           expect(visualiser_telemetry::legIdToVisualiserKey(LegID::R1) == std::optional<std::string>{"RF"},
                  "R1 should map to RF") &&
           expect(visualiser_telemetry::legIdToVisualiserKey(LegID::R2) == std::optional<std::string>{"RM"},
                  "R2 should map to RM") &&
           expect(visualiser_telemetry::legIdToVisualiserKey(LegID::R3) == std::optional<std::string>{"RR"},
                  "R3 should map to RR");
}

bool testJointConversionUsesDegrees() {
    LegState leg{};
    leg.joint_state[COXA].pos_rad = AngleRad{deg2rad(90.0)};
    leg.joint_state[FEMUR].pos_rad = AngleRad{deg2rad(-45.0)};
    leg.joint_state[TIBIA].pos_rad = AngleRad{deg2rad(10.0)};

    const auto converted = visualiser_telemetry::jointAnglesRadToDeg(leg);
    return expect(std::abs(converted[COXA] - 90.0) < 1e-9, "coxa should convert to degrees") &&
           expect(std::abs(converted[FEMUR] - (-45.0)) < 1e-9, "femur should convert to degrees") &&
           expect(std::abs(converted[TIBIA] - 10.0) < 1e-9, "tibia should convert to degrees");
}

bool testGeometryConversionUsesMillimeters() {
    HexapodGeometry geometry{};
    for (std::size_t i = 0; i < geometry.legGeometry.size(); ++i) {
        auto& leg = geometry.legGeometry[i];
        leg.legID = static_cast<LegID>(i);
        leg.coxaLength = LengthM{0.043};
        leg.femurLength = LengthM{0.060};
        leg.tibiaLength = LengthM{0.104};
    }

    geometry.legGeometry[0].bodyCoxaOffset = PositionM3{0.100, 0.0, 0.0};
    geometry.legGeometry[1].bodyCoxaOffset = PositionM3{0.090, 0.0, 0.0};

    const auto converted = visualiser_telemetry::geometryToVisualiserUnits(geometry);
    return expect(std::abs(converted.coxa_mm - 43.0) < 1e-9, "coxa length should convert to mm") &&
           expect(std::abs(converted.femur_mm - 60.0) < 1e-9, "femur length should convert to mm") &&
           expect(std::abs(converted.tibia_mm - 104.0) < 1e-9, "tibia length should convert to mm") &&
           expect(std::abs(converted.body_radius_mm - 100.0) < 1e-9, "body radius should convert to mm");
}

} // namespace

int main() {
    if (!testLegIdMappingIsDeterministic() ||
        !testJointConversionUsesDegrees() ||
        !testGeometryConversionUsesMillimeters()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
