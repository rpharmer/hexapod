#pragma once

#include "types.hpp"

#include <array>

struct ParsedToml;

namespace geometry_config {

inline HexapodGeometry buildDefaultHexapodGeometry() {
    HexapodGeometry geometry{};

    constexpr double coxa_len = 0.043;
    constexpr double femur_len = 0.060;
    constexpr double tibia_len = 0.104;

    constexpr std::array<LegID, kNumLegs> leg_ids{
        LegID::R3, LegID::L3, LegID::R2, LegID::L2, LegID::R1, LegID::L1};

    constexpr std::array<double, kNumLegs> mount_angles_deg{
        143.0, 217.0, 90.0, 270.0, 37.0, 323.0};

    constexpr std::array<Vec3, kNumLegs> coxa_offsets{{
        {0.063, -0.0835, -0.007},
        {-0.063, -0.0835, -0.007},
        {0.0815, 0, -0.007},
        {-0.0815, 0, -0.007},
        {0.063, 0.0835, -0.007},
        {-0.063, 0.0835, -0.007},
    }};

    constexpr double coxa_attach_deg = 0;
    constexpr std::array<double, kNumLegs> femur_attach_deg{35.0, -35.0, 35.0,
                                                             -35.0, 35.0, -35.0};
    constexpr std::array<double, kNumLegs> tibia_attach_deg{83.0, -83.0, 83.0,
                                                             -83.0, 83.0, -83.0};

    constexpr std::array<double, kNumLegs> side_sign{1.0, -1.0, 1.0, -1.0, 1.0,
                                                      -1.0};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& leg_geo = geometry.legGeometry[leg];
        leg_geo.legID = leg_ids[leg];
        leg_geo.bodyCoxaOffset = coxa_offsets[leg];
        leg_geo.mountAngle = deg2rad(mount_angles_deg[leg]);
        leg_geo.coxaLength = coxa_len;
        leg_geo.femurLength = femur_len;
        leg_geo.tibiaLength = tibia_len;

        leg_geo.servo.coxaOffset = deg2rad(coxa_attach_deg);
        leg_geo.servo.femurOffset = deg2rad(femur_attach_deg[leg]);
        leg_geo.servo.tibiaOffset = deg2rad(tibia_attach_deg[leg]);
        leg_geo.servo.coxaSign = side_sign[leg];
        leg_geo.servo.femurSign = side_sign[leg];
        leg_geo.servo.tibiaSign = side_sign[leg];
    }

    geometry.toBottom = 0.040;
    return geometry;
}

extern HexapodGeometry kHexapodGeometry;

void loadFromParsedToml(const ParsedToml& config);

inline const HexapodGeometry& activeHexapodGeometry() {
    return kHexapodGeometry;
}

} // namespace geometry_config

inline HexapodGeometry defaultHexapodGeometry() {
    return geometry_config::activeHexapodGeometry();
}
