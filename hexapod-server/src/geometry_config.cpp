#include "geometry_config.hpp"

#include "hexapod-server.hpp"

namespace geometry_config {

HexapodGeometry kHexapodGeometry = buildDefaultHexapodGeometry();

void loadFromParsedToml(const ParsedToml& config) {
    HexapodGeometry geometry = buildDefaultHexapodGeometry();

    constexpr std::array<LegID, kNumLegs> leg_ids{
        LegID::R3, LegID::L3, LegID::R2, LegID::L2, LegID::R1, LegID::L1};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& leg_geo = geometry.legGeometry[leg];
        leg_geo.legID = leg_ids[leg];
        leg_geo.mountAngle = deg2rad(config.mountAnglesDeg[leg]);
        leg_geo.bodyCoxaOffset = config.coxaOffsetsM[leg];
        leg_geo.coxaLength = config.coxaLengthM;
        leg_geo.femurLength = config.femurLengthM;
        leg_geo.tibiaLength = config.tibiaLengthM;

        leg_geo.servo.coxaOffset = deg2rad(config.coxaAttachDeg);
        leg_geo.servo.femurOffset = deg2rad(config.femurAttachDeg[leg]);
        leg_geo.servo.tibiaOffset = deg2rad(config.tibiaAttachDeg[leg]);
        leg_geo.servo.coxaSign = config.sideSign[leg];
        leg_geo.servo.femurSign = config.sideSign[leg];
        leg_geo.servo.tibiaSign = config.sideSign[leg];
    }

    geometry.toBottom = config.bodyToBottomM;
    kHexapodGeometry = geometry;
}

} // namespace geometry_config
