#include "geometry_config.hpp"

#include "hexapod-server.hpp"
#include "geometry_profile_service.hpp"

#include <stdexcept>
#include <string>

namespace geometry_config {

HexapodGeometry kHexapodGeometry = buildDefaultHexapodGeometry();

void loadFromParsedToml(const ParsedToml& config) {
    HexapodGeometry geometry = buildDefaultHexapodGeometry();

    constexpr std::array<LegID, kNumLegs> leg_ids{
        LegID::R3, LegID::L3, LegID::R2, LegID::L2, LegID::R1, LegID::L1};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& leg_geo = geometry.legGeometry[leg];
        leg_geo.legID = leg_ids[leg];
        leg_geo.mountAngle = AngleRad{deg2rad(config.mountAnglesDeg[leg])};
        leg_geo.bodyCoxaOffset = config.coxaOffsetsM[leg];
        leg_geo.coxaLength = LengthM{config.coxaLengthM};
        leg_geo.femurLength = LengthM{config.femurLengthM};
        leg_geo.tibiaLength = LengthM{config.tibiaLengthM};

        leg_geo.servo.coxaOffset = AngleRad{deg2rad(config.coxaAttachDeg)};
        leg_geo.servo.femurOffset = AngleRad{deg2rad(config.femurAttachDeg[leg])};
        leg_geo.servo.tibiaOffset = AngleRad{deg2rad(config.tibiaAttachDeg[leg])};
        leg_geo.servo.coxaSign = config.sideSign[leg];
        leg_geo.servo.femurSign = config.sideSign[leg];
        leg_geo.servo.tibiaSign = config.sideSign[leg];
    }

    geometry.toBottom = LengthM{config.bodyToBottomM};

    std::string error;
    if (!geometry_profile_service::preview(geometry, &error) ||
        !geometry_profile_service::apply(&error)) {
        throw std::runtime_error("failed to load geometry profile: " + error);
    }
}

} // namespace geometry_config
