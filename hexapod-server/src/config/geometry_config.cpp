#include "geometry_config.hpp"

#include "hexapod-server.hpp"
#include "geometry_profile_service.hpp"

#include <iomanip>
#include <sstream>
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
        leg_geo.servoDynamics[COXA].positive_direction.tau_s = config.servoDynamicsPositiveTauS[leg].x;
        leg_geo.servoDynamics[FEMUR].positive_direction.tau_s = config.servoDynamicsPositiveTauS[leg].y;
        leg_geo.servoDynamics[TIBIA].positive_direction.tau_s = config.servoDynamicsPositiveTauS[leg].z;
        leg_geo.servoDynamics[COXA].positive_direction.vmax_radps = config.servoDynamicsPositiveVmaxRadps[leg].x;
        leg_geo.servoDynamics[FEMUR].positive_direction.vmax_radps = config.servoDynamicsPositiveVmaxRadps[leg].y;
        leg_geo.servoDynamics[TIBIA].positive_direction.vmax_radps = config.servoDynamicsPositiveVmaxRadps[leg].z;
        leg_geo.servoDynamics[COXA].negative_direction.tau_s = config.servoDynamicsNegativeTauS[leg].x;
        leg_geo.servoDynamics[FEMUR].negative_direction.tau_s = config.servoDynamicsNegativeTauS[leg].y;
        leg_geo.servoDynamics[TIBIA].negative_direction.tau_s = config.servoDynamicsNegativeTauS[leg].z;
        leg_geo.servoDynamics[COXA].negative_direction.vmax_radps = config.servoDynamicsNegativeVmaxRadps[leg].x;
        leg_geo.servoDynamics[FEMUR].negative_direction.vmax_radps = config.servoDynamicsNegativeVmaxRadps[leg].y;
        leg_geo.servoDynamics[TIBIA].negative_direction.vmax_radps = config.servoDynamicsNegativeVmaxRadps[leg].z;
    }

    geometry.toBottom = LengthM{config.bodyToBottomM};

    std::string error;
    if (!geometry_profile_service::preview(geometry, &error) ||
        !geometry_profile_service::apply(&error)) {
        throw std::runtime_error("failed to load geometry profile: " + error);
    }
}

void writeToParsedToml(ParsedToml& config, const HexapodGeometry& geometry) {
    config.mountAnglesDeg.clear();
    config.coxaOffsetsM.clear();
    config.femurAttachDeg.clear();
    config.tibiaAttachDeg.clear();
    config.sideSign.clear();
    config.servoDynamicsPositiveTauS.clear();
    config.servoDynamicsPositiveVmaxRadps.clear();
    config.servoDynamicsNegativeTauS.clear();
    config.servoDynamicsNegativeVmaxRadps.clear();

    config.mountAnglesDeg.reserve(kNumLegs);
    config.coxaOffsetsM.reserve(kNumLegs);
    config.femurAttachDeg.reserve(kNumLegs);
    config.tibiaAttachDeg.reserve(kNumLegs);
    config.sideSign.reserve(kNumLegs);
    config.servoDynamicsPositiveTauS.reserve(kNumLegs);
    config.servoDynamicsPositiveVmaxRadps.reserve(kNumLegs);
    config.servoDynamicsNegativeTauS.reserve(kNumLegs);
    config.servoDynamicsNegativeVmaxRadps.reserve(kNumLegs);

    config.coxaLengthM = geometry.legGeometry[0].coxaLength.value;
    config.femurLengthM = geometry.legGeometry[0].femurLength.value;
    config.tibiaLengthM = geometry.legGeometry[0].tibiaLength.value;
    config.coxaAttachDeg = rad2deg(geometry.legGeometry[0].servo.coxaOffset);
    config.bodyToBottomM = geometry.toBottom.value;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const auto& leg_geo = geometry.legGeometry[leg];
        config.mountAnglesDeg.push_back(rad2deg(leg_geo.mountAngle));
        config.coxaOffsetsM.push_back(leg_geo.bodyCoxaOffset);
        config.femurAttachDeg.push_back(rad2deg(leg_geo.servo.femurOffset));
        config.tibiaAttachDeg.push_back(rad2deg(leg_geo.servo.tibiaOffset));
        config.sideSign.push_back(leg_geo.servo.femurSign);
        config.servoDynamicsPositiveTauS.push_back(
            Vec3{leg_geo.servoDynamics[COXA].positive_direction.tau_s,
                 leg_geo.servoDynamics[FEMUR].positive_direction.tau_s,
                 leg_geo.servoDynamics[TIBIA].positive_direction.tau_s});
        config.servoDynamicsPositiveVmaxRadps.push_back(
            Vec3{leg_geo.servoDynamics[COXA].positive_direction.vmax_radps,
                 leg_geo.servoDynamics[FEMUR].positive_direction.vmax_radps,
                 leg_geo.servoDynamics[TIBIA].positive_direction.vmax_radps});
        config.servoDynamicsNegativeTauS.push_back(
            Vec3{leg_geo.servoDynamics[COXA].negative_direction.tau_s,
                 leg_geo.servoDynamics[FEMUR].negative_direction.tau_s,
                 leg_geo.servoDynamics[TIBIA].negative_direction.tau_s});
        config.servoDynamicsNegativeVmaxRadps.push_back(
            Vec3{leg_geo.servoDynamics[COXA].negative_direction.vmax_radps,
                 leg_geo.servoDynamics[FEMUR].negative_direction.vmax_radps,
                 leg_geo.servoDynamics[TIBIA].negative_direction.vmax_radps});
    }
}

std::string geometrySectionToml(const HexapodGeometry& geometry) {
    ParsedToml config{};
    writeToParsedToml(config, geometry);

    auto appendVec3List = [](std::ostringstream& out, const std::vector<Vec3>& list) {
        out << "[\n";
        for (std::size_t i = 0; i < list.size(); ++i) {
            const Vec3& vec = list[i];
            out << "  [" << vec.x << ", " << vec.y << ", " << vec.z << "]";
            out << (i + 1 < list.size() ? ",\n" : "\n");
        }
        out << "]";
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "[Geometry]\n";
    out << "CoxaLengthM = " << config.coxaLengthM << "\n";
    out << "FemurLengthM = " << config.femurLengthM << "\n";
    out << "TibiaLengthM = " << config.tibiaLengthM << "\n";
    out << "BodyToBottomM = " << config.bodyToBottomM << "\n";
    out << "CoxaAttachDeg = " << config.coxaAttachDeg << "\n";
    out << "MountAnglesDeg = [";
    for (std::size_t i = 0; i < config.mountAnglesDeg.size(); ++i) {
        out << config.mountAnglesDeg[i] << (i + 1 < config.mountAnglesDeg.size() ? ", " : "");
    }
    out << "]\n";
    out << "CoxaOffsetsM = ";
    appendVec3List(out, config.coxaOffsetsM);
    out << "\nFemurAttachDeg = [";
    for (std::size_t i = 0; i < config.femurAttachDeg.size(); ++i) {
        out << config.femurAttachDeg[i] << (i + 1 < config.femurAttachDeg.size() ? ", " : "");
    }
    out << "]\n";
    out << "TibiaAttachDeg = [";
    for (std::size_t i = 0; i < config.tibiaAttachDeg.size(); ++i) {
        out << config.tibiaAttachDeg[i] << (i + 1 < config.tibiaAttachDeg.size() ? ", " : "");
    }
    out << "]\n";
    out << "SideSign = [";
    for (std::size_t i = 0; i < config.sideSign.size(); ++i) {
        out << config.sideSign[i] << (i + 1 < config.sideSign.size() ? ", " : "");
    }
    out << "]\n";
    out << "ServoDynamicsPositiveTauS = ";
    appendVec3List(out, config.servoDynamicsPositiveTauS);
    out << "\nServoDynamicsPositiveVmaxRadps = ";
    appendVec3List(out, config.servoDynamicsPositiveVmaxRadps);
    out << "\nServoDynamicsNegativeTauS = ";
    appendVec3List(out, config.servoDynamicsNegativeTauS);
    out << "\nServoDynamicsNegativeVmaxRadps = ";
    appendVec3List(out, config.servoDynamicsNegativeVmaxRadps);
    out << '\n';
    return out.str();
}

} // namespace geometry_config
