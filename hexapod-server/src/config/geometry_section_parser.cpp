#include "geometry_section_parser.hpp"

#include "config_validation.hpp"
#include "geometry_config.hpp"

namespace geometry_section_parser {

void parseGeometrySection(const toml::value& root, ParsedToml& out)
{
  const HexapodGeometry default_geometry = geometry_config::buildDefaultHexapodGeometry();
  std::vector<double> default_mount_angles;
  std::vector<Vec3> default_coxa_offsets;
  std::vector<double> default_femur_attach;
  std::vector<double> default_tibia_attach;
  std::vector<double> default_side_sign;
  std::vector<Vec3> default_positive_tau_s;
  std::vector<Vec3> default_positive_vmax_radps;
  std::vector<Vec3> default_negative_tau_s;
  std::vector<Vec3> default_negative_vmax_radps;
  default_mount_angles.reserve(kNumLegs);
  default_coxa_offsets.reserve(kNumLegs);
  default_femur_attach.reserve(kNumLegs);
  default_tibia_attach.reserve(kNumLegs);
  default_side_sign.reserve(kNumLegs);
  default_positive_tau_s.reserve(kNumLegs);
  default_positive_vmax_radps.reserve(kNumLegs);
  default_negative_tau_s.reserve(kNumLegs);
  default_negative_vmax_radps.reserve(kNumLegs);

  for (const auto& leg : default_geometry.legGeometry) {
    default_mount_angles.push_back(rad2deg(leg.mountAngle));
    default_coxa_offsets.push_back(leg.bodyCoxaOffset);
    default_femur_attach.push_back(rad2deg(leg.servo.femurOffset));
    default_tibia_attach.push_back(rad2deg(leg.servo.tibiaOffset));
    default_side_sign.push_back(leg.servo.femurSign);
    default_positive_tau_s.push_back(
        Vec3{leg.servoDynamics[COXA].positive_direction.tau_s,
             leg.servoDynamics[FEMUR].positive_direction.tau_s,
             leg.servoDynamics[TIBIA].positive_direction.tau_s});
    default_positive_vmax_radps.push_back(
        Vec3{leg.servoDynamics[COXA].positive_direction.vmax_radps,
             leg.servoDynamics[FEMUR].positive_direction.vmax_radps,
             leg.servoDynamics[TIBIA].positive_direction.vmax_radps});
    default_negative_tau_s.push_back(
        Vec3{leg.servoDynamics[COXA].negative_direction.tau_s,
             leg.servoDynamics[FEMUR].negative_direction.tau_s,
             leg.servoDynamics[TIBIA].negative_direction.tau_s});
    default_negative_vmax_radps.push_back(
        Vec3{leg.servoDynamics[COXA].negative_direction.vmax_radps,
             leg.servoDynamics[FEMUR].negative_direction.vmax_radps,
             leg.servoDynamics[TIBIA].negative_direction.vmax_radps});
  }

  out.coxaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaLengthM", default_geometry.legGeometry[0].coxaLength.value, 0.005, 0.30,
      "geometry");
  out.femurLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.FemurLengthM", default_geometry.legGeometry[0].femurLength.value, 0.005,
      0.30, "geometry");
  out.tibiaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.TibiaLengthM", default_geometry.legGeometry[0].tibiaLength.value, 0.005,
      0.40, "geometry");
  out.bodyToBottomM = config_validation::parseDoubleWithFallback(
      root, "Geometry.BodyToBottomM", default_geometry.toBottom.value, 0.005, 0.30, "geometry");
  out.coxaAttachDeg = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaAttachDeg", rad2deg(default_geometry.legGeometry[0].servo.coxaOffset),
      -180.0, 180.0, "geometry");

  out.mountAnglesDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.MountAnglesDeg", default_mount_angles, kNumLegs, -360.0, 360.0, "geometry");
  out.femurAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.FemurAttachDeg", default_femur_attach, kNumLegs, -180.0, 180.0, "geometry");
  out.tibiaAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.TibiaAttachDeg", default_tibia_attach, kNumLegs, -180.0, 180.0, "geometry");
  out.sideSign = config_validation::parseDoubleListWithFallback(
      root, "Geometry.SideSign", default_side_sign, kNumLegs, -1.0, 1.0, "geometry");
  out.coxaOffsetsM = config_validation::parseVec3ListWithFallback(
      root, "Geometry.CoxaOffsetsM", default_coxa_offsets, kNumLegs, -0.30, 0.30, "geometry");
  out.servoDynamicsPositiveTauS = config_validation::parseVec3ListWithFallback(
      root, "Geometry.ServoDynamicsPositiveTauS", default_positive_tau_s, kNumLegs, 0.0, 2.0,
      "geometry");
  out.servoDynamicsPositiveVmaxRadps = config_validation::parseVec3ListWithFallback(
      root, "Geometry.ServoDynamicsPositiveVmaxRadps", default_positive_vmax_radps, kNumLegs, 0.0,
      30.0, "geometry");
  out.servoDynamicsNegativeTauS = config_validation::parseVec3ListWithFallback(
      root, "Geometry.ServoDynamicsNegativeTauS", default_negative_tau_s, kNumLegs, 0.0, 2.0,
      "geometry");
  out.servoDynamicsNegativeVmaxRadps = config_validation::parseVec3ListWithFallback(
      root, "Geometry.ServoDynamicsNegativeVmaxRadps", default_negative_vmax_radps, kNumLegs, 0.0,
      30.0, "geometry");
}

} // namespace geometry_section_parser
