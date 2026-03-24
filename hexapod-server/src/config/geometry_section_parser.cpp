#include "geometry_section_parser.hpp"

#include "config_validation.hpp"
#include "geometry_config.hpp"

namespace geometry_section_parser {

void parseGeometrySection(const toml::value& root,
                          ParsedToml& out,
                          std::shared_ptr<logging::AsyncLogger> logger)
{
  const HexapodGeometry default_geometry = geometry_config::buildDefaultHexapodGeometry();
  std::vector<double> default_mount_angles;
  std::vector<Vec3> default_coxa_offsets;
  std::vector<double> default_femur_attach;
  std::vector<double> default_tibia_attach;
  std::vector<double> default_side_sign;
  default_mount_angles.reserve(kNumLegs);
  default_coxa_offsets.reserve(kNumLegs);
  default_femur_attach.reserve(kNumLegs);
  default_tibia_attach.reserve(kNumLegs);
  default_side_sign.reserve(kNumLegs);

  for (const auto& leg : default_geometry.legGeometry) {
    default_mount_angles.push_back(rad2deg(leg.mountAngle));
    default_coxa_offsets.push_back(leg.bodyCoxaOffset);
    default_femur_attach.push_back(rad2deg(leg.servo.femurOffset));
    default_tibia_attach.push_back(rad2deg(leg.servo.tibiaOffset));
    default_side_sign.push_back(leg.servo.femurSign);
  }

  out.coxaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaLengthM", default_geometry.legGeometry[0].coxaLength.value, 0.005, 0.30,
      "geometry", logger);
  out.femurLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.FemurLengthM", default_geometry.legGeometry[0].femurLength.value, 0.005,
      0.30, "geometry", logger);
  out.tibiaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.TibiaLengthM", default_geometry.legGeometry[0].tibiaLength.value, 0.005,
      0.40, "geometry", logger);
  out.bodyToBottomM = config_validation::parseDoubleWithFallback(
      root, "Geometry.BodyToBottomM", default_geometry.toBottom.value, 0.005, 0.30, "geometry", logger);
  out.coxaAttachDeg = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaAttachDeg", rad2deg(default_geometry.legGeometry[0].servo.coxaOffset),
      -180.0, 180.0, "geometry", logger);

  out.mountAnglesDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.MountAnglesDeg", default_mount_angles, kNumLegs, -360.0, 360.0, "geometry", logger);
  out.femurAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.FemurAttachDeg", default_femur_attach, kNumLegs, -180.0, 180.0, "geometry", logger);
  out.tibiaAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.TibiaAttachDeg", default_tibia_attach, kNumLegs, -180.0, 180.0, "geometry", logger);
  out.sideSign = config_validation::parseDoubleListWithFallback(
      root, "Geometry.SideSign", default_side_sign, kNumLegs, -1.0, 1.0, "geometry", logger);
  out.coxaOffsetsM = config_validation::parseVec3ListWithFallback(
      root, "Geometry.CoxaOffsetsM", default_coxa_offsets, kNumLegs, -0.30, 0.30, "geometry", logger);
}

} // namespace geometry_section_parser
