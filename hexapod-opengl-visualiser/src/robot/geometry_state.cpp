#include "visualiser/robot/defaults.hpp"

#include "visualiser/math/geometry.hpp"

namespace visualiser::robot {

HexapodGeometryState MakeDefaultGeometryState() {
  HexapodGeometryState geometry{};
  geometry.valid = false;
  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    geometry.legs[i].key = kLegKeys[i];
    geometry.legs[i].mount_angle_rad = kDefaultLegs[i].mount_angle_deg * visualiser::math::kPi / 180.0f;
    geometry.legs[i].coxa_attach_deg = kDefaultLegs[i].coxa_attach_deg;
    geometry.legs[i].femur_attach_deg = kDefaultLegs[i].femur_attach_deg;
    geometry.legs[i].tibia_attach_deg = kDefaultLegs[i].tibia_attach_deg;
    geometry.legs[i].body_coxa_offset = {
        kDefaultLegs[i].body_coxa_offset[0],
        kDefaultLegs[i].body_coxa_offset[1],
        kDefaultLegs[i].body_coxa_offset[2]};
    geometry.legs[i].coxa_sign = geometry.legs[i].coxa_attach_deg < 0.0f ? -1.0f : 1.0f;
    geometry.legs[i].femur_sign = geometry.legs[i].femur_attach_deg < 0.0f ? -1.0f : 1.0f;
    geometry.legs[i].tibia_sign = geometry.legs[i].tibia_attach_deg < 0.0f ? -1.0f : 1.0f;
  }
  return geometry;
}

}  // namespace visualiser::robot
