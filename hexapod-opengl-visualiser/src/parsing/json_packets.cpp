#include "visualiser/parsing/json_packets.hpp"

#include <algorithm>
#include <string>

#include "visualiser/math/geometry.hpp"
#include "visualiser/parsing/json_extract.hpp"
#include "visualiser/robot/defaults.hpp"

namespace visualiser::parsing {

using visualiser::scene::ShapeType;

ShapeType ParseShapeType(std::string_view shape_name) {
  if (shape_name == "sphere") return ShapeType::kSphere;
  if (shape_name == "box") return ShapeType::kBox;
  if (shape_name == "plane") return ShapeType::kPlane;
  if (shape_name == "capsule") return ShapeType::kCapsule;
  if (shape_name == "cylinder") return ShapeType::kCylinder;
  if (shape_name == "half_cylinder") return ShapeType::kHalfCylinder;
  if (shape_name == "compound") return ShapeType::kCompound;
  return ShapeType::kUnknown;
}

std::vector<visualiser::scene::CompoundChildState> ParseCompoundChildren(std::string_view payload) {
  std::vector<visualiser::scene::CompoundChildState> children;
  const auto data = ExtractArrayField(payload, "children");
  if (!data.has_value()) {
    return children;
  }
  // Keep parser conservative for now: children parsing remains handled in legacy path.
  return children;
}

bool ParseHexapodGeometryPacket(std::string_view payload, visualiser::robot::HexapodGeometryState& geometry) {
  if (const auto coxa = ExtractFloatField(payload, "coxa_mm")) geometry.coxa_mm = *coxa;
  if (const auto femur = ExtractFloatField(payload, "femur_mm")) geometry.femur_mm = *femur;
  if (const auto tibia = ExtractFloatField(payload, "tibia_mm")) geometry.tibia_mm = *tibia;
  if (const auto body_radius = ExtractFloatField(payload, "body_radius_mm")) geometry.body_radius_mm = *body_radius;

  const auto legs = ExtractArrayField(payload, "legs");
  if (legs.has_value()) {
    for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
      geometry.legs[i].key = visualiser::robot::kLegKeys[i];
      geometry.legs[i].coxa_mm = geometry.coxa_mm;
      geometry.legs[i].femur_mm = geometry.femur_mm;
      geometry.legs[i].tibia_mm = geometry.tibia_mm;
    }
  } else {
    geometry = visualiser::robot::MakeDefaultGeometryState();
  }
  geometry.valid = true;
  return true;
}

bool ParseAnglesPacket(std::string_view payload, std::array<std::array<float, 3>, 6>& angles_deg) {
  const auto joints = ExtractObjectField(payload, "joints");
  if (!joints.has_value()) {
    return false;
  }
  bool has_any = false;
  for (std::size_t i = 0; i < visualiser::robot::kLegKeys.size(); ++i) {
    const std::string leg_key = std::string(visualiser::robot::kLegKeys[i]);
    const auto leg_payload = ExtractObjectField(*joints, leg_key);
    if (!leg_payload.has_value()) {
      continue;
    }
    const auto angles = ExtractFloat3Field(*leg_payload, "angles_deg");
    if (!angles.has_value()) {
      continue;
    }
    angles_deg[i] = *angles;
    has_any = true;
  }
  return has_any;
}

bool ParseHexapodTelemetryPacket(std::string_view payload, visualiser::robot::HexapodTelemetryState& telemetry) {
  const auto type = ExtractStringField(payload, "type");
  if (!type.has_value()) {
    return false;
  }
  if (*type == "geometry") {
    telemetry.has_geometry = ParseHexapodGeometryPacket(payload, telemetry.geometry);
    return telemetry.has_geometry;
  }
  if (*type == "joints") {
    telemetry.has_joints = ParseAnglesPacket(payload, telemetry.angles_deg);
    return telemetry.has_joints;
  }
  if (*type == "status") {
    if (const auto mode = ExtractIntField(payload, "active_mode")) telemetry.status.active_mode = *mode;
    if (const auto fault = ExtractIntField(payload, "active_fault")) telemetry.status.active_fault = *fault;
    if (const auto voltage = ExtractDoubleField(payload, "voltage")) telemetry.status.voltage = static_cast<float>(*voltage);
    if (const auto current = ExtractDoubleField(payload, "current")) telemetry.status.current = static_cast<float>(*current);
    telemetry.status.valid = true;
    return true;
  }
  return false;
}

bool ParseTerrainPatchPacket(const std::string& payload, visualiser::scene::TerrainPatchState& terrain_patch) {
  if (const auto rows = ExtractIntField(payload, "rows")) terrain_patch.rows = *rows;
  if (const auto cols = ExtractIntField(payload, "cols")) terrain_patch.cols = *cols;
  if (const auto cell = ExtractFloatField(payload, "cell_size_m")) terrain_patch.cell_size_m = *cell;
  if (const auto center = ExtractFloat3Field(payload, "center")) {
    terrain_patch.center = {(*center)[0], (*center)[1], (*center)[2]};
  }
  if (const auto heights = ExtractFloatArrayField(payload, "heights")) terrain_patch.heights = *heights;
  terrain_patch.valid = terrain_patch.rows > 0 && terrain_patch.cols > 0;
  return terrain_patch.valid;
}

bool ParseEntityPacket(const std::string& payload, std::map<std::uint32_t, visualiser::scene::EntityState>& entities) {
  const auto type = ExtractStringField(payload, "type");
  if (!type.has_value()) {
    return false;
  }
  const auto id = ExtractUintField(payload, "id");
  if (!id.has_value()) {
    return false;
  }
  visualiser::scene::EntityState& entity = entities[*id];
  entity.id = *id;

  if (*type == "entity_static") {
    if (const auto shape = ExtractStringField(payload, "shape")) entity.shape = ParseShapeType(*shape);
    if (const auto radius = ExtractFloatField(payload, "radius")) entity.radius = *radius;
    if (const auto half_height = ExtractFloatField(payload, "half_height")) entity.half_height = *half_height;
    if (const auto half_extents = ExtractFloat3Field(payload, "half_extents")) {
      entity.half_extents = {(*half_extents)[0], (*half_extents)[1], (*half_extents)[2]};
    }
    entity.has_static = true;
    return true;
  }
  if (*type == "entity_frame") {
    if (const auto position = ExtractFloat3Field(payload, "position")) {
      entity.position = {(*position)[0], (*position)[1], (*position)[2]};
    }
    if (const auto rotation = ExtractFloat4Field(payload, "rotation")) {
      entity.rotation = {(*rotation)[0], (*rotation)[1], (*rotation)[2], (*rotation)[3]};
    }
    entity.has_frame = true;
    return true;
  }

  return false;
}

}  // namespace visualiser::parsing
