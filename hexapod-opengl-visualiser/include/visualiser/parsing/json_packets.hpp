#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string_view>
#include <vector>

#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"

namespace visualiser::parsing {

visualiser::scene::ShapeType ParseShapeType(std::string_view shape_name);
std::vector<visualiser::scene::CompoundChildState> ParseCompoundChildren(std::string_view payload);
bool ParseHexapodGeometryPacket(std::string_view payload, visualiser::robot::HexapodGeometryState& geometry);
bool ParseAnglesPacket(std::string_view payload, std::array<std::array<float, 3>, 6>& angles_deg);
bool ParseHexapodTelemetryPacket(std::string_view payload, visualiser::robot::HexapodTelemetryState& telemetry);
bool ParseTerrainPatchPacket(const std::string& payload, visualiser::scene::TerrainPatchState& terrain_patch);
bool ParseEntityPacket(const std::string& payload, std::map<std::uint32_t, visualiser::scene::EntityState>& entities);

}  // namespace visualiser::parsing
