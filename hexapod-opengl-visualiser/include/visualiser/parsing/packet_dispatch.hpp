#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "visualiser/parsing/viz_binary.hpp"
#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"

namespace visualiser::parsing {

struct PacketDispatchResult {
  bool accepted = false;
  std::string packet_kind = "unknown";
  std::string rejection_reason{};
};

PacketDispatchResult ParsePacket(const std::string& payload,
                                 std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
                                 visualiser::scene::TerrainPatchState& terrain_patch,
                                 visualiser::robot::HexapodTelemetryState& telemetry,
                                 VizTerrainReassembly* terrain_reassembly = nullptr);

}  // namespace visualiser::parsing
