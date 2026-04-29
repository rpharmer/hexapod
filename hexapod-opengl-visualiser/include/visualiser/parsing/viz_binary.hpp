#pragma once

#include <cstdint>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"

namespace visualiser::parsing {

struct VizTerrainReassembly {
  int rows = 0;
  int cols = 0;
  std::vector<float> heights{};
};

visualiser::scene::ShapeType ShapeFromViz(int shape);
bool ParseVizBinaryPacket(const std::uint8_t* data,
                          std::size_t size,
                          std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
                          visualiser::scene::TerrainPatchState& terrain_patch,
                          std::string& packet_kind,
                          VizTerrainReassembly& terrain_reassembly);

}  // namespace visualiser::parsing
