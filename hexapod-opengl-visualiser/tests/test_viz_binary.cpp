#include "visualiser/parsing/viz_binary.hpp"

#include "minphys_viz_protocol.hpp"

#include <cstdlib>
#include <iostream>
#include <map>

int main() {
  std::array<std::uint8_t, sizeof(minphys_viz::VizWireHeader)> buffer{};
  auto* header = reinterpret_cast<minphys_viz::VizWireHeader*>(buffer.data());
  header->magic[0] = 'M';
  header->magic[1] = 'P';
  header->magic[2] = 'V';
  header->magic[3] = '1';

  std::map<std::uint32_t, visualiser::scene::EntityState> entities;
  visualiser::scene::TerrainPatchState terrain;
  std::string packet_kind;
  visualiser::parsing::VizTerrainReassembly reassembly;
  if (!visualiser::parsing::ParseVizBinaryPacket(buffer.data(), buffer.size(), entities, terrain, packet_kind, reassembly)) {
    std::cerr << "FAIL: parse viz binary\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
