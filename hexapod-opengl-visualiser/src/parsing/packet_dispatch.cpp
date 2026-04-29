#include "visualiser/parsing/packet_dispatch.hpp"

#include "minphys_viz_protocol.hpp"
#include "visualiser/parsing/json_packets.hpp"

namespace visualiser::parsing {

PacketDispatchResult ParsePacket(const std::string& payload,
                                 std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
                                 visualiser::scene::TerrainPatchState& terrain_patch,
                                 visualiser::robot::HexapodTelemetryState& telemetry,
                                 VizTerrainReassembly* terrain_reassembly) {
  PacketDispatchResult result{};
  if (payload.size() >= sizeof(minphys_viz::VizWireHeader)
      && minphys_viz::IsVizBinaryPayload(reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size())) {
    std::string packet_kind;
    VizTerrainReassembly fallback{};
    VizTerrainReassembly& reassembly = terrain_reassembly ? *terrain_reassembly : fallback;
    if (ParseVizBinaryPacket(reinterpret_cast<const std::uint8_t*>(payload.data()),
                             payload.size(),
                             entities,
                             terrain_patch,
                             packet_kind,
                             reassembly)) {
      result.accepted = true;
      result.packet_kind = packet_kind;
      return result;
    }
    result.rejection_reason = "invalid viz binary packet";
    return result;
  }

  if (ParseEntityPacket(payload, entities)) {
    result.accepted = true;
    result.packet_kind = "entity";
    return result;
  }
  if (ParseTerrainPatchPacket(payload, terrain_patch)) {
    result.accepted = true;
    result.packet_kind = "terrain_patch";
    return result;
  }
  if (ParseHexapodTelemetryPacket(payload, telemetry)) {
    result.accepted = true;
    result.packet_kind = "telemetry";
    return result;
  }
  result.rejection_reason = "unknown json payload type";
  return result;
}

}  // namespace visualiser::parsing
