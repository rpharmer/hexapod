#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "visualiser/parsing/packet_dispatch.hpp"
#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"

namespace visualiser::net {

class UdpReceiver {
 public:
  explicit UdpReceiver(int port);
  ~UdpReceiver();

  bool valid() const;
  int Pump(std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
           visualiser::scene::TerrainPatchState& terrain_patch,
           visualiser::robot::HexapodTelemetryState& telemetry,
           std::uint64_t& accepted_packets,
           std::uint64_t& rejected_packets,
           std::string& last_packet_kind,
           std::string& last_rejection_reason);

 private:
  visualiser::parsing::PacketDispatchResult ClassifyAndDispatch(
      const char* data,
      std::size_t size,
      std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
      visualiser::scene::TerrainPatchState& terrain_patch,
      visualiser::robot::HexapodTelemetryState& telemetry);

  int socket_fd_ = -1;
  bool valid_ = false;
  visualiser::parsing::VizTerrainReassembly terrain_reassembly_{};
};

}  // namespace visualiser::net
