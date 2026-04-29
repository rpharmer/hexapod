#include "visualiser/net/udp_receiver.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <array>
#include <cerrno>
#include <cstring>

namespace visualiser::net {

UdpReceiver::UdpReceiver(int port) {
#ifndef _WIN32
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return;
  }
  const int reuse = 1;
  (void)::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in bind_addr{};
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_port = htons(static_cast<std::uint16_t>(port));
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return;
  }
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    (void)::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }
  valid_ = true;
#else
  (void)port;
#endif
}

UdpReceiver::~UdpReceiver() {
#ifndef _WIN32
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
#endif
}

bool UdpReceiver::valid() const { return valid_; }

visualiser::parsing::PacketDispatchResult UdpReceiver::ClassifyAndDispatch(
    const char* data,
    std::size_t size,
    std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
    visualiser::scene::TerrainPatchState& terrain_patch,
    visualiser::robot::HexapodTelemetryState& telemetry) {
  const std::string payload(data, size);
  return visualiser::parsing::ParsePacket(payload, entities, terrain_patch, telemetry, &terrain_reassembly_);
}

int UdpReceiver::Pump(std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
                      visualiser::scene::TerrainPatchState& terrain_patch,
                      visualiser::robot::HexapodTelemetryState& telemetry,
                      std::uint64_t& accepted_packets,
                      std::uint64_t& rejected_packets,
                      std::string& last_packet_kind,
                      std::string& last_rejection_reason) {
  if (!valid_) {
    return 0;
  }
  int packets = 0;
#ifndef _WIN32
  for (;;) {
    std::array<char, 65536> buffer{};
    const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (bytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      break;
    }
    if (bytes == 0) {
      break;
    }
    const auto result = ClassifyAndDispatch(
        buffer.data(), static_cast<std::size_t>(bytes), entities, terrain_patch, telemetry);
    if (result.accepted) {
      ++accepted_packets;
      ++packets;
      last_packet_kind = result.packet_kind;
    } else {
      ++rejected_packets;
      last_rejection_reason = result.rejection_reason;
    }
  }
#endif
  return packets;
}

}  // namespace visualiser::net
