#include "command_dispatch_internal.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "protocol_codec.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message)
{
  if(!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}

const SerialCommsClient::SentPacket* last_packet(const FirmwareContext& ctx)
{
  if(ctx.serial.sent_packets.empty())
    return nullptr;
  return &ctx.serial.sent_packets.back();
}

bool test_waiting_for_host_only_accepts_hello()
{
  FirmwareContext ctx{};
  ctx.state = HexapodState::WAITING_FOR_HOST;

  handleWaitingForHostPacket(ctx, DecodedPacket{1, as_u8(CommandCode::HEARTBEAT), {}});
  if(!expect(ctx.state == HexapodState::WAITING_FOR_HOST, "non-HELLO should be ignored in waiting state"))
    return false;
  if(!expect(ctx.serial.sent_packets.empty(), "non-HELLO should not emit replies in waiting state"))
    return false;

  const protocol::HelloRequest request{PROTOCOL_VERSION, 0};
  handleWaitingForHostPacket(ctx,
                             DecodedPacket{2, as_u8(CommandCode::HELLO), protocol::encode_hello_request(request)});

  if(!expect(ctx.state == HexapodState::ACTIVE, "HELLO should activate controller"))
    return false;

  const auto* packet = last_packet(ctx);
  return expect(packet != nullptr, "HELLO should produce ACK") &&
         expect(packet->cmd == ACK, "HELLO should ACK on successful handshake");
}

bool test_active_unknown_command_sends_unsupported_nack()
{
  FirmwareContext ctx{};
  ctx.state = HexapodState::ACTIVE;

  const bool keep_running = handleActivePacket(ctx, DecodedPacket{9, 0xFF, {}});
  if(!expect(keep_running, "unknown command should not terminate loop"))
    return false;

  const auto* packet = last_packet(ctx);
  return expect(packet != nullptr, "unknown command should send NACK") &&
         expect(packet->seq == 9, "NACK should preserve packet sequence") &&
         expect(packet->cmd == NACK, "unknown command should NACK") &&
         expect(packet->payload.size() == 1 && packet->payload[0] == UNSUPPORTED_COMMAND,
                "unknown command should use UNSUPPORTED_COMMAND");
}

bool test_host_liveness_timeout_transitions_to_safe_waiting_state()
{
  FirmwareContext ctx{};
  ctx.state = HexapodState::ACTIVE;
  ctx.servos.enable_all();

  int64_t last_host_activity_us = 100;
  enforceHostLivenessTimeout(ctx, 2000101, last_host_activity_us);

  if(!expect(ctx.state == HexapodState::WAITING_FOR_HOST, "timeout should transition to WAITING_FOR_HOST"))
    return false;

  for(int i = 0; i < FirmwareContext::NUM_SERVOS; ++i)
  {
    if(!expect(!ctx.servos.is_enabled(i), "timeout should disable all servos"))
      return false;
  }

  return expect(last_host_activity_us == 2000101,
                "timeout enforcement should update last activity timestamp after transition");
}

} // namespace

int main()
{
  if(!test_waiting_for_host_only_accepts_hello())
    return EXIT_FAILURE;
  if(!test_active_unknown_command_sends_unsupported_nack())
    return EXIT_FAILURE;
  if(!test_host_liveness_timeout_transitions_to_safe_waiting_state())
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
