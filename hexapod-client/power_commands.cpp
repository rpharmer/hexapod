#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"

void handleSetPowerRelayCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t relayOpen = payload[0];
  if(relayOpen == 1)
    gpio_put_masked(A0_GPIO_MASK, GPIO_HIGH_MASK);
  else if(relayOpen == 0)
    gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  else
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_ARGUMENT});
    return;
  }
  ctx.serial.send_packet(seq, ACK, {});
}


void handleSetServosEnabledCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolServoEnablePayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::ServoEnabled enabled{};
  if (!protocol::decode_servo_enabled(payload, enabled))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    if(enabled[static_cast<std::size_t>(s)] != 0)
      ctx.servos.enable(s);
    else
      ctx.servos.disable(s);
  }

  ctx.serial.send_packet(seq, ACK, {});
}


void handleGetServosEnabledCommand(FirmwareContext& ctx, uint16_t seq)
{
  protocol::ServoEnabled enabled{};

  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    enabled[static_cast<std::size_t>(s)] = ctx.servos.is_enabled(s) ? 1 : 0;
  }

  ctx.serial.send_packet(seq, ACK, protocol::encode_servo_enabled(enabled));
}


void handleSetServosToMidCommand(FirmwareContext& ctx, uint16_t seq)
{
  ctx.servos.all_to_mid();

  ctx.serial.send_packet(seq, ACK, {});
}


void handleHeartbeatCommand(FirmwareContext& ctx, uint16_t seq)
{
  const protocol::HelloAck heartbeat{PROTOCOL_VERSION, STATUS_OK, DEVICE_ID};
  ctx.serial.send_packet(seq, ACK, protocol::encode_hello_ack(heartbeat));
}

