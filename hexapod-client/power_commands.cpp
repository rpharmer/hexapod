#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"

void handleSetPowerRelayCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t relayOpen = payload[0];
  if(relayOpen == 1)
    gpio_put_masked(A0_GPIO_MASK, GPIO_HIGH_MASK);
  else if(relayOpen == 0)
    gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  else
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_ARGUMENT});
    return;
  }
  firmware().serial.send_packet(seq, ACK, {});
}

void handleSetServosEnabledCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolServoEnablePayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::ServoEnabled enabled{};
  if (!protocol::decode_servo_enabled(payload, enabled))
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    if(enabled[static_cast<std::size_t>(s)] != 0)
      firmware().servos.enable(s);
    else
      firmware().servos.disable(s);
  }

  firmware().serial.send_packet(seq, ACK, {});
}

void handleGetServosEnabledCommand(uint16_t seq)
{
  protocol::ServoEnabled enabled{};

  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    enabled[static_cast<std::size_t>(s)] = firmware().servos.is_enabled(s) ? 1 : 0;
  }

  firmware().serial.send_packet(seq, ACK, protocol::encode_servo_enabled(enabled));
}

void handleSetServosToMidCommand(uint16_t seq)
{
  firmware().servos.all_to_mid();

  firmware().serial.send_packet(seq, ACK, {});
}

void handleHeartbeatCommand(uint16_t seq)
{
  const protocol::HelloAck heartbeat{PROTOCOL_VERSION, STATUS_OK, DEVICE_ID};
  firmware().serial.send_packet(seq, ACK, protocol::encode_hello_ack(heartbeat));
}
