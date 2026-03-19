#include "hexapod-common.hpp"
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

  size_t offset = 0;
  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    bool servoEnabled;
    read_scalar(payload, offset, servoEnabled);
    if(servoEnabled)
      firmware().servos.enable(s);
    else
      firmware().servos.disable(s);
  }

  firmware().serial.send_packet(seq, ACK, {});
}

void handleGetServosEnabledCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve(kProtocolServoEnablePayloadBytes);

  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    append_scalar(payload, firmware().servos.is_enabled(s));
  }

  firmware().serial.send_packet(seq, ACK, payload);
}

void handleSetServosToMidCommand(uint16_t seq)
{
  firmware().servos.all_to_mid();

  firmware().serial.send_packet(seq, ACK, {});
}

void handleHeartbeatCommand(uint16_t seq)
{
  firmware().serial.send_packet(seq, ACK, {STATUS_OK});
}
