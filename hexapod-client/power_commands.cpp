#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"
#include "payload_decode.hpp"

static_assert(FirmwareContext::NUM_LEDS == kProtocolLedCount,
              "Protocol LED count must match firmware LED hardware count");
constexpr bool kHardwareAngleFeedbackAvailable = false;

namespace {

void nackInvalidPayloadLength(FirmwareContext& ctx, uint16_t seq, payload::DecodeStatus)
{
  ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
}

} // namespace

void handleSetPowerRelayCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  uint8_t relayOpen = 0;
  if(!payload::decode_scalar_exact_or_report(payload,
                                             relayOpen,
                                             [&](payload::DecodeStatus status) {
                                               nackInvalidPayloadLength(ctx, seq, status);
                                             }))
  {
    return;
  }

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
  protocol::ServoEnabled enabled{};
  if(!payload::expect_payload_or_report(payload,
                                        kProtocolServoEnablePayloadBytes,
                                        enabled,
                                        protocol::decode_servo_enabled,
                                        [&](payload::DecodeStatus status) {
                                          nackInvalidPayloadLength(ctx, seq, status);
                                        }))
  {
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
  for (float& position_rad : ctx.jointTargetPositionsRad) {
    position_rad = 0.0f;
  }

  ctx.serial.send_packet(seq, ACK, {});
}

void handleGetLedInfoCommand(FirmwareContext& ctx, uint16_t seq)
{
  const protocol::LedInfo info{1, static_cast<uint8_t>(kProtocolLedCount)};
  ctx.serial.send_packet(seq, ACK, protocol::encode_led_info(info));
}

void handleSetLedColorsCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  protocol::LedColors colors{};
  if(!payload::expect_payload_or_report(payload,
                                        kProtocolLedColorsPayloadBytes,
                                        colors,
                                        protocol::decode_led_colors,
                                        [&](payload::DecodeStatus status) {
                                          nackInvalidPayloadLength(ctx, seq, status);
                                        }))
  {
    return;
  }

  for(std::size_t led_index = 0; led_index < kProtocolLedCount; ++led_index)
  {
    const std::size_t offset = led_index * kProtocolLedColorChannels;
    ctx.led_bar.set_rgb(led_index,
                        static_cast<float>(colors[offset]),
                        static_cast<float>(colors[offset + 1]),
                        static_cast<float>(colors[offset + 2]));
  }

  ctx.serial.send_packet(seq, ACK, {});
}


void handleHeartbeatCommand(FirmwareContext& ctx, uint16_t seq)
{
  const uint8_t granted_caps = kHardwareAngleFeedbackAvailable ? CAPABILITY_ANGULAR_FEEDBACK : 0;
  const protocol::HelloAck heartbeat{PROTOCOL_VERSION, STATUS_OK, DEVICE_ID, granted_caps};
  ctx.serial.send_packet(seq, ACK, protocol::encode_hello_ack(heartbeat));
}
