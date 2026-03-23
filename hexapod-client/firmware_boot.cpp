#define TARGET_PICO

#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"

namespace {
// The speed that the LEDs will cycle at
constexpr uint SPEED = 5;

// The brightness of the LEDs
constexpr float BRIGHTNESS = 0.4f;

// How many times the LEDs will be updated per second
constexpr uint UPDATES = 50;
constexpr bool kHardwareAngleFeedbackAvailable = false;

void configureSensorAddressPulls(FirmwareContext& ctx)
{
  ctx.mux.configure_pulls(servo2040::SENSOR_1_ADDR, false, true);
  ctx.mux.configure_pulls(servo2040::SENSOR_2_ADDR, false, true);
  ctx.mux.configure_pulls(servo2040::SENSOR_3_ADDR, false, true);
  ctx.mux.configure_pulls(servo2040::SENSOR_4_ADDR, false, true);
  ctx.mux.configure_pulls(servo2040::SENSOR_5_ADDR, false, true);
  ctx.mux.configure_pulls(servo2040::SENSOR_6_ADDR, false, true);
}

void initializeRelayAddressLines()
{
  gpio_init_mask(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK);
  gpio_set_dir_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_OUTPUT_MASK);
  gpio_put_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_LOW_MASK);
}

void showWaitForUsbAnimation(FirmwareContext& ctx)
{
  float offset = 0.0f;
  while(!stdio_usb_connected())
  {
    offset += (float)SPEED / 1000.0f;

    for(auto i = 0u; i < servo2040::NUM_LEDS; i++)
    {
      float hue = (float)i / (float)servo2040::NUM_LEDS;
      ctx.led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
    }

    sleep_ms(1000 / UPDATES);
  }

  for(auto i = 0u; i < servo2040::NUM_LEDS; i++)
  {
    ctx.led_bar.set_rgb(i, 0.0f, 127.0f, 0.0f);
  }
}

void initializeHardware(FirmwareContext& ctx)
{
  ctx.state = HexapodState::BOOT;
  stdio_init_all();
  configureSensorAddressPulls(ctx);

  ctx.servos.init();
  initializeRelayAddressLines();
  calibServos(ctx, ctx.minmaxCalibrations);

  ctx.led_bar.start();
  showWaitForUsbAnimation(ctx);
  ctx.servos.enable_all();
}

void shutdownHardware(FirmwareContext& ctx)
{
  ctx.state = HexapodState::STOPPING;
  ctx.servos.disable_all();
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  ctx.led_bar.clear();
  sleep_ms(100);
  ctx.state = HexapodState::OFF;
}

} // namespace

int main()
{
  FirmwareContext& ctx = firmware();
  initializeHardware(ctx);
  runCommandLoop();
  shutdownHardware(ctx);
}

bool handleHandshake(uint16_t seq, const std::vector<uint8_t>& payload)
{
  return handleHandshake(firmware(), seq, payload);
}

bool handleHandshake(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  protocol::HelloRequest request{};
  if(!protocol::decode_hello_request(payload, request))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return false;
  }

  if(request.version == PROTOCOL_VERSION)
  {
    ctx.requestedCapabilities = request.capabilities;
    const uint8_t granted_caps = kHardwareAngleFeedbackAvailable ? CAPABILITY_ANGULAR_FEEDBACK : 0;
    ctx.softwareAngleFeedbackEstimatorEnabled =
      ((request.capabilities & CAPABILITY_ANGULAR_FEEDBACK) != 0) && !kHardwareAngleFeedbackAvailable;
    const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, DEVICE_ID, granted_caps};
    ctx.serial.send_packet(seq, ACK, protocol::encode_hello_ack(ack));
    return true;
  }

  ctx.serial.send_packet(seq, NACK, {VERSION_MISMATCH});
  return false;
}

void echoLoop()
{
  while(1)
  {
    int input = getchar_timeout_us(100);
    if(input >= 0 && input <= 255)
      putchar_raw(input);
  }
}
