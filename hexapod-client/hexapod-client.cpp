#define TARGET_PICO

#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "serialCommsClient.hpp"
#include "framing.hpp"
#include "servo2040.hpp"
#include "button.hpp"
#include "analog.hpp"
#include "analogmux.hpp"

#include <array>
#include <cstring>


/*
Displays a rotating rainbow pattern on the Servo 2040's onboard LED bar.

Press "Boot" to exit the program.
*/

using namespace plasma;
using namespace servo;


// Firmware hardware/application state grouped into a single context to reduce
// free-floating global mutable state.
struct FirmwareContext {
  Analog sen_adc{servo2040::SHARED_ADC};
  Analog vol_adc{servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN};
  Analog cur_adc{servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                 servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET};

  AnalogMux mux{servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
                PIN_UNUSED, servo2040::SHARED_ADC};

  static constexpr int START_PIN = servo2040::SERVO_1;
  static constexpr int END_PIN = servo2040::SERVO_18;
  static constexpr int NUM_SERVOS = (END_PIN - START_PIN) + 1;

  float minmaxCalibrations[18][2] =
    {{1031, 2088}, {1003, 2016}, {958, 1990}, {941, 2022}, {986, 2039}, {958, 1988},
     {1007, 2048}, {976, 2019}, {1057, 2090}, {993, 2015}, {1011, 2013}, {956, 2000},
     {1040, 2055}, {983, 2057}, {959, 1995}, {1031, 1998}, {951, 1978}, {1035, 2027}};

  ServoCluster servos{pio0, 0, START_PIN, NUM_SERVOS};
  std::array<float, 18> jointTargetPositionsRad{};
  SerialCommsClient serial{};
  WS2812 led_bar{servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA};
  Button user_sw{servo2040::USER_SW};
};

namespace {
FirmwareContext g_firmware{};

// The speed that the LEDs will cycle at
constexpr uint SPEED = 5;

// The brightness of the LEDs
constexpr float BRIGHTNESS = 0.4f;

// How many times the LEDs will be updated per second
constexpr uint UPDATES = 50;

bool dispatchPowerCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case SET_POWER_RELAY:
      handleSetPowerRelayCommand(packet.seq, packet.payload);
      return true;
    case SET_SERVOS_ENABLED:
      handleSetServosEnabledCommand(packet.seq, packet.payload);
      return true;
    case GET_SERVOS_ENABLED:
      handleGetServosEnabledCommand(packet.seq);
      return true;
    case SET_SERVOS_TO_MID:
      handleSetServosToMidCommand(packet.seq);
      return true;
    default:
      return false;
  }
}

bool dispatchSensingCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case GET_ANGLE_CALIBRATIONS:
      handleGetAngleCalibCommand(packet.seq);
      return true;
    case GET_CURRENT:
      handleGetCurrentCommand(packet.seq);
      return true;
    case GET_VOLTAGE:
      handleGetVoltageCommand(packet.seq);
      return true;
    case GET_SENSOR:
      handleGetSensorCommand(packet.seq, packet.payload);
      return true;
    case GET_FULL_HARDWARE_STATE:
      handleGetFullHardwareStateCommand(packet.seq);
      return true;
    default:
      return false;
  }
}

bool dispatchMotionCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case SET_ANGLE_CALIBRATIONS:
      handleCalibCommand(packet.seq, packet.payload);
      return true;
    case SET_TARGET_ANGLE:
      handleSetAngleCommand(packet.seq, packet.payload);
      return true;
    case SET_JOINT_TARGETS:
      handleSetJointTargetsCommand(packet.seq, packet.payload);
      return true;
    default:
      return false;
  }
}

} // namespace

int main() {
  
  stdio_init_all();
  
  // Set up the sensor addresses with pull downs
  g_firmware.mux.configure_pulls(servo2040::SENSOR_1_ADDR, false, true);
  g_firmware.mux.configure_pulls(servo2040::SENSOR_2_ADDR, false, true);
  g_firmware.mux.configure_pulls(servo2040::SENSOR_3_ADDR, false, true);
  g_firmware.mux.configure_pulls(servo2040::SENSOR_4_ADDR, false, true);
  g_firmware.mux.configure_pulls(servo2040::SENSOR_5_ADDR, false, true);
  g_firmware.mux.configure_pulls(servo2040::SENSOR_6_ADDR, false, true);
  
  /* Initialize the servo cluster */
	g_firmware.servos.init();
  
  /* Initialize A0,A1,A2 */
	gpio_init_mask(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK);
	gpio_set_dir_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_OUTPUT_MASK); // Set output
	gpio_put_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_LOW_MASK); // Set LOW
  
  // Calibrate servos using default settings
  calibServos(g_firmware.minmaxCalibrations);
  
  // Start updating the LED bar
  g_firmware.led_bar.start();

  float offset = 0.0f;

  // Make rainbows until the user button is pressed
  while(!stdio_usb_connected()) {

    offset += (float)SPEED / 1000.0f;

    // Update all the LEDs
    for(auto i = 0u; i < servo2040::NUM_LEDS; i++) {
      float hue = (float)i / (float)servo2040::NUM_LEDS;
      g_firmware.led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
    }

    sleep_ms(1000 / UPDATES);
  }

  for(auto i = 0u; i < servo2040::NUM_LEDS; i++){
    g_firmware.led_bar.set_rgb(i, 0.0f, 127.0f, 0.0f);
  }
  
  g_firmware.servos.enable_all();
  
  // Test Serial communication
  //echoLoop();
  
  // Process framed commands
  while(1){
    DecodedPacket packet;
    if(g_firmware.serial.recv_packet(packet))
    {
      if(packet.cmd == HELLO)
      {
        handleHandshake(packet.seq, packet.payload);
      }
      else if(packet.cmd == HEARTBEAT)
      {
        handleHeartbeatCommand(packet.seq);
      }
      else if(dispatchPowerCommand(packet) || dispatchSensingCommand(packet) || dispatchMotionCommand(packet))
      {
      }
      else if(packet.cmd == KILL)
      {
        break;
      }
      else
      {
        g_firmware.serial.send_packet(packet.seq, NACK, {UNSUPPORTED_COMMAND});
      }
    }
  }
  
  // Disable servos
  g_firmware.servos.disable_all();

  // Turn off power relay
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  
  // Turn off the LED bar
  g_firmware.led_bar.clear();

  // Sleep a short time so the clear takes effect
  sleep_ms(100);
}


void handleHandshake(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 2)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t version = payload[0];
  const uint8_t capabilities = payload[1];
  (void)capabilities;

  if(version == PROTOCOL_VERSION)
  {
    g_firmware.serial.send_packet(seq, ACK, {PROTOCOL_VERSION, STATUS_OK, DEVICE_ID});
    return;
  }

  g_firmware.serial.send_packet(seq, NACK, {VERSION_MISMATCH});
}

void echoLoop()
{
  while(1){
    int input = getchar_timeout_us(100);
    if(input >= 0 && input <= 255)
      putchar_raw(input);
  }
}

void handleSetAngleCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 3)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t servo = payload[0];
  if(servo >= FirmwareContext::NUM_SERVOS)
  {
    g_firmware.serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  const uint16_t angle = static_cast<uint16_t>(payload[1]) |
                         (static_cast<uint16_t>(payload[2]) << 8);
  g_firmware.servos.value(servo, angle);
  g_firmware.serial.send_packet(seq, ACK, {});
}

void handleGetAngleCalibCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve(18 * 8);
  for (int s =0; s < 18; s++)
  {
    Calibration& cal = g_firmware.servos.calibration(s);
    float minPulse = cal.first_pulse();
    float maxPulse = cal.last_pulse();

    const uint8_t* minBytes = reinterpret_cast<const uint8_t*>(&minPulse);
    const uint8_t* maxBytes = reinterpret_cast<const uint8_t*>(&maxPulse);
    payload.insert(payload.end(), minBytes, minBytes + sizeof(float));
    payload.insert(payload.end(), maxBytes, maxBytes + sizeof(float));
  }
  g_firmware.serial.send_packet(seq, ACK, payload);
}

void handleSetPowerRelayCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t relayOpen = payload[0];
  if(relayOpen == 1)
    gpio_put_masked(A0_GPIO_MASK, GPIO_HIGH_MASK);
  else if(relayOpen == 0)
    gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  else
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_ARGUMENT});
    return;
  }
  g_firmware.serial.send_packet(seq, ACK, {});
}
void handleGetCurrentCommand(uint16_t seq)
{
  g_firmware.mux.select(servo2040::CURRENT_SENSE_ADDR);
  float current = g_firmware.cur_adc.read_current();
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&current);
  g_firmware.serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}
void handleGetVoltageCommand(uint16_t seq)
{
  g_firmware.mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  float voltage = g_firmware.vol_adc.read_voltage();

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&voltage);
  g_firmware.serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}
void handleGetSensorCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t sensor = payload[0];
  if(sensor >= 6)
  {
    g_firmware.serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  g_firmware.mux.select(servo2040::SENSOR_1_ADDR + sensor);
  float voltage = g_firmware.sen_adc.read_voltage();

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&voltage);
  g_firmware.serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}

void handleCalibCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = 18 * 4 * 2;
  if(payload.size() != expectedPayloadBytes)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  size_t offset = 0;
  float calibs[18][2];
  for (int s =0; s < 18; s++)
  {
    read_scalar(payload, offset, calibs[s][0]);
    read_scalar(payload, offset, calibs[s][1]);
  }
  calibServos(calibs);
  g_firmware.serial.send_packet(seq, ACK, {});
}

void calibServos(float calibs[18][2])
{
  for (int s =0; s < 18; s++)
  {
    Calibration& cal = g_firmware.servos.calibration(s);
    cal.apply_two_pairs(calibs[s][0], calibs[s][1], -45.0f, 45.0f);
  }
  return;
}

void handleHeartbeatCommand(uint16_t seq)
{
  g_firmware.serial.send_packet(seq, ACK, {STATUS_OK});
}


void handleSetJointTargetsCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = 18 * sizeof(float);
  if(payload.size() != expectedPayloadBytes)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  for (size_t s = 0; s < 18; ++s)
  {
    const size_t offset = s * sizeof(float);
    float targetPosRad = 0.0f;
    const uint8_t* src = payload.data() + offset;
    std::memcpy(&targetPosRad, src, sizeof(float));

    Calibration& cal = g_firmware.servos.calibration(static_cast<int>(s));
    constexpr float kRadToDeg = 57.2957795f;
    const float targetPosDeg = targetPosRad * kRadToDeg;
    const uint16_t pulse = static_cast<uint16_t>(cal.value(targetPosDeg));
    g_firmware.servos.value(static_cast<int>(s), pulse);
    g_firmware.jointTargetPositionsRad[s] = targetPosRad;
  }

  g_firmware.serial.send_packet(seq, ACK, {});
}

void handleGetFullHardwareStateCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve((18 * sizeof(float)) + 6 + (2 * sizeof(float)));

  for (int s = 0; s < 18; ++s)
  {
    const float currentPosRad = g_firmware.servos.value(s);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&currentPosRad);
    payload.insert(payload.end(), bytes, bytes + sizeof(float));
  }

  for (int sensor = 0; sensor < 6; ++sensor)
  {
    g_firmware.mux.select(servo2040::SENSOR_1_ADDR + sensor);
    const float sensorVoltage = g_firmware.sen_adc.read_voltage();
    const uint8_t footContact = sensorVoltage > 1.0f ? 1 : 0;
    payload.push_back(footContact);
  }

  g_firmware.mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  const float voltage = g_firmware.vol_adc.read_voltage();
  const uint8_t* voltageBytes = reinterpret_cast<const uint8_t*>(&voltage);
  payload.insert(payload.end(), voltageBytes, voltageBytes + sizeof(float));

  g_firmware.mux.select(servo2040::CURRENT_SENSE_ADDR);
  const float current = g_firmware.cur_adc.read_current();
  const uint8_t* currentBytes = reinterpret_cast<const uint8_t*>(&current);
  payload.insert(payload.end(), currentBytes, currentBytes + sizeof(float));

  g_firmware.serial.send_packet(seq, ACK, payload);
}

void handleSetServosEnabledCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = 18 * sizeof(bool);
  if(payload.size() != expectedPayloadBytes)
  {
    g_firmware.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }
  
  size_t offset = 0;
  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    bool servoEnabled;
    read_scalar(payload, offset, servoEnabled);
    if(servoEnabled)
      g_firmware.servos.enable(s);
    else
      g_firmware.servos.disable(s);
  }
  
  g_firmware.serial.send_packet(seq, ACK, {});
}

void handleGetServosEnabledCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve(18 * sizeof(bool));
  
  for(int s = 0; s < FirmwareContext::NUM_SERVOS; s++)
  {
    append_scalar(payload, g_firmware.servos.is_enabled(s));
  }
  
  g_firmware.serial.send_packet(seq, ACK, payload);
}


void handleSetServosToMidCommand(uint16_t seq)
{
  g_firmware.servos.all_to_mid();
  
  g_firmware.serial.send_packet(seq, ACK, {});
}