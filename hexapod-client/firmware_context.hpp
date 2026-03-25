#ifndef FIRMWARE_CONTEXT_HPP
#define FIRMWARE_CONTEXT_HPP

#include "hexapod-client.hpp"
#include "framing.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef HEXAPOD_CLIENT_HOST_TEST

#include "servo2040.hpp"
#include "button.hpp"
#include "analog.hpp"
#include "analogmux.hpp"
#include "serialCommsClient.hpp"

using namespace plasma;
using namespace servo;

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
  static constexpr std::size_t NUM_LEDS = servo2040::NUM_LEDS;

  float minmaxCalibrations[kProtocolJointCount][kProtocolCalibrationPairsPerJoint] =
    {{1031, 2088}, {1003, 2016}, {958, 1990}, {941, 2022}, {986, 2039}, {958, 1988},
     {1007, 2048}, {976, 2019}, {1057, 2090}, {993, 2015}, {1011, 2013}, {956, 2000},
     {1040, 2055}, {983, 2057}, {959, 1995}, {1031, 1998}, {951, 1978}, {1035, 2027}};

  ServoCluster servos{pio0, 0, START_PIN, NUM_SERVOS};
  std::array<float, kProtocolJointCount> jointTargetPositionsRad{};
  uint8_t requestedCapabilities{0};
  bool softwareAngleFeedbackEstimatorEnabled{false};
  SerialCommsClient serial{};
  WS2812 led_bar{servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA};
  Button user_sw{servo2040::USER_SW};

  HexapodState state{HexapodState::OFF};
};

#else

namespace servo2040 {
constexpr int SENSOR_1_ADDR = 0;
constexpr int CURRENT_SENSE_ADDR = 10;
constexpr int VOLTAGE_SENSE_ADDR = 11;
constexpr std::size_t NUM_LEDS = kProtocolLedCount;
}

inline void gpio_put_masked(uint32_t, uint32_t) {}

struct Calibration {
  float first{0.0f};
  float last{0.0f};

  void apply_two_pairs(float first_pulse, float last_pulse, float, float)
  {
    first = first_pulse;
    last = last_pulse;
  }

  float first_pulse() const { return first; }
  float last_pulse() const { return last; }
};

struct ServoCluster {
  std::array<float, kProtocolJointCount> values{};
  std::array<uint8_t, kProtocolJointCount> enabled{};
  std::array<Calibration, kProtocolJointCount> calibrations{};

  void value(int idx, float val) { values[static_cast<std::size_t>(idx)] = val; }
  float value(int idx) const { return values[static_cast<std::size_t>(idx)]; }
  Calibration& calibration(int idx) { return calibrations[static_cast<std::size_t>(idx)]; }

  void enable(int idx) { enabled[static_cast<std::size_t>(idx)] = 1; }
  void disable(int idx) { enabled[static_cast<std::size_t>(idx)] = 0; }
  bool is_enabled(int idx) const { return enabled[static_cast<std::size_t>(idx)] != 0; }

  void all_to_mid()
  {
    for(float& value_ref : values)
      value_ref = 0.0f;
  }

  void disable_all()
  {
    for(uint8_t& enabled_ref : enabled)
      enabled_ref = 0;
  }

  void enable_all()
  {
    for(uint8_t& enabled_ref : enabled)
      enabled_ref = 1;
  }
};

struct Analog {
  float voltage{0.0f};
  float current{0.0f};

  float read_voltage() const { return voltage; }
  float read_current() const { return current; }
};

struct AnalogMux {
  int selected{-1};

  void select(int addr) { selected = addr; }
  void configure_pulls(int, bool, bool) {}
};

struct SerialCommsClient {
  struct SentPacket {
    uint16_t seq;
    uint8_t cmd;
    std::vector<uint8_t> payload;
  };

  std::vector<SentPacket> sent_packets{};

  void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)
  {
    sent_packets.push_back(SentPacket{seq, cmd, payload});
  }

  bool recv_packet(DecodedPacket&) { return false; }
};

struct WS2812 {
  std::array<uint8_t, kProtocolLedColorsPayloadBytes> rgb{};

  void start() {}
  void clear() {}
  void set_hsv(std::size_t, float, float, float) {}
  void set_rgb(std::size_t index, float r, float g, float b)
  {
    const std::size_t offset = index * kProtocolLedColorChannels;
    rgb[offset] = static_cast<uint8_t>(r);
    rgb[offset + 1] = static_cast<uint8_t>(g);
    rgb[offset + 2] = static_cast<uint8_t>(b);
  }
};

struct Button {};

struct FirmwareContext {
  static constexpr int NUM_SERVOS = static_cast<int>(kProtocolJointCount);
  static constexpr std::size_t NUM_LEDS = servo2040::NUM_LEDS;

  Analog sen_adc{};
  Analog vol_adc{};
  Analog cur_adc{};
  AnalogMux mux{};
  float minmaxCalibrations[kProtocolJointCount][kProtocolCalibrationPairsPerJoint]{};
  ServoCluster servos{};
  std::array<float, kProtocolJointCount> jointTargetPositionsRad{};
  uint8_t requestedCapabilities{0};
  bool softwareAngleFeedbackEstimatorEnabled{false};
  SerialCommsClient serial{};
  WS2812 led_bar{};
  Button user_sw{};
  HexapodState state{HexapodState::OFF};
};

#endif

FirmwareContext& firmware();

#endif
