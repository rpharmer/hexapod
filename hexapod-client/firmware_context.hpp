#ifndef FIRMWARE_CONTEXT_HPP
#define FIRMWARE_CONTEXT_HPP

#include "servo2040.hpp"
#include "button.hpp"
#include "analog.hpp"
#include "analogmux.hpp"
#include "serialCommsClient.hpp"
#include "hexapod-client.hpp"

#include <array>

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

  float minmaxCalibrations[kProtocolJointCount][kProtocolCalibrationPairsPerJoint] =
    {{1031, 2088}, {1003, 2016}, {958, 1990}, {941, 2022}, {986, 2039}, {958, 1988},
     {1007, 2048}, {976, 2019}, {1057, 2090}, {993, 2015}, {1011, 2013}, {956, 2000},
     {1040, 2055}, {983, 2057}, {959, 1995}, {1031, 1998}, {951, 1978}, {1035, 2027}};

  ServoCluster servos{pio0, 0, START_PIN, NUM_SERVOS};
  std::array<float, kProtocolJointCount> jointTargetPositionsRad{};
  SerialCommsClient serial{};
  WS2812 led_bar{servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA};
  Button user_sw{servo2040::USER_SW};
  
  HexapodState state{HexapodState::OFF};
};

FirmwareContext& firmware();

#endif
