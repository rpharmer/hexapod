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


/*
Displays a rotating rainbow pattern on the Servo 2040's onboard LED bar.

Press "Boot" to exit the program.
*/

using namespace plasma;
using namespace servo;


// Set up the shared analog inputs
Analog sen_adc = Analog(servo2040::SHARED_ADC);
Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                        servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

// Set up the analog multiplexer, including the pin for controlling pull-up/pull-down
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);


float minmaxCalibrations[18][2] =
  {{1031, 2088}, //R31 M41
   {1003,  2016}, //R32 M44
   {958,  1990}, //R33 M17
   {941,  2022}, //L31 M33
   {986, 2039}, //L32 M31
   {958,  1988}, //L33 M8
   {1007, 2048}, //R21 M38
   {976,  2019}, //R22 M43
   {1057,  2090}, //R23 M35
   {993,  2015}, //L21 M34
   {1011,  2013}, //L22 M32
   {956,  2000}, //L23 M36
   {1040, 2055}, //R11 M40
   {983, 2057}, //R12 M37
   {959,  1995}, //R13 M16
   {1031,  1998}, //L11 M39
   {951, 1978}, //L12 M30
   {1035, 2027}};//L13 M15
  

/* Create an array of servo pointers */
const int START_PIN = servo2040::SERVO_1;
const int END_PIN = servo2040::SERVO_18;
const int NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);


// The speed that the LEDs will cycle at
const uint SPEED = 5;

// The brightness of the LEDs
constexpr float BRIGHTNESS = 0.4f;

// How many times the LEDs will be updated per second
const uint UPDATES = 50;

// Serial communication class
SerialCommsClient serial;

// Create the LED bar, using PIO 1 and State Machine 0
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

// Create the user button
Button user_sw(servo2040::USER_SW);


int main() {
  
  stdio_init_all();
  
  // Set up the sensor addresses with pull downs
  mux.configure_pulls(servo2040::SENSOR_1_ADDR, false, true);
  mux.configure_pulls(servo2040::SENSOR_2_ADDR, false, true);
  mux.configure_pulls(servo2040::SENSOR_3_ADDR, false, true);
  mux.configure_pulls(servo2040::SENSOR_4_ADDR, false, true);
  mux.configure_pulls(servo2040::SENSOR_5_ADDR, false, true);
  mux.configure_pulls(servo2040::SENSOR_6_ADDR, false, true);
  
  /* Initialize the servo cluster */
	servos.init();
  
  /* Initialize A0,A1,A2 */
	gpio_init_mask(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK);
	gpio_set_dir_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_OUTPUT_MASK); // Set output
	gpio_put_masked(A0_GPIO_MASK | A1_GPIO_MASK | A2_GPIO_MASK, GPIO_LOW_MASK); // Set LOW
  
  // Calibrate servos using default settings
  calibServos(minmaxCalibrations);
  
  // Start updating the LED bar
  led_bar.start();

  float offset = 0.0f;

  // Make rainbows until the user button is pressed
  while(!stdio_usb_connected()) {

    offset += (float)SPEED / 1000.0f;

    // Update all the LEDs
    for(auto i = 0u; i < servo2040::NUM_LEDS; i++) {
      float hue = (float)i / (float)servo2040::NUM_LEDS;
      led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
    }

    sleep_ms(1000 / UPDATES);
  }

  for(auto i = 0u; i < servo2040::NUM_LEDS; i++){
    led_bar.set_rgb(i, 0.0f, 127.0f, 0.0f);
  }
  
  servos.enable_all();
  
  // Test Serial communication
  //echoLoop();
  
  // Listen for handshake
  while(true)
  {
    int input = getchar_timeout_us(100);
    if(input == HELLO)
      if(handleHandshake())
        break;
  }
  
  
  // Process commands
  while(1){
    int input = getchar_timeout_us(1000);
    if(input >= 0 && input <= 255)
    {
      switch(input & 0xff)
      {
        case HELLO:
        {
          handleHandshake();
          break;
        }
        case SET_ANGLE_CALIBRATIONS:
        {
          handleCalibCommand();
          break;
        }
        case SET_TARGET_ANGLE:
        {
          handleSetAngleCommand();
          break;
        }
        case SET_POWER_RELAY:
        {
          handleSetPowerRelayCommand();
          break;
        }
        case GET_ANGLE_CALIBRATIONS:
        {
          handleGetAngleCalibCommand();
          break;
        }
        case GET_CURRENT:
        {
          handleGetCurrentCommand();
          break;
        }
        case GET_VOLTAGE:
        {
          handleGetVoltageCommand();
          break;
        }
        case GET_SENSOR:
        {
          handleGetSensorCommand();
          break;
        }
        default:
        {
          break;
        }
      }
    }
  }
  
  // Disable servos
  servos.enable_all();

  // Turn off power relay
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  
  // Turn off the LED bar
  led_bar.clear();

  // Sleep a short time so the clear takes effect
  sleep_ms(100);
}


bool handleHandshake()
{
  uint8_t version = 0;
  uint8_t capabilities = 0;

  if(serial.recv_u8(&version) < 0 || serial.recv_u8(&capabilities) < 0)
  {
    // `NACK (0x12)` and an error code
    serial.send_u8(NACK);
    serial.send_u8(TIMEOUT);
    return false;
  }

  if(version == PROTOCOL_VERSION)
  {
    // `ACK (0x11)`, `PROTOCOL_VERSION`, `STATUS (0=ok)`, `DEVICE_ID`
    serial.send_u8(ACK);
    serial.send_u8(PROTOCOL_VERSION);
    serial.send_u8(STATUS_OK);
    serial.send_u8(DEVICE_ID);
    return true;
  }

  // `NACK (0x12)` and an error code
  serial.send_u8(NACK);
  serial.send_u8(VERSION_MISMATCH);
  return false;
}

void echoLoop()
{
  while(1){
    int input = getchar_timeout_us(100);
    if(input >= 0 && input <= 255)
      putchar_raw(input);
  }
}

void handleSetAngleCommand()
{
  uint8_t servo;
  uint16_t angle;
  serial.recv_u8(&servo);
  serial.recv_u16(&angle);
  servos.value(servo, angle);
}

void handleGetAngleCalibCommand()
{
  for (int s =0; s < 18; s++)
  {
    Calibration& cal = servos.calibration(s);
    float minPulse = cal.first_pulse();
    float maxPulse = cal.last_pulse();
    
    serial.send_f32(minPulse);
    serial.send_f32(maxPulse);
  }
}

void handleSetPowerRelayCommand()
{
  uint8_t relayOpen;
  serial.recv_u8(&relayOpen);
  if(relayOpen == 1)
    gpio_put_masked(A0_GPIO_MASK, GPIO_HIGH_MASK);
  else if(relayOpen == 0)
    gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  else
  {
    return;// An error has occured
  }
}
void handleGetCurrentCommand()
{
  mux.select(servo2040::CURRENT_SENSE_ADDR);
  float current = cur_adc.read_current();
  serial.send_f32(current);
}
void handleGetVoltageCommand()
{
  mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  float voltage = vol_adc.read_voltage();
  
  serial.send_f32(voltage);
}
void handleGetSensorCommand()
{
  // get sensor id
  uint8_t sensor;
  serial.recv_u8(&sensor);
  mux.select(servo2040::SENSOR_1_ADDR + sensor);
  float voltage = sen_adc.read_voltage();
  
  serial.send_f32(voltage);
}

void handleCalibCommand()
{
  float calibs[18][2];
  for (int s =0; s < 18; s++)
  {
    uint16_t c00;
    uint16_t c10;
    
    serial.recv_u16(&c00);
    serial.recv_u16(&c10);
    
    calibs[s][0] = c00;
    calibs[s][1] = c10;
  }
  calibServos(calibs);
}

void calibServos(float calibs[18][2])
{
  for (int s =0; s < 18; s++)
  {
    Calibration& cal = servos.calibration(s);
    cal.apply_two_pairs(calibs[s][0], calibs[s][1], -45.0f, 45.0f);
  }
  return;
}