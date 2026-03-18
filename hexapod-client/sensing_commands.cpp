#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"

void handleGetAngleCalibCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve(18 * 8);
  for (int s =0; s < 18; s++)
  {
    Calibration& cal = firmware().servos.calibration(s);
    float minPulse = cal.first_pulse();
    float maxPulse = cal.last_pulse();

    const uint8_t* minBytes = reinterpret_cast<const uint8_t*>(&minPulse);
    const uint8_t* maxBytes = reinterpret_cast<const uint8_t*>(&maxPulse);
    payload.insert(payload.end(), minBytes, minBytes + sizeof(float));
    payload.insert(payload.end(), maxBytes, maxBytes + sizeof(float));
  }
  firmware().serial.send_packet(seq, ACK, payload);
}

void handleGetCurrentCommand(uint16_t seq)
{
  firmware().mux.select(servo2040::CURRENT_SENSE_ADDR);
  float current = firmware().cur_adc.read_current();
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&current);
  firmware().serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}

void handleGetVoltageCommand(uint16_t seq)
{
  firmware().mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  float voltage = firmware().vol_adc.read_voltage();

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&voltage);
  firmware().serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}

void handleGetSensorCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  const uint8_t sensor = payload[0];
  if(sensor >= 6)
  {
    firmware().serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  firmware().mux.select(servo2040::SENSOR_1_ADDR + sensor);
  float voltage = firmware().sen_adc.read_voltage();

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&voltage);
  firmware().serial.send_packet(seq, ACK, std::vector<uint8_t>(bytes, bytes + sizeof(float)));
}

void handleGetFullHardwareStateCommand(uint16_t seq)
{
  std::vector<uint8_t> payload;
  payload.reserve((18 * sizeof(float)) + 6 + (2 * sizeof(float)));

  for (int s = 0; s < 18; ++s)
  {
    const float currentPosRad = firmware().servos.value(s);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&currentPosRad);
    payload.insert(payload.end(), bytes, bytes + sizeof(float));
  }

  for (int sensor = 0; sensor < 6; ++sensor)
  {
    firmware().mux.select(servo2040::SENSOR_1_ADDR + sensor);
    const float sensorVoltage = firmware().sen_adc.read_voltage();
    const uint8_t footContact = sensorVoltage > 1.0f ? 1 : 0;
    payload.push_back(footContact);
  }

  firmware().mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  const float voltage = firmware().vol_adc.read_voltage();
  const uint8_t* voltageBytes = reinterpret_cast<const uint8_t*>(&voltage);
  payload.insert(payload.end(), voltageBytes, voltageBytes + sizeof(float));

  firmware().mux.select(servo2040::CURRENT_SENSE_ADDR);
  const float current = firmware().cur_adc.read_current();
  const uint8_t* currentBytes = reinterpret_cast<const uint8_t*>(&current);
  payload.insert(payload.end(), currentBytes, currentBytes + sizeof(float));

  firmware().serial.send_packet(seq, ACK, payload);
}
