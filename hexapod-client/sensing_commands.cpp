#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"

void handleGetAngleCalibCommand(uint16_t seq)
{
  protocol::Calibrations calibrations{};
  std::size_t calib_idx = 0;
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    Calibration& cal = firmware().servos.calibration(static_cast<int>(s));
    calibrations[calib_idx++] = cal.first_pulse();
    calibrations[calib_idx++] = cal.last_pulse();
  }

  firmware().serial.send_packet(seq, ACK, protocol::encode_calibrations(calibrations));
}

void handleGetCurrentCommand(uint16_t seq)
{
  firmware().mux.select(servo2040::CURRENT_SENSE_ADDR);
  const protocol::ScalarFloat current{firmware().cur_adc.read_current()};
  firmware().serial.send_packet(seq, ACK, protocol::encode_scalar_float(current));
}

void handleGetVoltageCommand(uint16_t seq)
{
  firmware().mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  const protocol::ScalarFloat voltage{firmware().vol_adc.read_voltage()};
  firmware().serial.send_packet(seq, ACK, protocol::encode_scalar_float(voltage));
}

void handleGetSensorCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != 1)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  uint8_t sensor = 0;
  std::size_t offset = 0;
  if(!read_scalar(payload, offset, sensor) || offset != payload.size())
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }
  if(sensor >= kProtocolFootSensorCount)
  {
    firmware().serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  firmware().mux.select(servo2040::SENSOR_1_ADDR + sensor);
  const protocol::ScalarFloat voltage{firmware().sen_adc.read_voltage()};
  firmware().serial.send_packet(seq, ACK, protocol::encode_scalar_float(voltage));
}

void handleGetFullHardwareStateCommand(uint16_t seq)
{
  protocol::FullHardwareState state{};

  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    state.joint_positions_rad[s] = firmware().servos.value(static_cast<int>(s));
  }

  for (std::size_t sensor = 0; sensor < kProtocolFootSensorCount; ++sensor)
  {
    firmware().mux.select(servo2040::SENSOR_1_ADDR + static_cast<int>(sensor));
    const float sensorVoltage = firmware().sen_adc.read_voltage();
    state.foot_contacts[sensor] = sensorVoltage > 1.0f ? 1 : 0;
  }

  firmware().mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  state.voltage = firmware().vol_adc.read_voltage();

  firmware().mux.select(servo2040::CURRENT_SENSE_ADDR);
  state.current = firmware().cur_adc.read_current();

  firmware().serial.send_packet(seq, ACK, protocol::encode_full_hardware_state(state));
}
