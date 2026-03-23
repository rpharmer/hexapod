#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "payload_decode.hpp"

namespace {

void nackInvalidPayloadLength(FirmwareContext& ctx, uint16_t seq, payload::DecodeStatus)
{
  ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
}

} // namespace

void handleGetAngleCalibCommand(FirmwareContext& ctx, uint16_t seq)
{
  protocol::Calibrations calibrations{};
  std::size_t calib_idx = 0;
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    Calibration& cal = ctx.servos.calibration(static_cast<int>(s));
    calibrations[calib_idx++] = cal.first_pulse();
    calibrations[calib_idx++] = cal.last_pulse();
  }

  ctx.serial.send_packet(seq, ACK, protocol::encode_calibrations(calibrations));
}


void handleGetCurrentCommand(FirmwareContext& ctx, uint16_t seq)
{
  ctx.mux.select(servo2040::CURRENT_SENSE_ADDR);
  const protocol::ScalarFloat current{ctx.cur_adc.read_current()};
  ctx.serial.send_packet(seq, ACK, protocol::encode_scalar_float(current));
}


void handleGetVoltageCommand(FirmwareContext& ctx, uint16_t seq)
{
  ctx.mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  const protocol::ScalarFloat voltage{ctx.vol_adc.read_voltage()};
  ctx.serial.send_packet(seq, ACK, protocol::encode_scalar_float(voltage));
}


void handleGetSensorCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  uint8_t sensor = 0;
  if(!payload::decode_scalar_exact_or_report(payload,
                                             sensor,
                                             [&](payload::DecodeStatus status) {
                                               nackInvalidPayloadLength(ctx, seq, status);
                                             }))
  {
    return;
  }
  if(sensor >= kProtocolFootSensorCount)
  {
    ctx.serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  ctx.mux.select(servo2040::SENSOR_1_ADDR + sensor);
  const protocol::ScalarFloat voltage{ctx.sen_adc.read_voltage()};
  ctx.serial.send_packet(seq, ACK, protocol::encode_scalar_float(voltage));
}


void handleGetFullHardwareStateCommand(FirmwareContext& ctx, uint16_t seq)
{
  protocol::FullHardwareState state{};

  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    state.joint_positions_rad[s] = ctx.softwareAngleFeedbackEstimatorEnabled
      ? ctx.jointTargetPositionsRad[s]
      : ctx.servos.value(static_cast<int>(s));
  }

  for (std::size_t sensor = 0; sensor < kProtocolFootSensorCount; ++sensor)
  {
    ctx.mux.select(servo2040::SENSOR_1_ADDR + static_cast<int>(sensor));
    const float sensorVoltage = ctx.sen_adc.read_voltage();
    state.foot_contacts[sensor] = sensorVoltage > 1.0f ? 1 : 0;
  }

  ctx.mux.select(servo2040::VOLTAGE_SENSE_ADDR);
  state.voltage = ctx.vol_adc.read_voltage();

  ctx.mux.select(servo2040::CURRENT_SENSE_ADDR);
  state.current = ctx.cur_adc.read_current();

  ctx.serial.send_packet(seq, ACK, protocol::encode_full_hardware_state(state));
}
