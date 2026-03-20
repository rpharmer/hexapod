#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"

#include <cstring>

void handleSetAngleCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != (sizeof(uint8_t) + sizeof(float)))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  size_t offset = 0;
  uint8_t servo;
  read_scalar(payload, offset, servo);
  
  if(servo >= FirmwareContext::NUM_SERVOS)
  {
    ctx.serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  float angle;
  read_scalar(payload, offset, angle);
  
  ctx.servos.value(servo, angle);
  ctx.serial.send_packet(seq, ACK, {});
}

void handleSetAngleCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetAngleCommand(firmware(), seq, payload);
}

void handleCalibCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolCalibrationsPayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::Calibrations calibrations{};
  if (!protocol::decode_calibrations(payload, calibrations))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint];
  std::size_t calib_idx = 0;
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    calibs[s][0] = calibrations[calib_idx++];
    calibs[s][1] = calibrations[calib_idx++];
  }

  calibServos(ctx, calibs);
  ctx.serial.send_packet(seq, ACK, {});
}

void handleCalibCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleCalibCommand(firmware(), seq, payload);
}

void calibServos(FirmwareContext& ctx, float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint])
{
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    Calibration& cal = ctx.servos.calibration(static_cast<int>(s));
    cal.apply_two_pairs(calibs[s][0], calibs[s][1], -45.0f, 45.0f);
  }
  return;
}

void calibServos(float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint])
{
  calibServos(firmware(), calibs);
}

void handleSetJointTargetsCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolJointTargetsPayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::JointTargets target_positions{};
  if (!protocol::decode_joint_targets(payload, target_positions))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    ctx.servos.value(static_cast<int>(s), target_positions[s]);
    ctx.jointTargetPositionsRad[s] = target_positions[s];
  }

  ctx.serial.send_packet(seq, ACK, {});
}

void handleSetJointTargetsCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetJointTargetsCommand(firmware(), seq, payload);
}
