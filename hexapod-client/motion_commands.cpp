#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"
#include "payload_decode.hpp"

#include <cstring>

void handleSetAngleCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  uint8_t servo = 0;
  float angle = 0.0f;
  if(!payload::decode_two_scalars_exact(payload, servo, angle))
  {
    ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  if(servo >= FirmwareContext::NUM_SERVOS)
  {
    ctx.serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  ctx.servos.value(servo, angle);
  ctx.serial.send_packet(seq, ACK, {});
}


void handleCalibCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  protocol::Calibrations calibrations{};
  const auto status = payload::expect_payload(payload,
                                              kProtocolCalibrationsPayloadBytes,
                                              calibrations,
                                              protocol::decode_calibrations);
  if(status != payload::DecodeStatus::Ok)
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


void calibServos(FirmwareContext& ctx, float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint])
{
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    Calibration& cal = ctx.servos.calibration(static_cast<int>(s));
    cal.apply_two_pairs(calibs[s][0], calibs[s][1], -45.0f, 45.0f);
  }
  return;
}

void handleSetJointTargetsCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  protocol::JointTargets target_positions{};
  const auto status = payload::expect_payload(payload,
                                              kProtocolJointTargetsPayloadBytes,
                                              target_positions,
                                              protocol::decode_joint_targets);
  if(status != payload::DecodeStatus::Ok)
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
