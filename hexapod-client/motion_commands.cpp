#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"

#include <cstring>

void handleSetAngleCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != (sizeof(uint8_t) + sizeof(float)))
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  size_t offset = 0;
  uint8_t servo;
  read_scalar(payload, offset, servo);
  
  if(servo >= FirmwareContext::NUM_SERVOS)
  {
    firmware().serial.send_packet(seq, NACK, {OUT_OF_RANGE_INDEX});
    return;
  }

  float angle;
  read_scalar(payload, offset, angle);
  
  firmware().servos.value(servo, angle);
  firmware().serial.send_packet(seq, ACK, {});
}

void handleCalibCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolCalibrationsPayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::Calibrations calibrations{};
  if (!protocol::decode_calibrations(payload, calibrations))
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint];
  std::size_t calib_idx = 0;
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    calibs[s][0] = calibrations[calib_idx++];
    calibs[s][1] = calibrations[calib_idx++];
  }

  calibServos(calibs);
  firmware().serial.send_packet(seq, ACK, {});
}

void calibServos(float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint])
{
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    Calibration& cal = firmware().servos.calibration(static_cast<int>(s));
    cal.apply_two_pairs(calibs[s][0], calibs[s][1], -45.0f, 45.0f);
  }
  return;
}

void handleSetJointTargetsCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  constexpr size_t expectedPayloadBytes = kProtocolJointTargetsPayloadBytes;
  if(payload.size() != expectedPayloadBytes)
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  protocol::JointTargets target_positions{};
  if (!protocol::decode_joint_targets(payload, target_positions))
  {
    firmware().serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return;
  }

  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    firmware().servos.value(static_cast<int>(s), target_positions[s]);
    firmware().jointTargetPositionsRad[s] = target_positions[s];
  }

  firmware().serial.send_packet(seq, ACK, {});
}
