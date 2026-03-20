#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "framing.hpp"

#include <cstring>

void handleSetAngleCommand(uint16_t seq, const std::vector<uint8_t>& payload)
{
  if(payload.size() != (sizeof(uint8_t) + sizeof(float))
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

  size_t offset = 0;
  float calibs[kProtocolJointCount][kProtocolCalibrationPairsPerJoint];
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    read_scalar(payload, offset, calibs[s][0]);
    read_scalar(payload, offset, calibs[s][1]);
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

  size_t offset = 0;
  for (std::size_t s = 0; s < kProtocolJointCount; ++s)
  {
    float targetPosRad = 0.0f;
    read_scalar(payload, offset, targetPosRad);

    Calibration& cal = firmware().servos.calibration(static_cast<int>(s));
    firmware().servos.value(static_cast<int>(s), targetPosRad);
    firmware().jointTargetPositionsRad[s] = targetPosRad;
  }

  firmware().serial.send_packet(seq, ACK, {});
}
