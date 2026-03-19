// Header guard
#ifndef HEXAPOD_COMMON_H
#define HEXAPOD_COMMON_H

#include <stdint.h>
#include <cstddef>
#include "framing.hpp"

inline constexpr uint8_t PROTOCOL_VERSION = 0x04;
inline constexpr std::size_t kProtocolLegCount = 6;
inline constexpr std::size_t kProtocolJointsPerLeg = 3;
inline constexpr std::size_t kProtocolJointCount = kProtocolLegCount * kProtocolJointsPerLeg;
inline constexpr std::size_t kProtocolFootSensorCount = kProtocolLegCount;
inline constexpr std::size_t kProtocolCalibrationPairsPerJoint = 2;

inline constexpr std::size_t kProtocolJointTargetsPayloadBytes =
    kProtocolJointCount * sizeof(float);
inline constexpr std::size_t kProtocolServoEnablePayloadBytes =
    kProtocolJointCount * sizeof(uint8_t);
inline constexpr std::size_t kProtocolCalibrationsPayloadBytes =
    kProtocolJointCount * kProtocolCalibrationPairsPerJoint * sizeof(float);
inline constexpr std::size_t kProtocolFullStatePayloadBytes =
    (kProtocolJointCount * sizeof(float)) + kProtocolFootSensorCount + (2 * sizeof(float));

enum class CommandCode : uint8_t {
  SET_ANGLE_CALIBRATIONS = 0x01,
  SET_TARGET_ANGLE = 0x02,
  SET_POWER_RELAY = 0x03,
  GET_ANGLE_CALIBRATIONS = 0x04,
  GET_CURRENT = 0x05,
  GET_VOLTAGE = 0x06,
  GET_SENSOR = 0x07,
  HEARTBEAT = 0x08,
  DIAGNOSTIC = 0x09,
  HELLO = 0x10,
  ACK = 0x11,
  NACK = 0x12,
  SET_JOINT_TARGETS = 0x13,
  GET_FULL_HARDWARE_STATE = 0x14,
  SET_SERVOS_ENABLED = 0x15,
  GET_SERVOS_ENABLED = 0x16,
  SET_SERVOS_TO_MID = 0x17,
  KILL = 0x18
};

enum class StatusCode : uint8_t {
  OK = 0x00,
  BUSY = 0x01
};

enum class ErrorCode : uint8_t {
  TIMEOUT = 0xA1,
  VERSION_MISMATCH = 0xA2,
  INVALID_ARGUMENT = 0xA3,
  INVALID_PAYLOAD_LENGTH = 0xA4,
  OUT_OF_RANGE_INDEX = 0xA5,
  UNSUPPORTED_COMMAND = 0xA6,
  BUSY_NOT_READY = 0xA7,
  ALREADY_PAIRED = 0xA8
};

constexpr uint8_t as_u8(CommandCode code) { return static_cast<uint8_t>(code); }
constexpr uint8_t as_u8(StatusCode code) { return static_cast<uint8_t>(code); }
constexpr uint8_t as_u8(ErrorCode code) { return static_cast<uint8_t>(code); }

// Backward-compatible aliases for existing call sites.
inline constexpr uint8_t SET_ANGLE_CALIBRATIONS = as_u8(CommandCode::SET_ANGLE_CALIBRATIONS);
inline constexpr uint8_t SET_TARGET_ANGLE = as_u8(CommandCode::SET_TARGET_ANGLE);
inline constexpr uint8_t SET_POWER_RELAY = as_u8(CommandCode::SET_POWER_RELAY);
inline constexpr uint8_t GET_ANGLE_CALIBRATIONS = as_u8(CommandCode::GET_ANGLE_CALIBRATIONS);
inline constexpr uint8_t GET_CURRENT = as_u8(CommandCode::GET_CURRENT);
inline constexpr uint8_t GET_VOLTAGE = as_u8(CommandCode::GET_VOLTAGE);
inline constexpr uint8_t GET_SENSOR = as_u8(CommandCode::GET_SENSOR);
inline constexpr uint8_t HEARTBEAT = as_u8(CommandCode::HEARTBEAT);
inline constexpr uint8_t DIAGNOSTIC = as_u8(CommandCode::DIAGNOSTIC);
inline constexpr uint8_t HELLO = as_u8(CommandCode::HELLO);
inline constexpr uint8_t ACK = as_u8(CommandCode::ACK);
inline constexpr uint8_t NACK = as_u8(CommandCode::NACK);
inline constexpr uint8_t SET_JOINT_TARGETS = as_u8(CommandCode::SET_JOINT_TARGETS);
inline constexpr uint8_t GET_FULL_HARDWARE_STATE = as_u8(CommandCode::GET_FULL_HARDWARE_STATE);
inline constexpr uint8_t SET_SERVOS_ENABLED = as_u8(CommandCode::SET_SERVOS_ENABLED);
inline constexpr uint8_t GET_SERVOS_ENABLED = as_u8(CommandCode::GET_SERVOS_ENABLED);
inline constexpr uint8_t SET_SERVOS_TO_MID = as_u8(CommandCode::SET_SERVOS_TO_MID);
inline constexpr uint8_t KILL = as_u8(CommandCode::KILL);

inline constexpr uint8_t STATUS_OK = as_u8(StatusCode::OK);
inline constexpr uint8_t STATUS_BUSY = as_u8(StatusCode::BUSY);

inline constexpr uint8_t TIMEOUT = as_u8(ErrorCode::TIMEOUT);
inline constexpr uint8_t VERSION_MISMATCH = as_u8(ErrorCode::VERSION_MISMATCH);
inline constexpr uint8_t INVALID_ARGUMENT = as_u8(ErrorCode::INVALID_ARGUMENT);
inline constexpr uint8_t INVALID_PAYLOAD_LENGTH = as_u8(ErrorCode::INVALID_PAYLOAD_LENGTH);
inline constexpr uint8_t OUT_OF_RANGE_INDEX = as_u8(ErrorCode::OUT_OF_RANGE_INDEX);
inline constexpr uint8_t UNSUPPORTED_COMMAND = as_u8(ErrorCode::UNSUPPORTED_COMMAND);
inline constexpr uint8_t BUSY_NOT_READY = as_u8(ErrorCode::BUSY_NOT_READY);
inline constexpr uint8_t ALREADY_PAIRED = as_u8(ErrorCode::ALREADY_PAIRED);

// enum to map joints to ints
// [R/L] is Right/ Left handside from point of view of robot
// 1st number is Leg number 1 being front legs, 2 middle legs and 3 being back legs
// 2nd number is 1 -> Coxa, 2 -> Femur, 3 -> Tibia
// order is [2nd (R>L)][1st (descending)][3rd (ascending)]
// 0 - 17 corresponds to physical pin (1 - 18)
enum class JointID : uint8_t {
  R31 = 0,
  R32 = 1,
  R33 = 2,
  L31 = 3,
  L32 = 4,
  L33 = 5,
  R21 = 6,
  R22 = 7,
  R23 = 8,
  L21 = 9,
  L22 = 10,
  L23 = 11,
  R11 = 12,
  R12 = 13,
  R13 = 14,
  L11 = 15,
  L12 = 16,
  L13 = 17
};

enum class LegID : uint8_t {
  R3 = 0,
  L3 = 1,
  R2 = 2,
  L2 = 3,
  R1 = 4,
  L1 = 5
};

enum class LegJointID : uint8_t {
  Coxa = 0,
  Femur = 1,
  Tibia = 2
};

/* Abstract base class defining which communication functions must be implemented. Server
 and Client use different libaries but derived class will hide this */
class SerialComms
{
public:
    // framed protocol helpers
    virtual void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) = 0;
    virtual bool recv_packet(DecodedPacket& packet) = 0;
    
protected:
    std::vector<uint8_t> rxBuffer;
};

#endif // #ifndef HEXAPOD_COMMON_H
