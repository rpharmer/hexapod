#ifndef PROTOCOL_IDS_H
#define PROTOCOL_IDS_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "protocol_layout.hpp"

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
  KILL = 0x18,
  GET_LED_INFO = 0x19,
  SET_LED_COLORS = 0x1A
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

enum class CapabilityFlag : uint8_t {
  ANGULAR_FEEDBACK = (1u << 0)
};

constexpr uint8_t as_u8(CommandCode code) { return static_cast<uint8_t>(code); }
constexpr uint8_t as_u8(StatusCode code) { return static_cast<uint8_t>(code); }
constexpr uint8_t as_u8(ErrorCode code) { return static_cast<uint8_t>(code); }
constexpr uint8_t as_u8(CapabilityFlag capability) { return static_cast<uint8_t>(capability); }

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
inline constexpr uint8_t GET_LED_INFO = as_u8(CommandCode::GET_LED_INFO);
inline constexpr uint8_t SET_LED_COLORS = as_u8(CommandCode::SET_LED_COLORS);

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
inline constexpr uint8_t CAPABILITY_ANGULAR_FEEDBACK = as_u8(CapabilityFlag::ANGULAR_FEEDBACK);

struct CommandMetadata {
  uint8_t id{0};
  const char* canonical_name{"UNKNOWN"};
  std::size_t payload_bytes{0};
  bool exact_payload_size{false};
  bool handled_by_firmware_dispatch{false};
};

inline constexpr std::array<CommandCode, 17> kAllCommandCodes{{
    CommandCode::HELLO,
    CommandCode::HEARTBEAT,
    CommandCode::GET_FULL_HARDWARE_STATE,
    CommandCode::SET_JOINT_TARGETS,
    CommandCode::SET_TARGET_ANGLE,
    CommandCode::SET_POWER_RELAY,
    CommandCode::SET_ANGLE_CALIBRATIONS,
    CommandCode::GET_ANGLE_CALIBRATIONS,
    CommandCode::GET_CURRENT,
    CommandCode::GET_VOLTAGE,
    CommandCode::GET_SENSOR,
    CommandCode::DIAGNOSTIC,
    CommandCode::SET_SERVOS_ENABLED,
    CommandCode::GET_SERVOS_ENABLED,
    CommandCode::SET_SERVOS_TO_MID,
    CommandCode::GET_LED_INFO,
    CommandCode::SET_LED_COLORS,
}};

inline constexpr std::array<CommandMetadata, 17> kCommandMetadata{{
    {as_u8(CommandCode::HELLO), "HELLO", sizeof(uint8_t) * 2, true, false},
    {as_u8(CommandCode::HEARTBEAT), "HEARTBEAT", 0, true, false},
    {as_u8(CommandCode::GET_FULL_HARDWARE_STATE), "GET_FULL_HARDWARE_STATE", 0, true, true},
    {as_u8(CommandCode::SET_JOINT_TARGETS), "SET_JOINT_TARGETS", kProtocolJointTargetsPayloadBytes, true, true},
    {as_u8(CommandCode::SET_TARGET_ANGLE), "SET_TARGET_ANGLE", sizeof(uint8_t) + sizeof(float), true, true},
    {as_u8(CommandCode::SET_POWER_RELAY), "SET_POWER_RELAY", sizeof(uint8_t), true, true},
    {as_u8(CommandCode::SET_ANGLE_CALIBRATIONS), "SET_ANGLE_CALIBRATIONS", kProtocolCalibrationsPayloadBytes, true, true},
    {as_u8(CommandCode::GET_ANGLE_CALIBRATIONS), "GET_ANGLE_CALIBRATIONS", 0, true, true},
    {as_u8(CommandCode::GET_CURRENT), "GET_CURRENT", 0, true, true},
    {as_u8(CommandCode::GET_VOLTAGE), "GET_VOLTAGE", 0, true, true},
    {as_u8(CommandCode::GET_SENSOR), "GET_SENSOR", sizeof(uint8_t), true, true},
    {as_u8(CommandCode::DIAGNOSTIC), "DIAGNOSTIC", 0, false, false},
    {as_u8(CommandCode::SET_SERVOS_ENABLED), "SET_SERVOS_ENABLED", kProtocolServoEnablePayloadBytes, true, true},
    {as_u8(CommandCode::GET_SERVOS_ENABLED), "GET_SERVOS_ENABLED", 0, true, true},
    {as_u8(CommandCode::SET_SERVOS_TO_MID), "SET_SERVOS_TO_MID", 0, true, true},
    {as_u8(CommandCode::GET_LED_INFO), "GET_LED_INFO", 0, true, true},
    {as_u8(CommandCode::SET_LED_COLORS), "SET_LED_COLORS", kProtocolLedColorsPayloadBytes, true, true},
}};

inline constexpr const CommandMetadata* find_command_metadata(uint8_t cmd) {
  for(const CommandMetadata& metadata : kCommandMetadata) {
    if(metadata.id == cmd) {
      return &metadata;
    }
  }
  return nullptr;
}

inline constexpr const CommandMetadata* find_command_metadata(CommandCode cmd) {
  return find_command_metadata(as_u8(cmd));
}

inline constexpr bool try_parse_command_code(uint8_t raw_cmd, CommandCode& out_cmd) {
  const CommandMetadata* metadata = find_command_metadata(raw_cmd);
  if(metadata == nullptr) {
    return false;
  }
  out_cmd = static_cast<CommandCode>(raw_cmd);
  return true;
}

inline constexpr const char* command_name(uint8_t cmd) {
  const CommandMetadata* metadata = find_command_metadata(cmd);
  return (metadata != nullptr) ? metadata->canonical_name : "UNKNOWN";
}

inline constexpr const char* command_name(CommandCode cmd) {
  return command_name(as_u8(cmd));
}

inline constexpr bool command_metadata_is_complete() {
  if(kAllCommandCodes.size() != kCommandMetadata.size()) {
    return false;
  }

  for(CommandCode cmd : kAllCommandCodes) {
    if(find_command_metadata(cmd) == nullptr) {
      return false;
    }
  }
  return true;
}

static_assert(command_metadata_is_complete(),
              "kCommandMetadata and CommandCode enum must stay aligned.");

#endif
