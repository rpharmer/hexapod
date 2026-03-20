#ifndef PROTOCOL_CODEC_H
#define PROTOCOL_CODEC_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "hexapod-common.hpp"

namespace protocol {

struct HelloRequest {
  uint8_t version{};
  uint8_t capabilities{};
};

struct HelloAck {
  uint8_t version{};
  uint8_t status{};
  uint8_t device_id{};
};

struct ScalarFloat {
  float value{};
};

using JointTargets = std::array<float, kProtocolJointCount>;
using ServoEnabled = std::array<uint8_t, kProtocolJointCount>;
using Calibrations = std::array<float, kProtocolJointCount * kProtocolCalibrationPairsPerJoint>;

struct FullHardwareState {
  JointTargets joint_positions_rad{};
  std::array<uint8_t, kProtocolFootSensorCount> foot_contacts{};
  float voltage{};
  float current{};
};

inline std::vector<uint8_t> encode_hello_request(const HelloRequest& in) {
  std::vector<uint8_t> payload;
  payload.reserve(2);
  append_scalar(payload, in.version);
  append_scalar(payload, in.capabilities);
  return payload;
}

inline bool decode_hello_request(const std::vector<uint8_t>& payload, HelloRequest& out) {
  std::size_t offset = 0;
  if (!read_scalar(payload, offset, out.version) || !read_scalar(payload, offset, out.capabilities)) {
    return false;
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_hello_ack(const HelloAck& in) {
  std::vector<uint8_t> payload;
  payload.reserve(3);
  append_scalar(payload, in.version);
  append_scalar(payload, in.status);
  append_scalar(payload, in.device_id);
  return payload;
}

inline bool decode_hello_ack(const std::vector<uint8_t>& payload, HelloAck& out) {
  std::size_t offset = 0;
  if (!read_scalar(payload, offset, out.version) || !read_scalar(payload, offset, out.status) ||
      !read_scalar(payload, offset, out.device_id)) {
    return false;
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_scalar_float(const ScalarFloat& in) {
  std::vector<uint8_t> payload;
  payload.reserve(sizeof(float));
  append_scalar(payload, in.value);
  return payload;
}

inline bool decode_scalar_float(const std::vector<uint8_t>& payload, ScalarFloat& out) {
  std::size_t offset = 0;
  if (!read_scalar(payload, offset, out.value)) {
    return false;
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_joint_targets(const JointTargets& in) {
  std::vector<uint8_t> payload;
  payload.reserve(kProtocolJointTargetsPayloadBytes);
  for (const float joint : in) {
    append_scalar(payload, joint);
  }
  return payload;
}

inline bool decode_joint_targets(const std::vector<uint8_t>& payload, JointTargets& out) {
  std::size_t offset = 0;
  for (float& joint : out) {
    if (!read_scalar(payload, offset, joint)) {
      return false;
    }
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_servo_enabled(const ServoEnabled& in) {
  std::vector<uint8_t> payload;
  payload.reserve(kProtocolServoEnablePayloadBytes);
  for (uint8_t enabled : in) {
    append_scalar(payload, enabled);
  }
  return payload;
}

inline bool decode_servo_enabled(const std::vector<uint8_t>& payload, ServoEnabled& out) {
  std::size_t offset = 0;
  for (uint8_t& enabled : out) {
    if (!read_scalar(payload, offset, enabled)) {
      return false;
    }
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_calibrations(const Calibrations& in) {
  std::vector<uint8_t> payload;
  payload.reserve(kProtocolCalibrationsPayloadBytes);
  for (float value : in) {
    append_scalar(payload, value);
  }
  return payload;
}

inline bool decode_calibrations(const std::vector<uint8_t>& payload, Calibrations& out) {
  std::size_t offset = 0;
  for (float& value : out) {
    if (!read_scalar(payload, offset, value)) {
      return false;
    }
  }
  return offset == payload.size();
}

inline std::vector<uint8_t> encode_full_hardware_state(const FullHardwareState& in) {
  std::vector<uint8_t> payload;
  payload.reserve(kProtocolFullStatePayloadBytes);

  for (float pos_rad : in.joint_positions_rad) {
    append_scalar(payload, pos_rad);
  }

  for (uint8_t contact : in.foot_contacts) {
    append_scalar(payload, contact);
  }

  append_scalar(payload, in.voltage);
  append_scalar(payload, in.current);
  return payload;
}

inline bool decode_full_hardware_state(const std::vector<uint8_t>& payload, FullHardwareState& out) {
  std::size_t offset = 0;

  for (float& pos_rad : out.joint_positions_rad) {
    if (!read_scalar(payload, offset, pos_rad)) {
      return false;
    }
  }

  for (uint8_t& contact : out.foot_contacts) {
    if (!read_scalar(payload, offset, contact)) {
      return false;
    }
  }

  if (!read_scalar(payload, offset, out.voltage) || !read_scalar(payload, offset, out.current)) {
    return false;
  }

  return offset == payload.size();
}

}  // namespace protocol

#endif
