#include "hardware_bridge.hpp"

#include <array>
#include <cstdio>

#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "logger.hpp"

using namespace mn::CppLinuxSerial;

SimpleHardwareBridge::SimpleHardwareBridge(std::string device, int baud_rate, int timeout_ms, std::vector<float> calibrations)
    : device_(std::move(device)), baud_rate_(baud_rate), timeout_ms_(timeout_ms), calibrations_(std::move(calibrations))
{}

bool SimpleHardwareBridge::init()
{
    serialComs_ = std::make_unique<SerialCommsServer>(device_, SerialCommsServer::int_to_baud_rate(baud_rate_), NumDataBits::EIGHT,
                                                      Parity::NONE, NumStopBits::ONE);
    serialComs_->SetTimeout(timeout_ms_);
    serialComs_->Open();

    if (!do_handshake(0))
    {
        return false;
    }

    if (!send_heartbeat())
    {
        return false;
    }

    if (!calibrations_.empty() && !send_calibrations(calibrations_))
    {
        return false;
    }

    initialized_ = true;
    return true;
}

bool SimpleHardwareBridge::read(RawHardwareState& out)
{
    if (!initialized_ || !serialComs_)
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "either hardware bridge or serial coms not initialised"); }
      return false;
    }

    if (!send_heartbeat())
    {
        return false;
    }

    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, GET_FULL_HARDWARE_STATE, {});

    DecodedPacket response;
    if (!serialComs_->recv_packet(response))
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "timeout waiting for response to GET_FULL_HARDWARE_STATE"); }
      return false;
    }
    if (response.seq != seq)
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "incorrect seq recieved for GET_FULL_HARDWARE_STATE"); }
      return false;
    }
    if (response.cmd != ACK)
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "GET_FULL_HARDWARE_STATE not acknowledged"); }
      return false;
    }

    if (!decode_full_hardware_state(response.payload, out))
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "error decoding hardware state response"); }
        return false;
    }

    state_ = out;
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in)
{
    if (!initialized_ || !serialComs_)
    {
        return false;
    }

    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, SET_JOINT_TARGETS, encode_joint_targets(in));

    if (!wait_for_ack(seq))
    {
        return false;
    }

    last_written_ = in;
    return true;
}

bool SimpleHardwareBridge::set_angle_calibrations(const std::vector<float>& calibs)
{
    return send_calibrations(calibs);
}

bool SimpleHardwareBridge::set_target_angle(uint8_t servo_id, float angle)
{
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint8_t) + sizeof(float));
    append_scalar(payload, servo_id);
    append_scalar(payload, angle);
    return send_command_and_expect_ack(SET_TARGET_ANGLE, payload);
}

bool SimpleHardwareBridge::set_power_relay(bool enabled)
{
    return send_command_and_expect_ack(SET_POWER_RELAY, {static_cast<uint8_t>(enabled ? 1 : 0)});
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs)
{
    std::vector<uint8_t> payload;
    if (!send_command_and_expect_ack_payload(GET_ANGLE_CALIBRATIONS, {}, payload))
    {
        return false;
    }

    protocol::Calibrations calibrations{};
    if (!protocol::decode_calibrations(payload, calibrations))
    {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current)
{
    std::vector<uint8_t> payload;
    if (!send_command_and_expect_ack_payload(GET_CURRENT, {}, payload))
    {
        return false;
    }

    protocol::ScalarFloat current{};
    if (!protocol::decode_scalar_float(payload, current))
    {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage)
{
    std::vector<uint8_t> payload;
    if (!send_command_and_expect_ack_payload(GET_VOLTAGE, {}, payload))
    {
        return false;
    }

    protocol::ScalarFloat voltage{};
    if (!protocol::decode_scalar_float(payload, voltage))
    {
        return false;
    }

    out_voltage = voltage.value;
    return true;
}

bool SimpleHardwareBridge::get_sensor(uint8_t sensor_id, float& out_voltage)
{
    std::vector<uint8_t> request_payload;
    request_payload.reserve(sizeof(uint8_t));
    append_scalar(request_payload, sensor_id);

    std::vector<uint8_t> response_payload;
    if (!send_command_and_expect_ack_payload(GET_SENSOR, request_payload, response_payload))
    {
        return false;
    }

    protocol::ScalarFloat sensor_voltage{};
    if (!protocol::decode_scalar_float(response_payload, sensor_voltage))
    {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload)
{
    return send_command_and_expect_ack_payload(DIAGNOSTIC, payload, response_payload);
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled)
{
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i)
    {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return send_command_and_expect_ack(SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload));
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled)
{
    std::vector<uint8_t> payload;
    if (!send_command_and_expect_ack_payload(GET_SERVOS_ENABLED, {}, payload))
    {
        return false;
    }

    protocol::ServoEnabled states{};
    if (!protocol::decode_servo_enabled(payload, states))
    {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i)
    {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid()
{
    return send_command_and_expect_ack(SET_SERVOS_TO_MID, {});
}

uint16_t SimpleHardwareBridge::next_sequence()
{
    return seq_++;
}

bool SimpleHardwareBridge::wait_for_ack(uint16_t seq) const
{
    DecodedPacket response;
    return serialComs_->recv_packet(response) && parse_ack_or_nack(response, seq);
}

bool SimpleHardwareBridge::parse_ack_or_nack(const DecodedPacket& response,
                                             uint16_t expected_seq,
                                             std::vector<uint8_t>* ack_payload) const
{
    if (response.seq != expected_seq)
    {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "sequence mismatch (expected ",
                      static_cast<unsigned>(expected_seq),
                      ", got ",
                      static_cast<unsigned>(response.seq),
                      ")");
        }
        return false;
    }

    if (response.cmd == ACK)
    {
        if (ack_payload)
        {
            *ack_payload = response.payload;
        }
        return true;
    }

    if (response.cmd == NACK)
    {
        if (!response.payload.empty())
        {
            if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "NACK, error: ", static_cast<unsigned>(response.payload[0])); }
        }
        else
        {
            if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "NACK without error code"); }
        }
        return false;
    }

    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "unexpected response cmd: ", static_cast<unsigned>(response.cmd)); }
    return false;
}

bool SimpleHardwareBridge::send_command_and_expect_ack(uint8_t cmd,
                                                       const std::vector<uint8_t>& payload)
{
    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, cmd, payload);
    return wait_for_ack(seq);
}

bool SimpleHardwareBridge::send_command_and_expect_ack_payload(uint8_t cmd,
                                                               const std::vector<uint8_t>& payload,
                                                               std::vector<uint8_t>& ack_payload)
{
    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, cmd, payload);

    DecodedPacket response;
    if (!serialComs_->recv_packet(response))
    {
        if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "timeout waiting for response to cmd: ", static_cast<unsigned>(cmd)); }
        return false;
    }

    return parse_ack_or_nack(response, seq, &ack_payload);
}

std::vector<uint8_t> SimpleHardwareBridge::encode_joint_targets(const JointTargets& in) const
{
    protocol::JointTargets target_positions{};

    std::size_t idx = 0;
    for (const auto& leg : in.leg_raw_states)
    {
        for (int j = 0; j < kJointsPerLeg; ++j)
        {
            target_positions[idx++] = static_cast<float>(leg.joint_raw_state[j].pos_rad.value);
        }
    }

    return protocol::encode_joint_targets(target_positions);
}

bool SimpleHardwareBridge::decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                                      RawHardwareState& out) const
{
    protocol::FullHardwareState decoded{};
    if (!protocol::decode_full_hardware_state(payload, decoded))
    {
        return false;
    }

    std::size_t idx = 0;
    for (auto& leg : out.leg_states)
    {
        for (int j = 0; j < kJointsPerLeg; ++j)
        {
            leg.joint_raw_state[j].pos_rad = AngleRad{decoded.joint_positions_rad[idx++]};
        }
    }

    for (std::size_t i = 0; i < kProtocolFootSensorCount; ++i)
    {
        out.foot_contacts[i] = (decoded.foot_contacts[i] != 0);
    }

    out.voltage = decoded.voltage;
    out.current = decoded.current;
    out.timestamp_us = now_us();
    return true;
}

bool SimpleHardwareBridge::do_handshake(const uint8_t requested_caps)
{
  const uint16_t seq = next_sequence();
  const protocol::HelloRequest request{PROTOCOL_VERSION, requested_caps};
  serialComs_->send_packet(seq, HELLO, protocol::encode_hello_request(request));

  DecodedPacket response;
  if(!serialComs_->recv_packet(response))
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "handshake timeout waiting for response"); }
    return false;
  }

  if(response.seq != seq)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "sequence mismatch (expected ", static_cast<unsigned>(seq), ", got ", static_cast<unsigned>(response.seq), ")"); }
    return false;
  }

  if(response.cmd == NACK)
  {
    if(response.payload.empty())
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "NACK without error code"); }
      return false;
    }
    uint8_t errorCode = response.payload[0];
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "NACK, Error: ", errorCode); }
    return false;
  }
  
  else if(response.cmd == ACK)
  {
    protocol::HelloAck ack{};
    if(!protocol::decode_hello_ack(response.payload, ack))
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "malformed ACK response"); }
      return false;
    }

    if(ack.version != PROTOCOL_VERSION)
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "version mismatch"); }
      return false;
    }
    if(ack.status != STATUS_OK)
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "device not ready"); }
      return false;
    }
    return true;
  }
  else
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "malformed response, recieved: ", response.cmd); }
    return false;
  }
  
  return true;
}

bool SimpleHardwareBridge::send_heartbeat()
{
  const uint16_t seq = next_sequence();
  serialComs_->send_packet(seq, HEARTBEAT, {});

  DecodedPacket response;
  if(!serialComs_->recv_packet(response))
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "heartbeat timeout waiting for response"); }
    return false;
  }

  if(response.seq != seq)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "heartbeat sequence mismatch (expected ", static_cast<unsigned>(seq), ", got ", static_cast<unsigned>(response.seq), ")"); }
    return false;
  }

  if(response.cmd != ACK)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "heartbeat malformed response, recieved: ", response.cmd); }
    return false;
  }

  protocol::HelloAck heartbeat{};
  if (!protocol::decode_hello_ack(response.payload, heartbeat))
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "heartbeat malformed payload"); }
    return false;
  }

  if(heartbeat.status != STATUS_OK)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "heartbeat returned non-ok status: ", heartbeat.status); }
    return false;
  }

  return true;
}

bool SimpleHardwareBridge::send_calibrations(const std::vector<float>& calibs)
{
  if(calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint))
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "incorrect calibration size"); }
    return false;
  }
  
  protocol::Calibrations payload{};
  for (std::size_t i = 0; i < calibs.size(); ++i)
  {
    payload[i] = calibs[i];
  }
  
  const uint16_t seq = next_sequence();
  serialComs_->send_packet(seq, SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload));

  DecodedPacket response;
  if(!serialComs_->recv_packet(response))
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "calibration packet failed: timeout waiting for response"); }
    return false;
  }
  else if(response.seq != seq)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "calibration packet failed: sequence mismatch (expected ", static_cast<unsigned>(seq), ", got ", static_cast<unsigned>(response.seq), ")"); }
    return false;
  }
  else if(response.cmd == ACK)
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_INFO(logger, "calibration packet accepted"); }
    return true;
  }
  else
  {
    if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "calibration packet failed"); }
  }

  return false;
}
