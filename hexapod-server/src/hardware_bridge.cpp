#include "hardware_bridge.hpp"

#include <array>
#include <cstdio>
#include <utility>

#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "logger.hpp"

using namespace mn::CppLinuxSerial;

namespace {

constexpr DurationUs kLinkTimeoutUs{500000};
constexpr DurationUs kHeartbeatIdleIntervalUs{150000};

}  // namespace

class TransportSession {
public:
    TransportSession(SerialCommsServer& serial,
                     DurationUs link_timeout,
                     DurationUs heartbeat_interval)
        : serial_(serial), link_timeout_(link_timeout), heartbeat_interval_(heartbeat_interval) {}

    uint16_t next_sequence() {
        return seq_++;
    }

    bool send(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
        serial_.send_packet(seq, cmd, payload);
        mark_transfer();
        return true;
    }

    bool recv(DecodedPacket& packet) {
        if (!serial_.recv_packet(packet)) {
            return false;
        }
        mark_transfer();
        return true;
    }

    bool wait_for_ack(uint16_t seq, std::vector<uint8_t>* ack_payload = nullptr) {
        DecodedPacket response;
        if (!recv(response)) {
            return false;
        }
        return parse_ack_or_nack(response, seq, ack_payload);
    }

    bool parse_ack_or_nack(const DecodedPacket& response,
                           uint16_t expected_seq,
                           std::vector<uint8_t>* ack_payload = nullptr) const {
        if (response.seq != expected_seq) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger,
                          "sequence mismatch (expected ",
                          static_cast<unsigned>(expected_seq),
                          ", got ",
                          static_cast<unsigned>(response.seq),
                          ")");
            }
            return false;
        }

        if (response.cmd == ACK) {
            if (ack_payload) {
                *ack_payload = response.payload;
            }
            return true;
        }

        if (response.cmd == NACK) {
            if (!response.payload.empty()) {
                if (auto logger = logging::GetDefaultLogger()) {
                    LOG_ERROR(logger, "NACK, error: ", static_cast<unsigned>(response.payload[0]));
                }
            } else {
                if (auto logger = logging::GetDefaultLogger()) {
                    LOG_ERROR(logger, "NACK without error code");
                }
            }
            return false;
        }

        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "unexpected response cmd: ", static_cast<unsigned>(response.cmd));
        }
        return false;
    }

    bool has_link_timed_out(TimePointUs now) const {
        if (!last_transfer_us_.isZero() && ((now - last_transfer_us_) > link_timeout_)) {
            return true;
        }
        return false;
    }

    bool heartbeat_due(TimePointUs now) const {
        if (last_transfer_us_.isZero()) {
            return true;
        }
        return (now - last_transfer_us_) > heartbeat_interval_;
    }

    void reset_activity() {
        last_transfer_us_ = now_us();
    }

private:
    void mark_transfer() {
        last_transfer_us_ = now_us();
    }

    SerialCommsServer& serial_;
    DurationUs link_timeout_{};
    DurationUs heartbeat_interval_{};
    uint16_t seq_{0};
    TimePointUs last_transfer_us_{};
};

class HandshakeClient {
public:
    explicit HandshakeClient(TransportSession& transport)
        : transport_(transport) {}

    bool establish_link(uint8_t requested_caps) {
        const uint16_t seq = transport_.next_sequence();
        const protocol::HelloRequest request{PROTOCOL_VERSION, requested_caps};
        transport_.send(seq, HELLO, protocol::encode_hello_request(request));

        DecodedPacket response;
        if (!transport_.recv(response)) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "handshake timeout waiting for response");
            }
            return false;
        }

        if (response.seq != seq) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger,
                          "sequence mismatch (expected ",
                          static_cast<unsigned>(seq),
                          ", got ",
                          static_cast<unsigned>(response.seq),
                          ")");
            }
            return false;
        }

        if (response.cmd != ACK) {
            if (response.cmd == NACK && !response.payload.empty()) {
                if (auto logger = logging::GetDefaultLogger()) {
                    LOG_ERROR(logger, "NACK, Error: ", static_cast<unsigned>(response.payload[0]));
                }
            } else if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "malformed response, recieved: ", static_cast<unsigned>(response.cmd));
            }
            return false;
        }

        protocol::HelloAck ack{};
        if (!protocol::decode_hello_ack(response.payload, ack)) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "malformed ACK response");
            }
            return false;
        }

        if (ack.version != PROTOCOL_VERSION) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "version mismatch");
            }
            return false;
        }

        if (ack.status != STATUS_OK) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "device not ready");
            }
            return false;
        }

        transport_.reset_activity();
        return true;
    }

    bool send_heartbeat() {
        const uint16_t seq = transport_.next_sequence();
        transport_.send(seq, HEARTBEAT, {});

        DecodedPacket response;
        if (!transport_.recv(response)) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "heartbeat timeout waiting for response");
            }
            return false;
        }

        if (response.seq != seq) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger,
                          "heartbeat sequence mismatch (expected ",
                          static_cast<unsigned>(seq),
                          ", got ",
                          static_cast<unsigned>(response.seq),
                          ")");
            }
            return false;
        }

        if (response.cmd != ACK) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "heartbeat malformed response, recieved: ", static_cast<unsigned>(response.cmd));
            }
            return false;
        }

        protocol::HelloAck heartbeat{};
        if (!protocol::decode_hello_ack(response.payload, heartbeat)) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "heartbeat malformed payload");
            }
            return false;
        }

        if (heartbeat.status != STATUS_OK) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "heartbeat returned non-ok status: ", static_cast<unsigned>(heartbeat.status));
            }
            return false;
        }

        return true;
    }

private:
    TransportSession& transport_;
};

class HardwareStateCodec {
public:
    std::vector<uint8_t> encode_joint_targets(const JointTargets& in) const {
        protocol::JointTargets target_positions{};

        std::size_t idx = 0;
        for (const auto& leg : in.leg_raw_states) {
            for (int j = 0; j < kJointsPerLeg; ++j) {
                target_positions[idx++] = static_cast<float>(leg.joint_raw_state[j].pos_rad.value);
            }
        }

        return protocol::encode_joint_targets(target_positions);
    }

    bool decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                    RawHardwareState& out) const {
        protocol::FullHardwareState decoded{};
        if (!protocol::decode_full_hardware_state(payload, decoded)) {
            return false;
        }

        std::size_t idx = 0;
        for (auto& leg : out.leg_states) {
            for (int j = 0; j < kJointsPerLeg; ++j) {
                leg.joint_raw_state[j].pos_rad = AngleRad{decoded.joint_positions_rad[idx++]};
            }
        }

        for (std::size_t i = 0; i < kProtocolFootSensorCount; ++i) {
            out.foot_contacts[i] = (decoded.foot_contacts[i] != 0);
        }

        out.voltage = decoded.voltage;
        out.current = decoded.current;
        out.timestamp_us = now_us();
        return true;
    }
};

class CommandClient {
public:
    explicit CommandClient(TransportSession& transport)
        : transport_(transport) {}

    bool send_command_and_expect_ack(uint8_t cmd, const std::vector<uint8_t>& payload = {}) {
        const uint16_t seq = transport_.next_sequence();
        transport_.send(seq, cmd, payload);
        return transport_.wait_for_ack(seq);
    }

    bool send_command_and_expect_ack_payload(uint8_t cmd,
                                             const std::vector<uint8_t>& payload,
                                             std::vector<uint8_t>& ack_payload) {
        const uint16_t seq = transport_.next_sequence();
        transport_.send(seq, cmd, payload);
        return transport_.wait_for_ack(seq, &ack_payload);
    }

private:
    TransportSession& transport_;
};

SimpleHardwareBridge::SimpleHardwareBridge(std::string device,
                                           int baud_rate,
                                           int timeout_ms,
                                           std::vector<float> calibrations)
    : device_(std::move(device)),
      baud_rate_(baud_rate),
      timeout_ms_(timeout_ms),
      calibrations_(std::move(calibrations)) {}

SimpleHardwareBridge::~SimpleHardwareBridge() = default;

bool SimpleHardwareBridge::init() {
    serialComs_ = std::make_unique<SerialCommsServer>(device_,
                                                      SerialCommsServer::int_to_baud_rate(baud_rate_),
                                                      NumDataBits::EIGHT,
                                                      Parity::NONE,
                                                      NumStopBits::ONE);
    serialComs_->SetTimeout(timeout_ms_);
    serialComs_->Open();

    transport_ = std::make_unique<TransportSession>(*serialComs_, kLinkTimeoutUs, kHeartbeatIdleIntervalUs);
    handshake_ = std::make_unique<HandshakeClient>(*transport_);
    codec_ = std::make_unique<HardwareStateCodec>();
    command_client_ = std::make_unique<CommandClient>(*transport_);

    if (!handshake_->establish_link(0)) {
        return false;
    }

    if (!handshake_->send_heartbeat()) {
        return false;
    }

    if (!calibrations_.empty() && !send_calibrations(calibrations_)) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool SimpleHardwareBridge::ensure_link() {
    if (!initialized_ || !transport_ || !handshake_) {
        return false;
    }

    const TimePointUs now = now_us();
    if (transport_->has_link_timed_out(now)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_WARN(logger, "link timed out; attempting re-establish");
        }
        return handshake_->establish_link(0);
    }

    if (transport_->heartbeat_due(now)) {
        return handshake_->send_heartbeat();
    }

    return true;
}

bool SimpleHardwareBridge::read(RawHardwareState& out) {
    if (!initialized_ || !serialComs_ || !command_client_ || !codec_) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "either hardware bridge or serial coms not initialised");
        }
        return false;
    }

    if (!ensure_link()) {
        return false;
    }

    std::vector<uint8_t> payload;
    if (!command_client_->send_command_and_expect_ack_payload(GET_FULL_HARDWARE_STATE, {}, payload)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "GET_FULL_HARDWARE_STATE not acknowledged");
        }
        return false;
    }

    if (!codec_->decode_full_hardware_state(payload, out)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "error decoding hardware state response");
        }
        return false;
    }

    state_ = out;
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in) {
    if (!initialized_ || !serialComs_ || !command_client_ || !codec_) {
        return false;
    }

    if (!ensure_link()) {
        return false;
    }

    if (!command_client_->send_command_and_expect_ack(SET_JOINT_TARGETS, codec_->encode_joint_targets(in))) {
        return false;
    }

    last_written_ = in;
    return true;
}

bool SimpleHardwareBridge::set_angle_calibrations(const std::vector<float>& calibs) {
    return send_calibrations(calibs);
}

bool SimpleHardwareBridge::set_target_angle(uint8_t servo_id, float angle) {
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint8_t) + sizeof(float));
    append_scalar(payload, servo_id);
    append_scalar(payload, angle);
    return command_client_ && ensure_link() && command_client_->send_command_and_expect_ack(SET_TARGET_ANGLE, payload);
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return command_client_ && ensure_link() && command_client_->send_command_and_expect_ack(SET_POWER_RELAY, {static_cast<uint8_t>(enabled ? 1 : 0)});
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    std::vector<uint8_t> payload;
    if (!command_client_ || !ensure_link() || !command_client_->send_command_and_expect_ack_payload(GET_ANGLE_CALIBRATIONS, {}, payload)) {
        return false;
    }

    protocol::Calibrations calibrations{};
    if (!protocol::decode_calibrations(payload, calibrations)) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    std::vector<uint8_t> payload;
    if (!command_client_ || !ensure_link() || !command_client_->send_command_and_expect_ack_payload(GET_CURRENT, {}, payload)) {
        return false;
    }

    protocol::ScalarFloat current{};
    if (!protocol::decode_scalar_float(payload, current)) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    std::vector<uint8_t> payload;
    if (!command_client_ || !ensure_link() || !command_client_->send_command_and_expect_ack_payload(GET_VOLTAGE, {}, payload)) {
        return false;
    }

    protocol::ScalarFloat voltage{};
    if (!protocol::decode_scalar_float(payload, voltage)) {
        return false;
    }

    out_voltage = voltage.value;
    return true;
}

bool SimpleHardwareBridge::get_sensor(uint8_t sensor_id, float& out_voltage) {
    std::vector<uint8_t> request_payload;
    request_payload.reserve(sizeof(uint8_t));
    append_scalar(request_payload, sensor_id);

    std::vector<uint8_t> response_payload;
    if (!command_client_ || !ensure_link() || !command_client_->send_command_and_expect_ack_payload(GET_SENSOR, request_payload, response_payload)) {
        return false;
    }

    protocol::ScalarFloat sensor_voltage{};
    if (!protocol::decode_scalar_float(response_payload, sensor_voltage)) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    return command_client_ && ensure_link() && command_client_->send_command_and_expect_ack_payload(DIAGNOSTIC, payload, response_payload);
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return command_client_ && ensure_link() && command_client_->send_command_and_expect_ack(SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload));
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    std::vector<uint8_t> payload;
    if (!command_client_ || !ensure_link() || !command_client_->send_command_and_expect_ack_payload(GET_SERVOS_ENABLED, {}, payload)) {
        return false;
    }

    protocol::ServoEnabled states{};
    if (!protocol::decode_servo_enabled(payload, states)) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return command_client_ && ensure_link() && command_client_->send_command_and_expect_ack(SET_SERVOS_TO_MID, {});
}

bool SimpleHardwareBridge::send_calibrations(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "incorrect calibration size");
        }
        return false;
    }

    protocol::Calibrations payload{};
    for (std::size_t i = 0; i < calibs.size(); ++i) {
        payload[i] = calibs[i];
    }

    if (!command_client_ || !ensure_link()) {
        return false;
    }

    if (!command_client_->send_command_and_expect_ack(SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload))) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "calibration packet failed");
        }
        return false;
    }

    if (auto logger = logging::GetDefaultLogger()) {
        LOG_INFO(logger, "calibration packet accepted");
    }
    return true;
}
