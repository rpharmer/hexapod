#include "hardware_bridge.hpp"

#include <array>
#include <chrono>
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
    enum class OutcomeClass : uint8_t {
        Success,
        Nack,
        Timeout,
        RetryExhausted,
        ProtocolError
    };

    struct CommandOutcome {
        OutcomeClass outcome_class{OutcomeClass::ProtocolError};
        std::vector<uint8_t> ack_payload{};
        uint8_t nack_code{0};
    };

    TransportSession(IPacketEndpoint& endpoint,
                     DurationUs link_timeout,
                     DurationUs heartbeat_interval)
        : endpoint_(endpoint), link_timeout_(link_timeout), heartbeat_interval_(heartbeat_interval) {}

    uint16_t next_sequence() {
        return seq_++;
    }

    bool send(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
        endpoint_.send_packet(seq, cmd, payload);
        mark_transfer();
        return true;
    }

    bool recv(DecodedPacket& packet) {
        if (!endpoint_.recv_packet(packet)) {
            return false;
        }
        mark_transfer();
        return true;
    }

    CommandOutcome wait_for_ack(uint16_t seq) {
        DecodedPacket response;
        if (!recv(response)) {
            return CommandOutcome{OutcomeClass::Timeout, {}, 0};
        }
        return parse_ack_or_nack(response, seq);
    }

    CommandOutcome parse_ack_or_nack(const DecodedPacket& response,
                                     uint16_t expected_seq) const {
        if (response.seq != expected_seq) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger,
                          "sequence mismatch (expected ",
                          static_cast<unsigned>(expected_seq),
                          ", got ",
                          static_cast<unsigned>(response.seq),
                          ")");
            }
            return CommandOutcome{OutcomeClass::ProtocolError, {}, 0};
        }

        if (response.cmd == ACK) {
            return CommandOutcome{OutcomeClass::Success, response.payload, 0};
        }

        if (response.cmd == NACK) {
            const uint8_t nack_code = response.payload.empty() ? 0 : response.payload[0];
            if (!response.payload.empty()) {
                if (auto logger = logging::GetDefaultLogger()) {
                    LOG_ERROR(logger, "NACK, error: ", static_cast<unsigned>(nack_code));
                }
            } else {
                if (auto logger = logging::GetDefaultLogger()) {
                    LOG_ERROR(logger, "NACK without error code");
                }
            }
            return CommandOutcome{OutcomeClass::Nack, {}, nack_code};
        }

        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "unexpected response cmd: ", static_cast<unsigned>(response.cmd));
        }
        return CommandOutcome{OutcomeClass::ProtocolError, {}, 0};
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

    IPacketEndpoint& endpoint_;
    DurationUs link_timeout_{};
    DurationUs heartbeat_interval_{};
    uint16_t seq_{0};
    TimePointUs last_transfer_us_{};
};

class SerialPacketEndpoint final : public IPacketEndpoint {
public:
    explicit SerialPacketEndpoint(SerialCommsServer& serial)
        : serial_(serial) {}

    void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) override {
        serial_.send_packet(seq, cmd, payload);
    }

    bool recv_packet(DecodedPacket& packet) override {
        return serial_.recv_packet(packet);
    }

private:
    SerialCommsServer& serial_;
};

class CommandClient;

class HandshakeClient {
public:
    HandshakeClient(TransportSession& transport, CommandClient& command_client);

    bool establish_link(uint8_t requested_caps);
    bool send_heartbeat();

private:
    TransportSession& transport_;
    CommandClient& command_client_;
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
    struct RetryPolicy {
        int max_attempts{3};
    };

    explicit CommandClient(TransportSession& transport)
        : transport_(transport) {}

    void set_retry_policy(const RetryPolicy& retry_policy) {
        retry_policy_ = retry_policy;
    }

    bool send_command_and_expect_ack(uint8_t cmd, const std::vector<uint8_t>& payload = {}) {
        return transact(cmd, payload, nullptr).outcome_class == TransportSession::OutcomeClass::Success;
    }

    bool send_command_and_expect_ack_payload(uint8_t cmd,
                                             const std::vector<uint8_t>& payload,
                                             std::vector<uint8_t>& ack_payload) {
        return transact(cmd, payload, &ack_payload).outcome_class == TransportSession::OutcomeClass::Success;
    }

    TransportSession::CommandOutcome transact(uint8_t cmd,
                                              const std::vector<uint8_t>& payload,
                                              std::vector<uint8_t>* ack_payload) {
        const int max_attempts = (retry_policy_.max_attempts > 0) ? retry_policy_.max_attempts : 1;
        const auto start = std::chrono::steady_clock::now();
        TransportSession::CommandOutcome last_outcome{TransportSession::OutcomeClass::ProtocolError, {}, 0};
        uint16_t seq = 0;
        int attempts_used = 0;

        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            seq = transport_.next_sequence();
            ++attempts_used;
            transport_.send(seq, cmd, payload);
            last_outcome = transport_.wait_for_ack(seq);
            if (last_outcome.outcome_class == TransportSession::OutcomeClass::Success) {
                break;
            }
            if (last_outcome.outcome_class == TransportSession::OutcomeClass::Nack) {
                break;
            }
        }

        if (last_outcome.outcome_class != TransportSession::OutcomeClass::Success &&
            last_outcome.outcome_class != TransportSession::OutcomeClass::Nack &&
            attempts_used >= max_attempts) {
            last_outcome.outcome_class = TransportSession::OutcomeClass::RetryExhausted;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_INFO(logger,
                     "telemetry command=",
                     command_name(cmd),
                     " seq=",
                     static_cast<unsigned>(seq),
                     " latency_us=",
                     elapsed,
                     " outcome=",
                     outcome_to_text(last_outcome.outcome_class),
                     " attempts=",
                     attempts_used);
            if (last_outcome.outcome_class != TransportSession::OutcomeClass::Success) {
                LOG_WARN(logger,
                         "command_failure command=",
                         command_name(cmd),
                         " domain_error=",
                         domain_error_for_outcome(last_outcome.outcome_class),
                         " nack_code=",
                         static_cast<unsigned>(last_outcome.nack_code));
            }
        }

        if (last_outcome.outcome_class == TransportSession::OutcomeClass::Success && ack_payload) {
            *ack_payload = last_outcome.ack_payload;
        }

        return last_outcome;
    }

private:
    static const char* outcome_to_text(TransportSession::OutcomeClass outcome) {
        switch (outcome) {
            case TransportSession::OutcomeClass::Success:
                return "success";
            case TransportSession::OutcomeClass::Nack:
                return "nack";
            case TransportSession::OutcomeClass::Timeout:
                return "timeout";
            case TransportSession::OutcomeClass::RetryExhausted:
                return "retry_exhausted";
            case TransportSession::OutcomeClass::ProtocolError:
                return "protocol_error";
        }
        return "protocol_error";
    }

    static const char* command_name(uint8_t cmd) {
        switch (cmd) {
            case HELLO: return "HELLO";
            case HEARTBEAT: return "HEARTBEAT";
            case GET_FULL_HARDWARE_STATE: return "GET_FULL_HARDWARE_STATE";
            case SET_JOINT_TARGETS: return "SET_JOINT_TARGETS";
            case SET_TARGET_ANGLE: return "SET_TARGET_ANGLE";
            case SET_POWER_RELAY: return "SET_POWER_RELAY";
            case SET_ANGLE_CALIBRATIONS: return "SET_ANGLE_CALIBRATIONS";
            case GET_ANGLE_CALIBRATIONS: return "GET_ANGLE_CALIBRATIONS";
            case GET_CURRENT: return "GET_CURRENT";
            case GET_VOLTAGE: return "GET_VOLTAGE";
            case GET_SENSOR: return "GET_SENSOR";
            case DIAGNOSTIC: return "DIAGNOSTIC";
            case SET_SERVOS_ENABLED: return "SET_SERVOS_ENABLED";
            case GET_SERVOS_ENABLED: return "GET_SERVOS_ENABLED";
            case SET_SERVOS_TO_MID: return "SET_SERVOS_TO_MID";
            default: return "UNKNOWN";
        }
    }

    static const char* domain_error_for_outcome(TransportSession::OutcomeClass outcome) {
        switch (outcome) {
            case TransportSession::OutcomeClass::Success:
                return "none";
            case TransportSession::OutcomeClass::Nack:
                return "device_rejected";
            case TransportSession::OutcomeClass::Timeout:
                return "response_timeout";
            case TransportSession::OutcomeClass::RetryExhausted:
                return "retry_exhausted";
            case TransportSession::OutcomeClass::ProtocolError:
                return "protocol_violation";
        }
        return "protocol_violation";
    }

    TransportSession& transport_;
    RetryPolicy retry_policy_{};
};

HandshakeClient::HandshakeClient(TransportSession& transport, CommandClient& command_client)
    : transport_(transport), command_client_(command_client) {}

bool HandshakeClient::establish_link(uint8_t requested_caps) {
    const protocol::HelloRequest request{PROTOCOL_VERSION, requested_caps};
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(
            HELLO, protocol::encode_hello_request(request), ack_payload)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "handshake failed");
        }
        return false;
    }

    protocol::HelloAck ack{};
    if (!protocol::decode_hello_ack(ack_payload, ack)) {
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

bool HandshakeClient::send_heartbeat() {
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(HEARTBEAT, {}, ack_payload)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "heartbeat failed");
        }
        return false;
    }

    protocol::HelloAck heartbeat{};
    if (!protocol::decode_hello_ack(ack_payload, heartbeat)) {
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

SimpleHardwareBridge::SimpleHardwareBridge(std::string device,
                                           int baud_rate,
                                           int timeout_ms,
                                           std::vector<float> calibrations)
    : device_(std::move(device)),
      baud_rate_(baud_rate),
      timeout_ms_(timeout_ms),
      calibrations_(std::move(calibrations)) {}

SimpleHardwareBridge::SimpleHardwareBridge(std::unique_ptr<IPacketEndpoint> endpoint,
                                           int timeout_ms,
                                           std::vector<float> calibrations)
    : timeout_ms_(timeout_ms),
      calibrations_(std::move(calibrations)),
      packet_endpoint_(std::move(endpoint)) {}

SimpleHardwareBridge::~SimpleHardwareBridge() = default;

bool SimpleHardwareBridge::init() {
    if (!packet_endpoint_) {
        serialComs_ = std::make_unique<SerialCommsServer>(device_,
                                                          SerialCommsServer::int_to_baud_rate(baud_rate_),
                                                          NumDataBits::EIGHT,
                                                          Parity::NONE,
                                                          NumStopBits::ONE);
        serialComs_->SetTimeout(timeout_ms_);
        serialComs_->Open();
        packet_endpoint_ = std::make_unique<SerialPacketEndpoint>(*serialComs_);
    }

    if (!packet_endpoint_) {
        return false;
    }

    transport_ = std::make_unique<TransportSession>(*packet_endpoint_, kLinkTimeoutUs, kHeartbeatIdleIntervalUs);
    codec_ = std::make_unique<HardwareStateCodec>();
    command_client_ = std::make_unique<CommandClient>(*transport_);
    handshake_ = std::make_unique<HandshakeClient>(*transport_, *command_client_);

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
    if (!initialized_ || !packet_endpoint_ || !codec_) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "either hardware bridge or serial coms not initialised");
        }
        return false;
    }

    const auto decode_state = [this](const std::vector<uint8_t>& payload, RawHardwareState& decoded) {
        return codec_->decode_full_hardware_state(payload, decoded);
    };
    if (!request_decoded(GET_FULL_HARDWARE_STATE, {}, decode_state, out, "GET_FULL_HARDWARE_STATE")) {
        return false;
    }

    state_ = out;
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in) {
    if (!initialized_ || !packet_endpoint_ || !codec_) {
        return false;
    }

    if (!request_ack(SET_JOINT_TARGETS, codec_->encode_joint_targets(in), "SET_JOINT_TARGETS")) {
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
    return request_ack(SET_TARGET_ANGLE, payload, "SET_TARGET_ANGLE");
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return request_ack(SET_POWER_RELAY, {static_cast<uint8_t>(enabled ? 1 : 0)}, "SET_POWER_RELAY");
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };
    if (!request_decoded(GET_ANGLE_CALIBRATIONS,
                         {},
                         decode_calibrations,
                         calibrations,
                         "GET_ANGLE_CALIBRATIONS")) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    protocol::ScalarFloat current{};
    const auto decode_scalar = [](const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
        return protocol::decode_scalar_float(payload, decoded);
    };
    if (!request_decoded(GET_CURRENT, {}, decode_scalar, current, "GET_CURRENT")) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    const auto decode_scalar = [](const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
        return protocol::decode_scalar_float(payload, decoded);
    };
    if (!request_decoded(GET_VOLTAGE, {}, decode_scalar, voltage, "GET_VOLTAGE")) {
        return false;
    }

    out_voltage = voltage.value;
    return true;
}

bool SimpleHardwareBridge::get_sensor(uint8_t sensor_id, float& out_voltage) {
    std::vector<uint8_t> request_payload;
    request_payload.reserve(sizeof(uint8_t));
    append_scalar(request_payload, sensor_id);

    protocol::ScalarFloat sensor_voltage{};
    const auto decode_scalar = [](const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
        return protocol::decode_scalar_float(payload, decoded);
    };
    if (!request_decoded(GET_SENSOR, request_payload, decode_scalar, sensor_voltage, "GET_SENSOR")) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    return request_ack_payload(DIAGNOSTIC, payload, response_payload, "DIAGNOSTIC");
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return request_ack(SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload), "SET_SERVOS_ENABLED");
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };
    if (!request_decoded(GET_SERVOS_ENABLED, {}, decode_servo_state, states, "GET_SERVOS_ENABLED")) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return request_ack(SET_SERVOS_TO_MID, {}, "SET_SERVOS_TO_MID");
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

    if (!request_ack(SET_ANGLE_CALIBRATIONS,
                     protocol::encode_calibrations(payload),
                     "SET_ANGLE_CALIBRATIONS")) {
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

bool SimpleHardwareBridge::request_ack(uint8_t cmd,
                                       const std::vector<uint8_t>& payload,
                                       const char* command_name) {
    if (!command_client_) {
        return false;
    }

    if (!ensure_link()) {
        return false;
    }

    const auto outcome = command_client_->transact(cmd, payload, nullptr);
    if (outcome.outcome_class == TransportSession::OutcomeClass::Success) {
        return true;
    }

    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name,
                  " failed (outcome=",
                  static_cast<unsigned>(outcome.outcome_class),
                  ", nack_code=",
                  static_cast<unsigned>(outcome.nack_code),
                  ")");
    }
    return false;
}

bool SimpleHardwareBridge::request_ack_payload(uint8_t cmd,
                                               const std::vector<uint8_t>& payload,
                                               std::vector<uint8_t>& out_payload,
                                               const char* command_name) {
    if (!command_client_) {
        return false;
    }

    if (!ensure_link()) {
        return false;
    }

    const auto outcome = command_client_->transact(cmd, payload, &out_payload);
    if (outcome.outcome_class == TransportSession::OutcomeClass::Success) {
        return true;
    }

    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name,
                  " failed (outcome=",
                  static_cast<unsigned>(outcome.outcome_class),
                  ", nack_code=",
                  static_cast<unsigned>(outcome.nack_code),
                  ")");
    }
    return false;
}
