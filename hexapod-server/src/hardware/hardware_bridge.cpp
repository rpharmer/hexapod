#include "hardware_bridge.hpp"

#include <array>
#include <cstdio>
#include <utility>

#include <CppLinuxSerial/SerialPort.hpp>
#include "command_client.hpp"
#include "handshake_client.hpp"
#include "hardware_state_codec.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"
#include "protocol_codec.hpp"
#include "transport_session.hpp"

using namespace mn::CppLinuxSerial;

namespace {

constexpr DurationUs kLinkTimeoutUs{500000};
constexpr DurationUs kHeartbeatIdleIntervalUs{150000};

void logCommandFailure(const char* command_name,
                       TransportSession::OutcomeClass outcome_class,
                       uint8_t nack_code,
                       std::size_t response_payload_size) {
    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name,
                  " failed (outcome=",
                  static_cast<unsigned>(outcome_class),
                  ", nack_code=",
                  static_cast<unsigned>(nack_code),
                  ", payload_size=",
                  static_cast<unsigned>(response_payload_size),
                  ")");
    }
}

bool decodeScalarFloatPayload(const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
    return protocol::decode_scalar_float(payload, decoded);
}

}  // namespace

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
    if (!request_decoded(GET_CURRENT, {}, decodeScalarFloatPayload, current, "GET_CURRENT")) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    if (!request_decoded(GET_VOLTAGE, {}, decodeScalarFloatPayload, voltage, "GET_VOLTAGE")) {
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
    if (!request_decoded(
            GET_SENSOR, request_payload, decodeScalarFloatPayload, sensor_voltage, "GET_SENSOR")) {
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
    return request_transaction(cmd, payload, nullptr, command_name);
}

bool SimpleHardwareBridge::request_ack_payload(uint8_t cmd,
                                               const std::vector<uint8_t>& payload,
                                               std::vector<uint8_t>& out_payload,
                                               const char* command_name) {
    return request_transaction(cmd, payload, &out_payload, command_name);
}

bool SimpleHardwareBridge::request_transaction(uint8_t cmd,
                                               const std::vector<uint8_t>& payload,
                                               std::vector<uint8_t>* out_payload,
                                               const char* command_name) {
    if (!command_client_) {
        return false;
    }

    if (!ensure_link()) {
        return false;
    }

    const auto outcome = command_client_->transact(cmd, payload, out_payload);
    if (outcome.outcome_class == TransportSession::OutcomeClass::Success) {
        return true;
    }

    const std::size_t payload_size = (out_payload != nullptr) ? out_payload->size() : 0;
    logCommandFailure(command_name, outcome.outcome_class, outcome.nack_code, payload_size);
    return false;
}
