#include "hardware_bridge.hpp"

#include <array>
#include <utility>

#include "bridge_command_api.hpp"
#include "bridge_link_manager.hpp"
#include "command_client.hpp"
#include "geometry_config.hpp"
#include "hardware_state_codec.hpp"
#include "hexapod-common.hpp"
#include "joint_feedback_estimator.hpp"
#include "logger.hpp"
#include "protocol_codec.hpp"

namespace {

constexpr uint8_t kRequestedCapabilities = CAPABILITY_ANGULAR_FEEDBACK;

bool decodeScalarFloatPayload(const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
    return protocol::decode_scalar_float(payload, decoded);
}

}  // namespace

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
    link_manager_ = std::make_unique<BridgeLinkManager>(
        device_, baud_rate_, timeout_ms_, std::move(packet_endpoint_));

    if (!link_manager_->init(kRequestedCapabilities)) {
        return false;
    }

    CommandClient* command_client = link_manager_->command_client();
    if (command_client == nullptr) {
        return false;
    }

    command_api_ = std::make_unique<BridgeCommandApi>(*command_client);
    feedback_estimator_ = std::make_unique<JointFeedbackEstimator>();

    if (!calibrations_.empty() && !send_calibrations(calibrations_)) {
        return false;
    }

    feedback_estimator_->reset();
    const bool software_feedback_enabled = !link_manager_->has_capability(CAPABILITY_ANGULAR_FEEDBACK);
    feedback_estimator_->set_enabled(software_feedback_enabled);
    if (software_feedback_enabled) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_WARN(logger,
                     "peer did not grant ANGULAR_FEEDBACK at handshake; enabling software joint feedback estimate");
        }
    }

    initialized_ = true;
    return true;
}

bool SimpleHardwareBridge::read(RobotState& out) {
    if (!initialized_ || !link_manager_ || !command_api_ || !feedback_estimator_) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "either hardware bridge or serial coms not initialised");
        }
        return false;
    }

    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return false;
    }

    const auto decode_state = [codec](const std::vector<uint8_t>& payload, RobotState& decoded) {
        return codec->decode_full_hardware_state(payload, decoded);
    };
    if (!command_api_->request_decoded(GET_FULL_HARDWARE_STATE, {}, decode_state, out)) {
        return false;
    }

    feedback_estimator_->on_hardware_read(out);
    feedback_estimator_->synthesize(out);
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in) {
    if (!initialized_ || !link_manager_ || !command_api_ || !feedback_estimator_) {
        return false;
    }

    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return false;
    }

    if (!command_api_->request_ack(SET_JOINT_TARGETS, codec->encode_joint_targets(in))) {
        return false;
    }

    feedback_estimator_->on_write(in);
    return true;
}

bool SimpleHardwareBridge::set_angle_calibrations(const std::vector<float>& calibs) {
    return send_calibrations(calibs);
}

bool SimpleHardwareBridge::set_target_angle(uint8_t servo_id, float angle) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint8_t) + sizeof(float));
    append_scalar(payload, servo_id);
    append_scalar(payload, angle);
    return command_api_->request_ack(SET_TARGET_ANGLE, payload);
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }
    return command_api_->request_ack(SET_POWER_RELAY, {static_cast<uint8_t>(enabled ? 1 : 0)});
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };
    if (!command_api_->request_decoded(GET_ANGLE_CALIBRATIONS,
                                       {},
                                       decode_calibrations,
                                       calibrations)) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::ScalarFloat current{};
    if (!command_api_->request_decoded(GET_CURRENT, {}, decodeScalarFloatPayload, current)) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::ScalarFloat voltage{};
    if (!command_api_->request_decoded(GET_VOLTAGE, {}, decodeScalarFloatPayload, voltage)) {
        return false;
    }

    out_voltage = voltage.value;
    return true;
}

bool SimpleHardwareBridge::get_sensor(uint8_t sensor_id, float& out_voltage) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    std::vector<uint8_t> request_payload;
    request_payload.reserve(sizeof(uint8_t));
    append_scalar(request_payload, sensor_id);

    protocol::ScalarFloat sensor_voltage{};
    if (!command_api_->request_decoded(
            GET_SENSOR, request_payload, decodeScalarFloatPayload, sensor_voltage)) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }
    return command_api_->request_ack_payload(DIAGNOSTIC, payload, response_payload);
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return command_api_->request_ack(SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload));
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };
    if (!command_api_->request_decoded(GET_SERVOS_ENABLED, {}, decode_servo_state, states)) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }
    return command_api_->request_ack(SET_SERVOS_TO_MID, {});
}

bool SimpleHardwareBridge::get_led_info(bool& present, uint8_t& count) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::LedInfo info{};
    const auto decode_led_info = [](const std::vector<uint8_t>& payload, protocol::LedInfo& decoded) {
        return protocol::decode_led_info(payload, decoded);
    };
    if (!command_api_->request_decoded(GET_LED_INFO, {}, decode_led_info, info)) {
        return false;
    }

    present = (info.present != 0);
    count = info.count;
    return true;
}

bool SimpleHardwareBridge::set_led_colors(
    const std::array<uint8_t, kProtocolLedColorsPayloadBytes>& colors) {
    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::LedColors payload{};
    for (std::size_t i = 0; i < colors.size(); ++i) {
        payload[i] = colors[i];
    }

    return command_api_->request_ack(SET_LED_COLORS, protocol::encode_led_colors(payload));
}

bool SimpleHardwareBridge::send_calibrations(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "incorrect calibration size");
        }
        return false;
    }

    if (!initialized_ || !link_manager_ || !command_api_) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return false;
    }

    protocol::Calibrations payload{};
    for (std::size_t i = 0; i < calibs.size(); ++i) {
        payload[i] = calibs[i];
    }

    if (!command_api_->request_ack(SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload))) {
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
