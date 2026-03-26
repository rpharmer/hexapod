#include "hardware_bridge.hpp"

#include <utility>
#include "bridge_command_api.hpp"
#include "bridge_link_manager.hpp"
#include "hardware_bridge_internal.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"

using hardware_bridge::detail::BridgeResultMetadataBuilder;
using hardware_bridge::detail::kRequestedCapabilities;

bool SimpleHardwareBridge::run_ack_command(const char* command_name,
                                           CommandCode command_code,
                                           const std::vector<uint8_t>& payload,
                                           bool require_estimator) {
    const BridgeError command_error = withCommandApi(
        command_name,
        command_code,
        [command_code, &payload](BridgeCommandApi& api) {
            return api.request_ack_with_error(command_code, payload);
        },
        require_estimator);
    return complete_command(command_name, command_error, "command execution failed");
}

template <typename DecodedPayload, typename Decoder>
bool SimpleHardwareBridge::run_decoded_command(const char* command_name,
                                               CommandCode command_code,
                                               const std::vector<uint8_t>& request_payload,
                                               Decoder&& decoder,
                                               DecodedPayload& decoded,
                                               bool require_estimator) {
    const BridgeError command_error = withCommandApi(
        command_name,
        command_code,
        [&request_payload, command_code, &decoder, &decoded](BridgeCommandApi& api) {
            return api.request_decoded_with_error(
                command_code, request_payload, std::forward<Decoder>(decoder), decoded);
        },
        require_estimator);
    return complete_command(command_name, command_error, "command execution failed");
}

bool SimpleHardwareBridge::set_angle_calibrations(const std::vector<float>& calibs) {
    return complete_command("set_angle_calibrations",
                            send_calibrations_result(calibs),
                            "calibration command failed");
}

bool SimpleHardwareBridge::set_target_angle(uint8_t servo_id, float angle) {
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint8_t) + sizeof(float));
    append_scalar(payload, servo_id);
    append_scalar(payload, angle);
    return run_ack_command("set_target_angle", CommandCode::SET_TARGET_ANGLE, payload);
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return run_ack_command("set_power_relay",
                           CommandCode::SET_POWER_RELAY,
                           {static_cast<uint8_t>(enabled ? 1 : 0)});
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };

    if (!run_decoded_command("get_angle_calibrations",
                             CommandCode::GET_ANGLE_CALIBRATIONS,
                             {},
                             decode_calibrations,
                             calibrations)) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    protocol::ScalarFloat current{};
    if (!run_decoded_command("get_current",
                             CommandCode::GET_CURRENT,
                             {},
                             hardware_bridge::detail::decode_scalar_float_payload,
                             current)) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    if (!run_decoded_command("get_voltage",
                             CommandCode::GET_VOLTAGE,
                             {},
                             hardware_bridge::detail::decode_scalar_float_payload,
                             voltage)) {
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
    if (!run_decoded_command("get_sensor",
                             CommandCode::GET_SENSOR,
                             request_payload,
                             hardware_bridge::detail::decode_scalar_float_payload,
                             sensor_voltage)) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    const BridgeError command_error = withCommandApi(
        "send_diagnostic",
        CommandCode::DIAGNOSTIC,
        [&payload, &response_payload](BridgeCommandApi& api) {
            return api.request_ack_payload_with_error(CommandCode::DIAGNOSTIC, payload, response_payload);
        });
    return complete_command("send_diagnostic", command_error, "command execution failed");
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return run_ack_command(
        "set_servos_enabled", CommandCode::SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload));
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };

    if (!run_decoded_command(
            "get_servos_enabled", CommandCode::GET_SERVOS_ENABLED, {}, decode_servo_state, states)) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return run_ack_command("set_servos_to_mid", CommandCode::SET_SERVOS_TO_MID, {});
}

bool SimpleHardwareBridge::get_led_info(bool& present, uint8_t& count) {
    protocol::LedInfo info{};
    const auto decode_led_info = [](const std::vector<uint8_t>& payload, protocol::LedInfo& decoded) {
        return protocol::decode_led_info(payload, decoded);
    };

    if (!run_decoded_command("get_led_info", CommandCode::GET_LED_INFO, {}, decode_led_info, info)) {
        return false;
    }

    present = (info.present != 0);
    count = info.count;
    return true;
}

bool SimpleHardwareBridge::set_led_colors(
    const std::array<uint8_t, kProtocolLedColorsPayloadBytes>& colors) {
    protocol::LedColors payload{};
    for (std::size_t i = 0; i < colors.size(); ++i) {
        payload[i] = colors[i];
    }

    return run_ack_command("set_led_colors", CommandCode::SET_LED_COLORS, protocol::encode_led_colors(payload));
}

BridgeError SimpleHardwareBridge::send_calibrations_result(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::ProtocolFailure,
                                                    BridgeFailurePhase::CommandExecution,
                                                    BridgeFailureDomain::CommandProtocol,
                                                    false,
                                                    as_u8(CommandCode::SET_ANGLE_CALIBRATIONS),
                                                    kRequestedCapabilities,
                                                    static_cast<uint8_t>(
                                                        link_manager_ ? link_manager_->negotiated_capabilities() : 0)});
        return BridgeError::ProtocolFailure;
    }

    const BridgeError command_error = withCommandApi(
        "set_angle_calibrations",
        CommandCode::SET_ANGLE_CALIBRATIONS,
        [&calibs](BridgeCommandApi& api) {
            protocol::Calibrations payload{};
            for (std::size_t i = 0; i < calibs.size(); ++i) {
                payload[i] = calibs[i];
            }
            return api.request_ack_with_error(CommandCode::SET_ANGLE_CALIBRATIONS,
                                              protocol::encode_calibrations(payload));
        });

    if (command_error != BridgeError::None) {
        if (auto logger = hardware_bridge::detail::resolve_logger(logger_)) {
            LOG_ERROR(logger, "calibration packet failed");
        }
        return command_error;
    }

    if (auto logger = hardware_bridge::detail::resolve_logger(logger_)) {
        LOG_INFO(logger, "calibration packet accepted");
    }
    return BridgeError::None;
}
