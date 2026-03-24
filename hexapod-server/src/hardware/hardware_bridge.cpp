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

std::shared_ptr<logging::AsyncLogger> resolveLogger(
    const std::shared_ptr<logging::AsyncLogger>& logger) {
    if (logger) {
        return logger;
    }
    return logging::GetDefaultLogger();
}

}  // namespace

SimpleHardwareBridge::SimpleHardwareBridge(std::string device,
                                           int baud_rate,
                                           int timeout_ms,
                                           std::vector<float> calibrations,
                                           std::shared_ptr<logging::AsyncLogger> logger)
    : device_(std::move(device)),
      baud_rate_(baud_rate),
      timeout_ms_(timeout_ms),
      calibrations_(std::move(calibrations)),
      logger_(std::move(logger)) {}

SimpleHardwareBridge::SimpleHardwareBridge(std::unique_ptr<IPacketEndpoint> endpoint,
                                           int timeout_ms,
                                           std::vector<float> calibrations,
                                           std::shared_ptr<logging::AsyncLogger> logger)
    : timeout_ms_(timeout_ms),
      calibrations_(std::move(calibrations)),
      packet_endpoint_(std::move(endpoint)),
      logger_(std::move(logger)) {}

SimpleHardwareBridge::~SimpleHardwareBridge() = default;

bool SimpleHardwareBridge::log_command_failure(const char* command_name, const char* reason) const {
    if (auto logger = resolveLogger(logger_)) {
        LOG_ERROR(logger, "command '{}' failed: {}", command_name, reason);
    }
    return false;
}

bool SimpleHardwareBridge::requireReady(const char* command_name, bool require_estimator) const {
    if (!initialized_) {
        return log_command_failure(command_name, "bridge not initialized");
    }
    if (!link_manager_) {
        return log_command_failure(command_name, "link manager unavailable");
    }
    if (require_estimator && !feedback_estimator_) {
        return log_command_failure(command_name, "feedback estimator unavailable");
    }
    return true;
}

bool SimpleHardwareBridge::withCommandApi(const char* command_name,
                                          const std::function<bool(BridgeCommandApi&)>& action,
                                          bool require_estimator) const {
    if (!requireReady(command_name, require_estimator)) {
        return false;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return log_command_failure(command_name, "link unavailable");
    }
    if (!command_api_) {
        return log_command_failure(command_name, "command API unavailable");
    }
    if (!action(*command_api_)) {
        return log_command_failure(command_name, "command execution failed");
    }
    return true;
}

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
    if (!requireReady("read", true)) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return log_command_failure("read", "hardware codec unavailable");
    }

    const auto decode_state = [codec](const std::vector<uint8_t>& payload, RobotState& decoded) {
        return codec->decode_full_hardware_state(payload, decoded);
    };
    if (!withCommandApi(
            "read",
            [&out, &decode_state](BridgeCommandApi& api) {
                return api.request_decoded(CommandCode::GET_FULL_HARDWARE_STATE, {}, decode_state, out);
            },
            true)) {
        return false;
    }

    feedback_estimator_->on_hardware_read(out);
    feedback_estimator_->synthesize(out);
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in) {
    if (!requireReady("write", true)) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return log_command_failure("write", "hardware codec unavailable");
    }

    if (!withCommandApi(
            "write",
            [&codec, &in](BridgeCommandApi& api) {
                return api.request_ack(CommandCode::SET_JOINT_TARGETS, codec->encode_joint_targets(in));
            },
            true)) {
        return false;
    }

    feedback_estimator_->on_write(in);
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
    return withCommandApi(
        "set_target_angle",
        [&payload](BridgeCommandApi& api) { return api.request_ack(CommandCode::SET_TARGET_ANGLE, payload); });
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return withCommandApi("set_power_relay",
                          [enabled](BridgeCommandApi& api) {
                              return api.request_ack(
                                  CommandCode::SET_POWER_RELAY, {static_cast<uint8_t>(enabled ? 1 : 0)});
                          });
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };
    if (!withCommandApi(
            "get_angle_calibrations",
            [&calibrations, &decode_calibrations](BridgeCommandApi& api) {
                return api.request_decoded(
                    CommandCode::GET_ANGLE_CALIBRATIONS, {}, decode_calibrations, calibrations);
            })) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    protocol::ScalarFloat current{};
    if (!withCommandApi("get_current",
                        [&current](BridgeCommandApi& api) {
                            return api.request_decoded(
                                CommandCode::GET_CURRENT, {}, decodeScalarFloatPayload, current);
                        })) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    if (!withCommandApi("get_voltage",
                        [&voltage](BridgeCommandApi& api) {
                            return api.request_decoded(
                                CommandCode::GET_VOLTAGE, {}, decodeScalarFloatPayload, voltage);
                        })) {
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
    if (!withCommandApi("get_sensor",
                        [&request_payload, &sensor_voltage](BridgeCommandApi& api) {
                            return api.request_decoded(CommandCode::GET_SENSOR,
                                                       request_payload,
                                                       decodeScalarFloatPayload,
                                                       sensor_voltage);
                        })) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    return withCommandApi("send_diagnostic",
                          [&payload, &response_payload](BridgeCommandApi& api) {
                              return api.request_ack_payload(
                                  CommandCode::DIAGNOSTIC, payload, response_payload);
                          });
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return withCommandApi("set_servos_enabled",
                          [&payload](BridgeCommandApi& api) {
                              return api.request_ack(
                                  CommandCode::SET_SERVOS_ENABLED, protocol::encode_servo_enabled(payload));
                          });
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };
    if (!withCommandApi(
            "get_servos_enabled",
            [&states, &decode_servo_state](BridgeCommandApi& api) {
                return api.request_decoded(CommandCode::GET_SERVOS_ENABLED, {}, decode_servo_state, states);
            })) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return withCommandApi(
        "set_servos_to_mid", [](BridgeCommandApi& api) { return api.request_ack(CommandCode::SET_SERVOS_TO_MID, {}); });
}

bool SimpleHardwareBridge::get_led_info(bool& present, uint8_t& count) {
    protocol::LedInfo info{};
    const auto decode_led_info = [](const std::vector<uint8_t>& payload, protocol::LedInfo& decoded) {
        return protocol::decode_led_info(payload, decoded);
    };
    if (!withCommandApi("get_led_info",
                        [&info, &decode_led_info](BridgeCommandApi& api) {
                            return api.request_decoded(CommandCode::GET_LED_INFO, {}, decode_led_info, info);
                        })) {
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

    return withCommandApi("set_led_colors",
                          [&payload](BridgeCommandApi& api) {
                              return api.request_ack(
                                  CommandCode::SET_LED_COLORS, protocol::encode_led_colors(payload));
                          });
}

bool SimpleHardwareBridge::send_calibrations(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        return log_command_failure("set_angle_calibrations", "incorrect calibration size");
    }

    if (!withCommandApi(
            "set_angle_calibrations",
            [&calibs](BridgeCommandApi& api) {
                protocol::Calibrations payload{};
                for (std::size_t i = 0; i < calibs.size(); ++i) {
                    payload[i] = calibs[i];
                }
                return api.request_ack(
                    CommandCode::SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload));
            })) {
        if (auto logger = resolveLogger(logger_)) {
            LOG_ERROR(logger, "calibration packet failed");
        }
        return false;
    }

    if (auto logger = resolveLogger(logger_)) {
        LOG_INFO(logger, "calibration packet accepted");
    }
    return true;
}
