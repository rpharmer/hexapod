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

const char* SimpleHardwareBridge::bridge_error_to_text(BridgeError error) {
    switch (error) {
        case BridgeError::None:
            return "none";
        case BridgeError::NotReady:
            return "not_ready";
        case BridgeError::TransportFailure:
            return "transport_failure";
        case BridgeError::ProtocolFailure:
            return "protocol_failure";
        case BridgeError::Timeout:
            return "timeout";
        case BridgeError::Unsupported:
            return "unsupported";
    }
    return "protocol_failure";
}

const char* SimpleHardwareBridge::bridge_phase_to_text(BridgeFailurePhase phase) {
    switch (phase) {
        case BridgeFailurePhase::None: return "none";
        case BridgeFailurePhase::Readiness: return "readiness";
        case BridgeFailurePhase::CapabilityNegotiation: return "capability_negotiation";
        case BridgeFailurePhase::CommandTransport: return "command_transport";
        case BridgeFailurePhase::CommandResponse: return "command_response";
        case BridgeFailurePhase::CommandDecode: return "command_decode";
        case BridgeFailurePhase::CommandExecution: return "command_execution";
        case BridgeFailurePhase::Initialization: return "initialization";
    }
    return "none";
}

const char* SimpleHardwareBridge::bridge_domain_to_text(BridgeFailureDomain domain) {
    switch (domain) {
        case BridgeFailureDomain::None: return "none";
        case BridgeFailureDomain::CapabilityProtocol: return "capability_protocol";
        case BridgeFailureDomain::TransportLink: return "transport_link";
        case BridgeFailureDomain::CommandProtocol: return "command_protocol";
    }
    return "none";
}

void SimpleHardwareBridge::set_last_result(const BridgeCommandResultMetadata& metadata) {
    last_result_ = metadata;
    last_error_ = metadata.error;
}

bool SimpleHardwareBridge::complete_command(const char* command_name,
                                            BridgeError error,
                                            const char* reason) {
    last_result_.error = error;
    last_error_ = error;
    if (error != BridgeError::None) {
        if (auto logger = resolveLogger(logger_)) {
            LOG_ERROR(logger,
                      "command '{}' failed: {} (error={}, phase={}, domain={}, retryable={}, cmd_code={}, req_caps={}, nego_caps={})",
                      command_name,
                      reason != nullptr ? reason : bridge_error_to_text(error),
                      bridge_error_to_text(error),
                      bridge_phase_to_text(last_result_.phase),
                      bridge_domain_to_text(last_result_.domain),
                      (last_result_.retryable ? "yes" : "no"),
                      static_cast<unsigned>(last_result_.command_code),
                      static_cast<unsigned>(last_result_.requested_capabilities),
                      static_cast<unsigned>(last_result_.negotiated_capabilities));
        }
        return false;
    }
    last_result_ = BridgeCommandResultMetadata{};
    return true;
}

BridgeError SimpleHardwareBridge::requireReady(const char* command_name, bool require_estimator) {
    (void)command_name;
    if (!initialized_ || !link_manager_ || (require_estimator && !feedback_estimator_)) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::NotReady,
                                                    BridgeFailurePhase::Readiness,
                                                    BridgeFailureDomain::CommandProtocol,
                                                    false,
                                                    0,
                                                    kRequestedCapabilities,
                                                    static_cast<uint8_t>(link_manager_ ? link_manager_->negotiated_capabilities() : 0)});
        return BridgeError::NotReady;
    }
    return BridgeError::None;
}

BridgeError SimpleHardwareBridge::withCommandApi(
    const char* command_name,
    CommandCode command_code,
    const std::function<BridgeError(BridgeCommandApi&)>& action,
    bool require_estimator) {
    (void)command_name;
    const BridgeError ready_error = requireReady(command_name, require_estimator);
    if (ready_error != BridgeError::None) {
        return ready_error;
    }
    const auto ensure = link_manager_->ensure_link_with_status(kRequestedCapabilities);
    if (!ensure.ok) {
        if (ensure.failure == BridgeLinkManager::EnsureLinkFailure::CapabilityNegotiation) {
            set_last_result(BridgeCommandResultMetadata{BridgeError::ProtocolFailure,
                                                        BridgeFailurePhase::CapabilityNegotiation,
                                                        BridgeFailureDomain::CapabilityProtocol,
                                                        true,
                                                        as_u8(command_code),
                                                        kRequestedCapabilities,
                                                        ensure.negotiated_capabilities});
            return BridgeError::ProtocolFailure;
        }
        set_last_result(BridgeCommandResultMetadata{BridgeError::TransportFailure,
                                                    BridgeFailurePhase::CommandTransport,
                                                    BridgeFailureDomain::TransportLink,
                                                    true,
                                                    as_u8(command_code),
                                                    kRequestedCapabilities,
                                                    ensure.negotiated_capabilities});
        return BridgeError::TransportFailure;
    }
    if (!command_api_) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::NotReady,
                                                    BridgeFailurePhase::Readiness,
                                                    BridgeFailureDomain::CommandProtocol,
                                                    false,
                                                    as_u8(command_code),
                                                    kRequestedCapabilities,
                                                    link_manager_->negotiated_capabilities()});
        return BridgeError::NotReady;
    }
    const BridgeError error = action(*command_api_);
    const auto command_meta = command_api_->last_result_metadata();
    set_last_result(BridgeCommandResultMetadata{error,
                                                command_meta.phase,
                                                command_meta.domain,
                                                command_meta.retryable,
                                                command_meta.command_code,
                                                kRequestedCapabilities,
                                                link_manager_->negotiated_capabilities()});
    return error;
}

BridgeError SimpleHardwareBridge::last_error() const {
    return last_error_;
}

std::optional<BridgeCommandResultMetadata> SimpleHardwareBridge::last_bridge_result() const {
    return last_result_;
}

bool SimpleHardwareBridge::init() {
    set_last_result(BridgeCommandResultMetadata{});
    link_manager_ = std::make_unique<BridgeLinkManager>(
        device_, baud_rate_, timeout_ms_, std::move(packet_endpoint_));

    if (!link_manager_->init(kRequestedCapabilities)) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::TransportFailure,
                                                    BridgeFailurePhase::Initialization,
                                                    BridgeFailureDomain::TransportLink,
                                                    true,
                                                    as_u8(CommandCode::HELLO),
                                                    kRequestedCapabilities,
                                                    0});
        return complete_command("init", BridgeError::TransportFailure, "link manager init failed");
    }

    CommandClient* command_client = link_manager_->command_client();
    if (command_client == nullptr) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::NotReady,
                                                    BridgeFailurePhase::Initialization,
                                                    BridgeFailureDomain::CommandProtocol,
                                                    false,
                                                    0,
                                                    kRequestedCapabilities,
                                                    link_manager_->negotiated_capabilities()});
        return complete_command("init", BridgeError::NotReady, "command client unavailable");
    }

    command_api_ = std::make_unique<BridgeCommandApi>(*command_client);
    feedback_estimator_ = std::make_unique<JointFeedbackEstimator>();

    if (!calibrations_.empty() && !set_angle_calibrations(calibrations_)) {
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
    set_last_result(BridgeCommandResultMetadata{});
    return true;
}

bool SimpleHardwareBridge::read(RobotState& out) {
    const BridgeError ready_error = requireReady("read", true);
    if (!complete_command("read", ready_error, "bridge not ready")) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return complete_command("read", BridgeError::NotReady, "hardware codec unavailable");
    }

    const auto decode_state = [codec](const std::vector<uint8_t>& payload, RobotState& decoded) {
        return codec->decode_full_hardware_state(payload, decoded);
    };
    const BridgeError command_error = withCommandApi(
        "read",
        CommandCode::GET_FULL_HARDWARE_STATE,
        [&out, &decode_state](BridgeCommandApi& api) {
            return api.request_decoded_with_error(CommandCode::GET_FULL_HARDWARE_STATE, {}, decode_state, out);
        },
        true);
    if (!complete_command("read", command_error, "command execution failed")) {
        return false;
    }

    feedback_estimator_->on_hardware_read(out);
    feedback_estimator_->synthesize(out);
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in) {
    const BridgeError ready_error = requireReady("write", true);
    if (!complete_command("write", ready_error, "bridge not ready")) {
        return false;
    }

    HardwareStateCodec* codec = link_manager_->codec();
    if (codec == nullptr) {
        return complete_command("write", BridgeError::NotReady, "hardware codec unavailable");
    }

    const BridgeError command_error = withCommandApi(
        "write",
        CommandCode::SET_JOINT_TARGETS,
        [&codec, &in](BridgeCommandApi& api) {
            return api.request_ack_with_error(CommandCode::SET_JOINT_TARGETS, codec->encode_joint_targets(in));
        },
        true);
    if (!complete_command("write", command_error, "command execution failed")) {
        return false;
    }

    feedback_estimator_->on_write(in);
    return true;
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
    return complete_command("set_target_angle",
                            withCommandApi("set_target_angle",
                                           CommandCode::SET_TARGET_ANGLE,
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_TARGET_ANGLE, payload);
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return complete_command("set_power_relay",
                            withCommandApi("set_power_relay",
                                           CommandCode::SET_POWER_RELAY,
                                           [enabled](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_POWER_RELAY,
                                                   {static_cast<uint8_t>(enabled ? 1 : 0)});
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };
    const BridgeError command_error = withCommandApi(
        "get_angle_calibrations",
        CommandCode::GET_ANGLE_CALIBRATIONS,
        [&calibrations, &decode_calibrations](BridgeCommandApi& api) {
            return api.request_decoded_with_error(
                CommandCode::GET_ANGLE_CALIBRATIONS, {}, decode_calibrations, calibrations);
        });
    if (!complete_command("get_angle_calibrations", command_error, "command execution failed")) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    protocol::ScalarFloat current{};
    const BridgeError command_error = withCommandApi("get_current",
                                                     CommandCode::GET_CURRENT,
                                                     [&current](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_CURRENT,
                                                             {},
                                                             decodeScalarFloatPayload,
                                                             current);
                                                     });
    if (!complete_command("get_current", command_error, "command execution failed")) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    const BridgeError command_error = withCommandApi("get_voltage",
                                                     CommandCode::GET_VOLTAGE,
                                                     [&voltage](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_VOLTAGE,
                                                             {},
                                                             decodeScalarFloatPayload,
                                                             voltage);
                                                     });
    if (!complete_command("get_voltage", command_error, "command execution failed")) {
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
    const BridgeError command_error = withCommandApi("get_sensor",
                                                     CommandCode::GET_SENSOR,
                                                     [&request_payload, &sensor_voltage](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(CommandCode::GET_SENSOR,
                                                                                              request_payload,
                                                                                              decodeScalarFloatPayload,
                                                                                              sensor_voltage);
                                                     });
    if (!complete_command("get_sensor", command_error, "command execution failed")) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    return complete_command("send_diagnostic",
                            withCommandApi("send_diagnostic",
                                           CommandCode::DIAGNOSTIC,
                                           [&payload, &response_payload](BridgeCommandApi& api) {
                                               return api.request_ack_payload_with_error(
                                                   CommandCode::DIAGNOSTIC, payload, response_payload);
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return complete_command("set_servos_enabled",
                            withCommandApi("set_servos_enabled",
                                           CommandCode::SET_SERVOS_ENABLED,
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_SERVOS_ENABLED,
                                                   protocol::encode_servo_enabled(payload));
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };
    const BridgeError command_error = withCommandApi(
        "get_servos_enabled",
        CommandCode::GET_SERVOS_ENABLED,
        [&states, &decode_servo_state](BridgeCommandApi& api) {
            return api.request_decoded_with_error(CommandCode::GET_SERVOS_ENABLED, {}, decode_servo_state, states);
        });
    if (!complete_command("get_servos_enabled", command_error, "command execution failed")) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return complete_command("set_servos_to_mid",
                            withCommandApi("set_servos_to_mid",
                                           CommandCode::SET_SERVOS_TO_MID,
                                           [](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_SERVOS_TO_MID, {});
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::get_led_info(bool& present, uint8_t& count) {
    protocol::LedInfo info{};
    const auto decode_led_info = [](const std::vector<uint8_t>& payload, protocol::LedInfo& decoded) {
        return protocol::decode_led_info(payload, decoded);
    };
    const BridgeError command_error = withCommandApi("get_led_info",
                                                     CommandCode::GET_LED_INFO,
                                                     [&info, &decode_led_info](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_LED_INFO,
                                                             {},
                                                             decode_led_info,
                                                             info);
                                                     });
    if (!complete_command("get_led_info", command_error, "command execution failed")) {
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

    return complete_command("set_led_colors",
                            withCommandApi("set_led_colors",
                                           CommandCode::SET_LED_COLORS,
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_LED_COLORS,
                                                   protocol::encode_led_colors(payload));
                                           }),
                            "command execution failed");
}

BridgeError SimpleHardwareBridge::send_calibrations_result(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        set_last_result(BridgeCommandResultMetadata{BridgeError::ProtocolFailure,
                                                    BridgeFailurePhase::CommandExecution,
                                                    BridgeFailureDomain::CommandProtocol,
                                                    false,
                                                    as_u8(CommandCode::SET_ANGLE_CALIBRATIONS),
                                                    kRequestedCapabilities,
                                                    static_cast<uint8_t>(link_manager_ ? link_manager_->negotiated_capabilities() : 0)});
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
            return api.request_ack_with_error(
                CommandCode::SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload));
        });

    if (command_error != BridgeError::None) {
        if (auto logger = resolveLogger(logger_)) {
            LOG_ERROR(logger, "calibration packet failed");
        }
        return command_error;
    }

    if (auto logger = resolveLogger(logger_)) {
        LOG_INFO(logger, "calibration packet accepted");
    }
    return BridgeError::None;
}
