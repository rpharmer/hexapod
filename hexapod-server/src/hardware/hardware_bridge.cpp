#include "hardware_bridge.hpp"

#include <utility>

#include "bridge_command_api.hpp"
#include "bridge_link_manager.hpp"
#include "command_client.hpp"
#include "hardware_bridge_internal.hpp"
#include "hardware_state_codec.hpp"
#include "hexapod-common.hpp"
#include "joint_feedback_estimator.hpp"
#include "logger.hpp"

using hardware_bridge::detail::BridgeResultMetadataBuilder;
using hardware_bridge::detail::kRequestedCapabilities;

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
        if (auto logger = hardware_bridge::detail::resolve_logger(logger_)) {
            LOG_ERROR(logger,
                      "command '{}' failed: {} (error={}, phase={}, domain={}, retryable={}, cmd_code={}, req_caps={}, nego_caps={})",
                      command_name,
                      reason != nullptr ? reason : hardware_bridge::detail::bridge_error_to_text(error),
                      hardware_bridge::detail::bridge_error_to_text(error),
                      hardware_bridge::detail::bridge_phase_to_text(last_result_.phase),
                      hardware_bridge::detail::bridge_domain_to_text(last_result_.domain),
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
        const BridgeResultMetadataBuilder metadata_builder{kRequestedCapabilities};
        set_last_result(metadata_builder.build(
            BridgeError::NotReady,
            BridgeFailurePhase::Readiness,
            BridgeFailureDomain::CommandProtocol,
            false,
            0,
            static_cast<uint8_t>(link_manager_ ? link_manager_->negotiated_capabilities() : 0)));
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
    const BridgeResultMetadataBuilder metadata_builder{kRequestedCapabilities};
    const BridgeError ready_error = requireReady(command_name, require_estimator);
    if (ready_error != BridgeError::None) {
        return ready_error;
    }

    const auto ensure = link_manager_->ensure_link_with_status(kRequestedCapabilities);
    if (!ensure.ok) {
        if (ensure.failure == BridgeLinkManager::EnsureLinkFailure::CapabilityNegotiation) {
            set_last_result(metadata_builder.build(BridgeError::ProtocolFailure,
                                                  BridgeFailurePhase::CapabilityNegotiation,
                                                  BridgeFailureDomain::CapabilityProtocol,
                                                  true,
                                                  as_u8(command_code),
                                                  ensure.negotiated_capabilities));
            return BridgeError::ProtocolFailure;
        }
        set_last_result(metadata_builder.build(BridgeError::TransportFailure,
                                              BridgeFailurePhase::CommandTransport,
                                              BridgeFailureDomain::TransportLink,
                                              true,
                                              as_u8(command_code),
                                              ensure.negotiated_capabilities));
        return BridgeError::TransportFailure;
    }

    if (!command_api_) {
        set_last_result(metadata_builder.build(BridgeError::NotReady,
                                              BridgeFailurePhase::Readiness,
                                              BridgeFailureDomain::CommandProtocol,
                                              false,
                                              as_u8(command_code),
                                              link_manager_->negotiated_capabilities()));
        return BridgeError::NotReady;
    }

    const BridgeError error = action(*command_api_);
    const auto command_meta = command_api_->last_result_metadata();
    set_last_result(metadata_builder.build(error,
                                          command_meta.phase,
                                          command_meta.domain,
                                          command_meta.retryable,
                                          command_meta.command_code,
                                          link_manager_->negotiated_capabilities()));
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

    const BridgeResultMetadataBuilder metadata_builder{kRequestedCapabilities};

    if (!link_manager_->init(kRequestedCapabilities)) {
        set_last_result(metadata_builder.build(BridgeError::TransportFailure,
                                              BridgeFailurePhase::Initialization,
                                              BridgeFailureDomain::TransportLink,
                                              true,
                                              as_u8(CommandCode::HELLO),
                                              0));
        return complete_command("init", BridgeError::TransportFailure, "link manager init failed");
    }

    CommandClient* command_client = link_manager_->command_client();
    if (command_client == nullptr) {
        set_last_result(metadata_builder.build(BridgeError::NotReady,
                                              BridgeFailurePhase::Initialization,
                                              BridgeFailureDomain::CommandProtocol,
                                              false,
                                              0,
                                              link_manager_->negotiated_capabilities()));
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
