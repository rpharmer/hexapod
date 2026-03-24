#include "hardware_bridge.hpp"

#include <utility>

#include "bridge_command_api.hpp"
#include "bridge_link_manager.hpp"
#include "command_client.hpp"
#include "hardware_state_codec.hpp"
#include "joint_feedback_estimator.hpp"
#include "logger.hpp"

namespace {

constexpr uint8_t kRequestedCapabilities = CAPABILITY_ANGULAR_FEEDBACK;

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

bool SimpleHardwareBridge::complete_command(const char* command_name,
                                            BridgeError error,
                                            const char* reason) {
    last_error_ = error;
    if (error != BridgeError::None) {
        if (auto logger = resolveLogger(logger_)) {
            LOG_ERROR(logger,
                      "command '{}' failed: {} ({})",
                      command_name,
                      reason != nullptr ? reason : bridge_error_to_text(error),
                      bridge_error_to_text(error));
        }
        return false;
    }
    return true;
}

BridgeError SimpleHardwareBridge::requireReady(const char* command_name, bool require_estimator) {
    (void)command_name;
    if (!initialized_ || !link_manager_ || (require_estimator && !feedback_estimator_)) {
        return BridgeError::NotReady;
    }
    return BridgeError::None;
}

BridgeError SimpleHardwareBridge::withCommandApi(
    const char* command_name,
    const std::function<BridgeError(BridgeCommandApi&)>& action,
    bool require_estimator) {
    (void)command_name;
    const BridgeError ready_error = requireReady(command_name, require_estimator);
    if (ready_error != BridgeError::None) {
        return ready_error;
    }
    if (!link_manager_->ensure_link(kRequestedCapabilities)) {
        return BridgeError::TransportFailure;
    }
    if (!command_api_) {
        return BridgeError::NotReady;
    }
    return action(*command_api_);
}

BridgeError SimpleHardwareBridge::last_error() const {
    return last_error_;
}

bool SimpleHardwareBridge::init() {
    last_error_ = BridgeError::None;
    link_manager_ = std::make_unique<BridgeLinkManager>(
        device_, baud_rate_, timeout_ms_, std::move(packet_endpoint_));

    if (!link_manager_->init(kRequestedCapabilities)) {
        return complete_command("init", BridgeError::TransportFailure, "link manager init failed");
    }

    CommandClient* command_client = link_manager_->command_client();
    if (command_client == nullptr) {
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
    last_error_ = BridgeError::None;
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
