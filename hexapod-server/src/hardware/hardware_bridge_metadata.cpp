#include "hardware_bridge_internal.hpp"

#include "logger.hpp"

namespace hardware_bridge::detail {

const char* bridge_error_to_text(BridgeError error) {
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

const char* bridge_phase_to_text(BridgeFailurePhase phase) {
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

const char* bridge_domain_to_text(BridgeFailureDomain domain) {
    switch (domain) {
        case BridgeFailureDomain::None: return "none";
        case BridgeFailureDomain::CapabilityProtocol: return "capability_protocol";
        case BridgeFailureDomain::TransportLink: return "transport_link";
        case BridgeFailureDomain::CommandProtocol: return "command_protocol";
    }
    return "none";
}

BridgeResultMetadataBuilder::BridgeResultMetadataBuilder(uint8_t requested_capabilities)
    : requested_capabilities_(requested_capabilities) {}

BridgeCommandResultMetadata BridgeResultMetadataBuilder::build(BridgeError error,
                                                               BridgeFailurePhase phase,
                                                               BridgeFailureDomain domain,
                                                               bool retryable,
                                                               uint8_t command_code,
                                                               uint8_t negotiated_capabilities) const {
    return BridgeCommandResultMetadata{error,
                                       phase,
                                       domain,
                                       retryable,
                                       command_code,
                                       requested_capabilities_,
                                       negotiated_capabilities};
}

bool decode_scalar_float_payload(const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
    return protocol::decode_scalar_float(payload, decoded);
}

std::shared_ptr<logging::AsyncLogger> resolve_logger(const std::shared_ptr<logging::AsyncLogger>& logger) {
    if (logger) {
        return logger;
    }
    return logging::GetDefaultLogger();
}

}  // namespace hardware_bridge::detail
