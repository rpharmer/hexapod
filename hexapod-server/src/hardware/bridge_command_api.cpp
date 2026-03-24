#include "bridge_command_api.hpp"

#include <cstddef>

#include "command_client.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"

BridgeCommandApi::BridgeCommandApi(CommandClient& command_client)
    : command_client_(command_client) {}

bool BridgeCommandApi::request_ack(CommandCode cmd,
                                   const std::vector<uint8_t>& payload) {
    return request_ack_with_error(cmd, payload) == BridgeError::None;
}

bool BridgeCommandApi::request_ack(uint8_t cmd,
                                   const std::vector<uint8_t>& payload) {
    return request_ack_with_error(cmd, payload) == BridgeError::None;
}

BridgeError BridgeCommandApi::request_ack_with_error(CommandCode cmd,
                                                     const std::vector<uint8_t>& payload) {
    return request_ack_with_error(as_u8(cmd), payload);
}

BridgeError BridgeCommandApi::request_ack_with_error(uint8_t cmd,
                                                     const std::vector<uint8_t>& payload) {
    return request_transaction(cmd, payload, nullptr);
}

bool BridgeCommandApi::request_ack_payload(CommandCode cmd,
                                           const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& out_payload) {
    return request_ack_payload_with_error(cmd, payload, out_payload) == BridgeError::None;
}

bool BridgeCommandApi::request_ack_payload(uint8_t cmd,
                                           const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& out_payload) {
    return request_ack_payload_with_error(cmd, payload, out_payload) == BridgeError::None;
}

BridgeError BridgeCommandApi::request_ack_payload_with_error(CommandCode cmd,
                                                             const std::vector<uint8_t>& payload,
                                                             std::vector<uint8_t>& out_payload) {
    return request_ack_payload_with_error(as_u8(cmd), payload, out_payload);
}

BridgeError BridgeCommandApi::request_ack_payload_with_error(uint8_t cmd,
                                                             const std::vector<uint8_t>& payload,
                                                             std::vector<uint8_t>& out_payload) {
    return request_transaction(cmd, payload, &out_payload);
}

BridgeError BridgeCommandApi::request_transaction(uint8_t cmd,
                                                  const std::vector<uint8_t>& payload,
                                                  std::vector<uint8_t>* out_payload) {
    const auto outcome = command_client_.transact(cmd, payload, out_payload);
    if (outcome.outcome_class == TransportSession::OutcomeClass::Success) {
        return BridgeError::None;
    }

    const std::size_t payload_size = (out_payload != nullptr) ? out_payload->size() : 0;
    log_command_failure(cmd, outcome.outcome_class, outcome.nack_code, payload_size);
    return map_outcome_to_bridge_error(outcome);
}

BridgeError BridgeCommandApi::map_outcome_to_bridge_error(
    const TransportSession::CommandOutcome& outcome) {
    switch (outcome.outcome_class) {
        case TransportSession::OutcomeClass::Success:
            return BridgeError::None;
        case TransportSession::OutcomeClass::Nack:
            if (outcome.nack_code == UNSUPPORTED_COMMAND) {
                return BridgeError::Unsupported;
            }
            if (outcome.nack_code == BUSY_NOT_READY) {
                return BridgeError::NotReady;
            }
            return BridgeError::ProtocolFailure;
        case TransportSession::OutcomeClass::Timeout:
        case TransportSession::OutcomeClass::RetryExhausted:
            return BridgeError::Timeout;
        case TransportSession::OutcomeClass::ProtocolError:
            return BridgeError::ProtocolFailure;
    }
    return BridgeError::ProtocolFailure;
}

void BridgeCommandApi::log_command_failure(CommandCode cmd,
                                           TransportSession::OutcomeClass outcome_class,
                                           uint8_t nack_code,
                                           std::size_t response_payload_size) {
    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name(cmd),
                  " failed (outcome=",
                  static_cast<unsigned>(outcome_class),
                  ", nack_code=",
                  static_cast<unsigned>(nack_code),
                  ", payload_size=",
                  static_cast<unsigned>(response_payload_size),
                  ")");
    }
}

void BridgeCommandApi::log_decode_failure(CommandCode cmd, std::size_t response_payload_size) {
    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name(cmd),
                  " decode failed (payload_size=",
                  static_cast<unsigned>(response_payload_size),
                  ")");
    }
}
