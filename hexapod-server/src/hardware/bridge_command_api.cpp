#include "bridge_command_api.hpp"

#include <cstddef>

#include "command_client.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"

BridgeCommandApi::BridgeCommandApi(CommandClient& command_client)
    : command_client_(command_client) {}

bool BridgeCommandApi::request_ack(uint8_t cmd,
                                   const std::vector<uint8_t>& payload) {
    return request_transaction(cmd, payload, nullptr);
}

bool BridgeCommandApi::request_ack_payload(uint8_t cmd,
                                           const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& out_payload) {
    return request_transaction(cmd, payload, &out_payload);
}

bool BridgeCommandApi::request_transaction(uint8_t cmd,
                                           const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>* out_payload) {
    const auto outcome = command_client_.transact(cmd, payload, out_payload);
    if (outcome.outcome_class == TransportSession::OutcomeClass::Success) {
        return true;
    }

    const std::size_t payload_size = (out_payload != nullptr) ? out_payload->size() : 0;
    log_command_failure(cmd, outcome.outcome_class, outcome.nack_code, payload_size);
    return false;
}

void BridgeCommandApi::log_command_failure(uint8_t cmd,
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

void BridgeCommandApi::log_decode_failure(uint8_t cmd, std::size_t response_payload_size) {
    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger,
                  "command ",
                  command_name(cmd),
                  " decode failed (payload_size=",
                  static_cast<unsigned>(response_payload_size),
                  ")");
    }
}
