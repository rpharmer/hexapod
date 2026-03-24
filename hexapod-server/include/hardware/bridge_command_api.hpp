#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "hardware_bridge.hpp"
#include "transport_session.hpp"
#include "hexapod-common.hpp"

class CommandClient;

class BridgeCommandApi {
public:
    struct ResultMetadata {
        BridgeError error{BridgeError::None};
        BridgeFailurePhase phase{BridgeFailurePhase::None};
        BridgeFailureDomain domain{BridgeFailureDomain::None};
        bool retryable{false};
        uint8_t command_code{0};
        uint8_t nack_code{0};
    };

    explicit BridgeCommandApi(CommandClient& command_client);

    // Migration note: bool methods remain for compatibility. New call sites should
    // prefer *_with_error variants so failures can be classified via BridgeError.
    bool request_ack(CommandCode cmd, const std::vector<uint8_t>& payload);
    BridgeError request_ack_with_error(CommandCode cmd, const std::vector<uint8_t>& payload);
    bool request_ack_payload(CommandCode cmd,
                             const std::vector<uint8_t>& payload,
                             std::vector<uint8_t>& out_payload);
    BridgeError request_ack_payload_with_error(CommandCode cmd,
                                               const std::vector<uint8_t>& payload,
                                               std::vector<uint8_t>& out_payload);
    ResultMetadata last_result_metadata() const;

    template <typename T, typename Decoder>
    bool request_decoded(CommandCode cmd,
                         const std::vector<uint8_t>& payload,
                         Decoder&& decoder,
                         T& out) {
        std::vector<uint8_t> response_payload;
        if (!request_ack_payload(cmd, payload, response_payload)) {
            return false;
        }

        if (!decoder(response_payload, out)) {
            log_decode_failure(cmd, response_payload.size());
            return false;
        }

        return true;
    }

    template <typename T, typename Decoder>
    BridgeError request_decoded_with_error(CommandCode cmd,
                                           const std::vector<uint8_t>& payload,
                                           Decoder&& decoder,
                                           T& out) {
        std::vector<uint8_t> response_payload;
        const BridgeError request_error =
            request_ack_payload_with_error(cmd, payload, response_payload);
        if (request_error != BridgeError::None) {
            return request_error;
        }

        if (!decoder(response_payload, out)) {
            log_decode_failure(static_cast<CommandCode>(cmd), response_payload.size());
            last_result_.error = BridgeError::ProtocolFailure;
            last_result_.phase = BridgeFailurePhase::CommandDecode;
            last_result_.domain = BridgeFailureDomain::CommandProtocol;
            last_result_.retryable = false;
            return BridgeError::ProtocolFailure;
        }

        return BridgeError::None;
    }

    template <typename T, typename Decoder>
    BridgeError request_decoded_with_error(uint8_t,
                                           const std::vector<uint8_t>&,
                                           Decoder&&,
                                           T&) = delete;

private:
    BridgeError request_transaction(CommandCode cmd,
                                    const std::vector<uint8_t>& payload,
                                    std::vector<uint8_t>* out_payload);
    static BridgeError map_outcome_to_bridge_error(const TransportSession::CommandOutcome& outcome);

    static void log_command_failure(CommandCode cmd,
                                    TransportSession::OutcomeClass outcome_class,
                                    uint8_t nack_code,
                                    std::size_t response_payload_size);
    static void log_decode_failure(CommandCode cmd, std::size_t response_payload_size);

    CommandClient& command_client_;
    ResultMetadata last_result_{};
};
