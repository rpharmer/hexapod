#pragma once

#include <cstdint>
#include <vector>
#include <utility>

#include "transport_session.hpp"
#include "hexapod-common.hpp"

class CommandClient;

class BridgeCommandApi {
public:
    explicit BridgeCommandApi(CommandClient& command_client);

    bool request_ack(CommandCode cmd, const std::vector<uint8_t>& payload);
    bool request_ack_payload(CommandCode cmd,
                             const std::vector<uint8_t>& payload,
                             std::vector<uint8_t>& out_payload);

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

private:
    bool request_transaction(CommandCode cmd,
                             const std::vector<uint8_t>& payload,
                             std::vector<uint8_t>* out_payload);

    static void log_command_failure(CommandCode cmd,
                                    TransportSession::OutcomeClass outcome_class,
                                    uint8_t nack_code,
                                    std::size_t response_payload_size);
    static void log_decode_failure(CommandCode cmd, std::size_t response_payload_size);

    CommandClient& command_client_;
};
