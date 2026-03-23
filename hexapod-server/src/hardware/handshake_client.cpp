#include "handshake_client.hpp"

#include <vector>

#include "command_client.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"
#include "protocol_codec.hpp"
#include "transport_session.hpp"

HandshakeClient::HandshakeClient(TransportSession& transport, CommandClient& command_client)
    : transport_(transport), command_client_(command_client) {}

bool HandshakeClient::establish_link(uint8_t requested_caps) {
    const protocol::HelloRequest request{PROTOCOL_VERSION, requested_caps};
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(
            HELLO, protocol::encode_hello_request(request), ack_payload)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "handshake failed");
        }
        return false;
    }

    protocol::HelloAck ack{};
    if (!protocol::decode_hello_ack(ack_payload, ack)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "malformed ACK response");
        }
        return false;
    }

    if (ack.version != PROTOCOL_VERSION) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "version mismatch");
        }
        return false;
    }

    if (ack.status != STATUS_OK) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "device not ready");
        }
        return false;
    }

    transport_.reset_activity();
    return true;
}

bool HandshakeClient::send_heartbeat() {
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(HEARTBEAT, {}, ack_payload)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "heartbeat failed");
        }
        return false;
    }

    protocol::HelloAck heartbeat{};
    if (!protocol::decode_hello_ack(ack_payload, heartbeat)) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "heartbeat malformed payload");
        }
        return false;
    }

    if (heartbeat.status != STATUS_OK) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger, "heartbeat returned non-ok status: ", static_cast<unsigned>(heartbeat.status));
        }
        return false;
    }

    return true;
}
