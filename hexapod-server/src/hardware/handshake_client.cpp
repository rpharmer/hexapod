#include "handshake_client.hpp"

#include <utility>
#include <vector>

#include "command_client.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"
#include "protocol_codec.hpp"
#include "transport_session.hpp"

HandshakeClient::HandshakeClient(TransportSession& transport,
                               CommandClient& command_client,
                               std::shared_ptr<logging::AsyncLogger> logger)
    : transport_(transport), command_client_(command_client), logger_(std::move(logger)) {}

bool HandshakeClient::establish_link(uint8_t requested_caps) {
    const protocol::HelloRequest request{PROTOCOL_VERSION, requested_caps};
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(
            HELLO, protocol::encode_hello_request(request), ack_payload)) {
        if (logger_) {
            LOG_ERROR(logger_, "handshake failed");
        }
        return false;
    }

    protocol::HelloAck ack{};
    if (!protocol::decode_hello_ack(ack_payload, ack)) {
        if (logger_) {
            LOG_ERROR(logger_, "malformed ACK response");
        }
        return false;
    }

    if (ack.version != PROTOCOL_VERSION) {
        if (logger_) {
            LOG_ERROR(logger_, "version mismatch");
        }
        return false;
    }

    if (ack.status != STATUS_OK) {
        if (logger_) {
            LOG_ERROR(logger_, "device not ready");
        }
        return false;
    }

    negotiated_capabilities_ = ack.capabilities;
    transport_.reset_activity();
    return true;
}

bool HandshakeClient::send_heartbeat() {
    std::vector<uint8_t> ack_payload;
    if (!command_client_.send_command_and_expect_ack_payload(HEARTBEAT, {}, ack_payload)) {
        if (logger_) {
            LOG_ERROR(logger_, "heartbeat failed");
        }
        return false;
    }

    protocol::HelloAck heartbeat{};
    if (!protocol::decode_hello_ack(ack_payload, heartbeat)) {
        if (logger_) {
            LOG_ERROR(logger_, "heartbeat malformed payload");
        }
        return false;
    }

    if (heartbeat.status != STATUS_OK) {
        if (logger_) {
            LOG_ERROR(logger_, "heartbeat returned non-ok status: ", static_cast<unsigned>(heartbeat.status));
        }
        return false;
    }

    negotiated_capabilities_ = heartbeat.capabilities;
    return true;
}

bool HandshakeClient::has_capability(uint8_t capability) const {
    return (negotiated_capabilities_ & capability) != 0;
}

uint8_t HandshakeClient::negotiated_capabilities() const {
    return negotiated_capabilities_;
}
