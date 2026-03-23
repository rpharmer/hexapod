#include "transport_session.hpp"

#include "hardware_bridge.hpp"
#include "hexapod-common.hpp"
#include "logger.hpp"

TransportSession::TransportSession(IPacketEndpoint& endpoint,
                                   DurationUs link_timeout,
                                   DurationUs heartbeat_interval)
    : endpoint_(endpoint),
      link_timeout_(link_timeout),
      heartbeat_interval_(heartbeat_interval) {}

uint16_t TransportSession::next_sequence() {
    return seq_++;
}

bool TransportSession::send(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
    endpoint_.send_packet(seq, cmd, payload);
    mark_transfer();
    return true;
}

bool TransportSession::recv(DecodedPacket& packet) {
    if (!endpoint_.recv_packet(packet)) {
        return false;
    }
    mark_transfer();
    return true;
}

TransportSession::CommandOutcome TransportSession::wait_for_ack(uint16_t seq) {
    DecodedPacket response;
    if (!recv(response)) {
        return CommandOutcome{OutcomeClass::Timeout, {}, 0};
    }
    return parse_ack_or_nack(response, seq);
}

TransportSession::CommandOutcome TransportSession::parse_ack_or_nack(
    const DecodedPacket& response,
    uint16_t expected_seq) const {
    if (response.seq != expected_seq) {
        if (auto logger = logging::GetDefaultLogger()) {
            LOG_ERROR(logger,
                      "sequence mismatch (expected ",
                      static_cast<unsigned>(expected_seq),
                      ", got ",
                      static_cast<unsigned>(response.seq),
                      ")");
        }
        return CommandOutcome{OutcomeClass::ProtocolError, {}, 0};
    }

    if (response.cmd == ACK) {
        return CommandOutcome{OutcomeClass::Success, response.payload, 0};
    }

    if (response.cmd == NACK) {
        const uint8_t nack_code = response.payload.empty() ? 0 : response.payload[0];
        if (!response.payload.empty()) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "NACK, error: ", static_cast<unsigned>(nack_code));
            }
        } else {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger, "NACK without error code");
            }
        }
        return CommandOutcome{OutcomeClass::Nack, {}, nack_code};
    }

    if (auto logger = logging::GetDefaultLogger()) {
        LOG_ERROR(logger, "unexpected response cmd: ", static_cast<unsigned>(response.cmd));
    }
    return CommandOutcome{OutcomeClass::ProtocolError, {}, 0};
}

bool TransportSession::has_link_timed_out(TimePointUs now) const {
    if (!last_transfer_us_.isZero() && ((now - last_transfer_us_) > link_timeout_)) {
        return true;
    }
    return false;
}

bool TransportSession::heartbeat_due(TimePointUs now) const {
    if (last_transfer_us_.isZero()) {
        return true;
    }
    return (now - last_transfer_us_) > heartbeat_interval_;
}

void TransportSession::reset_activity() {
    last_transfer_us_ = now_us();
}

void TransportSession::mark_transfer() {
    last_transfer_us_ = now_us();
}
