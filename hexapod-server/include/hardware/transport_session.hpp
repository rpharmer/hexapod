#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "logger.hpp"
#include "serialCommsServer.hpp"
#include "types.hpp"

class IPacketEndpoint;

class TransportSession {
public:
    enum class OutcomeClass : uint8_t {
        Success,
        Nack,
        Timeout,
        RetryExhausted,
        ProtocolError
    };

    struct CommandOutcome {
        OutcomeClass outcome_class{OutcomeClass::ProtocolError};
        std::vector<uint8_t> ack_payload{};
        uint8_t nack_code{0};
    };

    TransportSession(IPacketEndpoint& endpoint,
                     DurationUs link_timeout,
                     DurationUs heartbeat_interval,
                     std::shared_ptr<logging::AsyncLogger> logger = nullptr);

    uint16_t next_sequence();
    bool send(uint16_t seq, CommandCode cmd, const std::vector<uint8_t>& payload);
    bool recv(DecodedPacket& packet);
    CommandOutcome wait_for_ack(uint16_t seq);
    CommandOutcome parse_ack_or_nack(const DecodedPacket& response,
                                     uint16_t expected_seq) const;

    bool has_link_timed_out(TimePointUs now) const;
    bool heartbeat_due(TimePointUs now) const;
    void reset_activity();

private:
    void mark_transfer();

    IPacketEndpoint& endpoint_;
    DurationUs link_timeout_{};
    DurationUs heartbeat_interval_{};
    std::shared_ptr<logging::AsyncLogger> logger_{};
    uint16_t seq_{0};
    TimePointUs last_transfer_us_{};
};
