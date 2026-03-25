#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "logger.hpp"
#include "protocol_ids.hpp"
#include "transport_session.hpp"

class CommandClient {
public:
    struct RetryPolicy {
        int max_attempts{3};
    };

    explicit CommandClient(TransportSession& transport,
                           std::shared_ptr<logging::AsyncLogger> logger = nullptr);

    void set_retry_policy(const RetryPolicy& retry_policy);

    bool send_command_and_expect_ack(CommandCode cmd, const std::vector<uint8_t>& payload = {});

    bool send_command_and_expect_ack_payload(CommandCode cmd,
                                             const std::vector<uint8_t>& payload,
                                             std::vector<uint8_t>& ack_payload);

    TransportSession::CommandOutcome transact(CommandCode cmd,
                                              const std::vector<uint8_t>& payload,
                                              std::vector<uint8_t>* ack_payload);

private:
    static const char* outcome_to_text(TransportSession::OutcomeClass outcome);
    static const char* command_name(CommandCode cmd);
    static const char* domain_error_for_outcome(TransportSession::OutcomeClass outcome);

    TransportSession& transport_;
    std::shared_ptr<logging::AsyncLogger> logger_{};
    RetryPolicy retry_policy_{};
};
