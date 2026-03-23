#pragma once

#include <cstdint>
#include <vector>

#include "transport_session.hpp"

class CommandClient {
public:
    struct RetryPolicy {
        int max_attempts{3};
    };

    explicit CommandClient(TransportSession& transport);

    void set_retry_policy(const RetryPolicy& retry_policy);

    bool send_command_and_expect_ack(uint8_t cmd, const std::vector<uint8_t>& payload = {});

    bool send_command_and_expect_ack_payload(uint8_t cmd,
                                             const std::vector<uint8_t>& payload,
                                             std::vector<uint8_t>& ack_payload);

    TransportSession::CommandOutcome transact(uint8_t cmd,
                                              const std::vector<uint8_t>& payload,
                                              std::vector<uint8_t>* ack_payload);

private:
    static const char* outcome_to_text(TransportSession::OutcomeClass outcome);
    static const char* command_name(uint8_t cmd);
    static const char* domain_error_for_outcome(TransportSession::OutcomeClass outcome);

    TransportSession& transport_;
    RetryPolicy retry_policy_{};
};
