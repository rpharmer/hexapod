#include "command_client.hpp"

#include <chrono>
#include <utility>

#include "hexapod-common.hpp"
#include "logger.hpp"

CommandClient::CommandClient(TransportSession& transport,
                           std::shared_ptr<logging::AsyncLogger> logger)
    : transport_(transport), logger_(std::move(logger)) {}

void CommandClient::set_retry_policy(const RetryPolicy& retry_policy) {
    retry_policy_ = retry_policy;
}

bool CommandClient::send_command_and_expect_ack(uint8_t cmd, const std::vector<uint8_t>& payload) {
    return transact(cmd, payload, nullptr).outcome_class == TransportSession::OutcomeClass::Success;
}

bool CommandClient::send_command_and_expect_ack_payload(uint8_t cmd,
                                                        const std::vector<uint8_t>& payload,
                                                        std::vector<uint8_t>& ack_payload) {
    return transact(cmd, payload, &ack_payload).outcome_class == TransportSession::OutcomeClass::Success;
}

TransportSession::CommandOutcome CommandClient::transact(uint8_t cmd,
                                                         const std::vector<uint8_t>& payload,
                                                         std::vector<uint8_t>* ack_payload) {
    const int max_attempts = (retry_policy_.max_attempts > 0) ? retry_policy_.max_attempts : 1;
    const auto start = std::chrono::steady_clock::now();
    TransportSession::CommandOutcome last_outcome{TransportSession::OutcomeClass::ProtocolError, {}, 0};
    uint16_t seq = 0;
    int attempts_used = 0;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        seq = transport_.next_sequence();
        ++attempts_used;
        transport_.send(seq, cmd, payload);
        last_outcome = transport_.wait_for_ack(seq);
        if (last_outcome.outcome_class == TransportSession::OutcomeClass::Success) {
            break;
        }
        if (last_outcome.outcome_class == TransportSession::OutcomeClass::Nack) {
            break;
        }
    }

    if (last_outcome.outcome_class != TransportSession::OutcomeClass::Success &&
        last_outcome.outcome_class != TransportSession::OutcomeClass::Nack &&
        attempts_used >= max_attempts) {
        last_outcome.outcome_class = TransportSession::OutcomeClass::RetryExhausted;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count();
    if (logger_) {
        LOG_INFO(logger_,
                 "telemetry command=",
                 command_name(cmd),
                 " seq=",
                 static_cast<unsigned>(seq),
                 " latency_us=",
                 elapsed,
                 " outcome=",
                 outcome_to_text(last_outcome.outcome_class),
                 " attempts=",
                 attempts_used);
        if (last_outcome.outcome_class != TransportSession::OutcomeClass::Success) {
            LOG_WARN(logger_,
                     "command_failure command=",
                     command_name(cmd),
                     " domain_error=",
                     domain_error_for_outcome(last_outcome.outcome_class),
                     " nack_code=",
                     static_cast<unsigned>(last_outcome.nack_code));
        }
    }

    if (last_outcome.outcome_class == TransportSession::OutcomeClass::Success && ack_payload) {
        *ack_payload = last_outcome.ack_payload;
    }

    return last_outcome;
}

const char* CommandClient::outcome_to_text(TransportSession::OutcomeClass outcome) {
    switch (outcome) {
        case TransportSession::OutcomeClass::Success:
            return "success";
        case TransportSession::OutcomeClass::Nack:
            return "nack";
        case TransportSession::OutcomeClass::Timeout:
            return "timeout";
        case TransportSession::OutcomeClass::RetryExhausted:
            return "retry_exhausted";
        case TransportSession::OutcomeClass::ProtocolError:
            return "protocol_error";
    }
    return "protocol_error";
}

const char* CommandClient::command_name(uint8_t cmd) {
    return ::command_name(cmd);
}

const char* CommandClient::domain_error_for_outcome(TransportSession::OutcomeClass outcome) {
    switch (outcome) {
        case TransportSession::OutcomeClass::Success:
            return "none";
        case TransportSession::OutcomeClass::Nack:
            return "device_rejected";
        case TransportSession::OutcomeClass::Timeout:
            return "response_timeout";
        case TransportSession::OutcomeClass::RetryExhausted:
            return "retry_exhausted";
        case TransportSession::OutcomeClass::ProtocolError:
            return "protocol_violation";
    }
    return "protocol_violation";
}
