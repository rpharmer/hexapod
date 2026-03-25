#pragma once

#include <cstdint>
#include <memory>

#include "protocol_ids.hpp"
#include "logger.hpp"

class TransportSession;
class CommandClient;

class HandshakeClient {
public:
    HandshakeClient(TransportSession& transport,
                    CommandClient& command_client,
                    std::shared_ptr<logging::AsyncLogger> logger = nullptr);

    bool establish_link(uint8_t requested_caps);
    bool send_heartbeat();
    bool has_capability(uint8_t capability) const;
    uint8_t negotiated_capabilities() const;

private:
    TransportSession& transport_;
    CommandClient& command_client_;
    std::shared_ptr<logging::AsyncLogger> logger_{};
    uint8_t negotiated_capabilities_{CAPABILITY_ANGULAR_FEEDBACK};
};
