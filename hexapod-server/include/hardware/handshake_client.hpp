#pragma once

#include <cstdint>

#include "hexapod-common.hpp"

class TransportSession;
class CommandClient;

class HandshakeClient {
public:
    HandshakeClient(TransportSession& transport, CommandClient& command_client);

    bool establish_link(uint8_t requested_caps);
    bool send_heartbeat();
    bool has_capability(uint8_t capability) const;
    uint8_t negotiated_capabilities() const;

private:
    TransportSession& transport_;
    CommandClient& command_client_;
    uint8_t negotiated_capabilities_{CAPABILITY_ANGULAR_FEEDBACK};
};
