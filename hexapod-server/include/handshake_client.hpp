#pragma once

#include <cstdint>

class TransportSession;
class CommandClient;

class HandshakeClient {
public:
    HandshakeClient(TransportSession& transport, CommandClient& command_client);

    bool establish_link(uint8_t requested_caps);
    bool send_heartbeat();

private:
    TransportSession& transport_;
    CommandClient& command_client_;
};
