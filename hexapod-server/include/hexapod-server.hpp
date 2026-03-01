// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"


bool do_handshake(SerialCommsServer& sc, uint16_t seq, uint8_t requested_caps);
bool send_heartbeat(SerialCommsServer& sc, uint16_t seq);

#endif  // #ifndef HEXAPOD_SERVER_HPP