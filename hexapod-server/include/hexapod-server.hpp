// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"


bool do_handshake(SerialCommsServer& sc, uint8_t seq, uint8_t requested_caps);

#endif  // #ifndef HEXAPOD_SERVER_HPP