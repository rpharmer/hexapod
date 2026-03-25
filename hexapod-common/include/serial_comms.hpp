#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <cstdint>
#include <vector>

#include "framing.hpp"

/* Abstract base class defining which communication functions must be implemented. Server
 and Client use different libaries but derived class will hide this */
class SerialComms {
public:
    // framed protocol helpers
    virtual void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) = 0;
    virtual bool recv_packet(DecodedPacket& packet) = 0;

protected:
    std::vector<uint8_t> rxBuffer;
};

#endif
