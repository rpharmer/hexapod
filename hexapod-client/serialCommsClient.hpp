#ifndef HEXAPOD_COMMS_CLIENT_H
#define HEXAPOD_COMMS_CLIENT_H

#include <stdint.h>
#include "serial_comms.hpp"

// Concrete class derived from SerialComms defining which communication functions
class SerialCommsClient : public SerialComms
{
private:
    int32_t _timeout_ms;
public:
    // constructor
    SerialCommsClient();

    // opens COM port for use, must be called before you configure port
    void Open();
    // closes COM port
    void Close();
    /// \brief      Sets the read timeout (in milliseconds)/blocking mode.
    /// \details    Only call when state != OPEN. This method manipulates VMIN and VTIME.
    /// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
    ///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
    ///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
    ///             25500ms (another Linux API restriction).
    void SetTimeout(int32_t timeout_ms);

    // framed protocol helpers
    void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) override;
    bool recv_packet(DecodedPacket& packet) override;
};

#endif
