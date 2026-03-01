// Header guard
#ifndef HEXAPOD_COMMS_H
#define HEXAPOD_COMMS_H

#include "hexapod-common.hpp"
#include "framing.hpp"


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


    /* functions to send data */
    
    // send a char (1 byte)
    void send_char(char data) override;
    // send a uint8_t (1 bytes)
    void send_u8(uint8_t data) override;
    // send a uint16_t (2 bytes)
    void send_u16(uint16_t data) override;
    // send a uint32_t (4 bytes)
    void send_u32(uint32_t data) override;
    // send a int16_t  (2 bytes)
    void send_i16(int16_t data) override;
    // send a int32_t  (4 bytes)
    void send_i32(int32_t data) override;
    // send a float    (4 bytes)
    void send_f32(float data) override;
    
    /* functions to recieve data */
    
    // receive a char (1 byte)
    int recv_char(char *data) override;
    // recieve a uint8_t (1 bytes)
    int recv_u8(uint8_t *data) override;
    // recieve a uint16_t (2 bytes)
    int recv_u16(uint16_t *data) override;
    // recieve a uint32_t (4 bytes)
    int recv_u32(uint32_t *data) override;
    // recieve a int16_t  (2 bytes)
    int recv_i16(int16_t *data) override;
    // recieve a int32_t  (4 bytes)
    int recv_i32(int32_t *data) override;
    // recieve a float    (4 bytes)
    int recv_f32(float *data) override;

    // framed protocol helpers
    void send_packet(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) override;
    bool recv_packet(DecodedPacket& packet) override;
};

#endif
