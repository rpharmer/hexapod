// Header guard
#ifndef HEXAPOD_COMMS_SERVER_H
#define HEXAPOD_COMMS_SERVER_H

#include <cstddef>
#include <memory>
#include <CppLinuxSerial/SerialPort.hpp>
#include "serial_comms.hpp"
#include "framing.hpp"
#include "logger.hpp"


using namespace mn::CppLinuxSerial;
// Concrete class derived from SerialComms defining which communication functions
class SerialCommsServer : public SerialComms
{
private:
    SerialPort serialport;
    std::vector<uint8_t> readBuffer;
    std::size_t readBufferHead = 0;
    std::shared_ptr<logging::AsyncLogger> logger_{};

    int recv_bytes(void *data, std::size_t size);
    void refill_read_buffer();
public:
    
    static BaudRate int_to_baud_rate(int baud);

    // constructor
    SerialCommsServer(const std::string &device,
                      BaudRate baudRate,
                      NumDataBits numDataBits,
                      Parity parity,
                      NumStopBits numStopBits,
                      std::shared_ptr<logging::AsyncLogger> logger = nullptr);
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

#endif // #ifndef HEXAPOD_COMMS_SERVER_H
