#include "pico/stdlib.h"
#include "serialCommsClient.hpp"

#include <cstddef>

namespace {

int recv_bytes(char *buffer, size_t len) {
  for(size_t i = 0; i < len; ++i) {
    const int ch = getchar_timeout_us(1000);
    if(ch < 0) {
      return ch;
    }
    buffer[i] = static_cast<char>(ch);
  }
  return static_cast<int>(len);
}

void write_bytes(const uint8_t* bytes, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    putchar_raw(static_cast<char>(bytes[i]));
  }
}

} // namespace


// constructor
SerialCommsClient::SerialCommsClient()
{
  _timeout_ms = 100;
}
// opens COM port for use, must be called before you configure port
void SerialCommsClient::Open(){}
// closes COM port
void SerialCommsClient::Close(){}
/// \brief      Sets the read timeout (in milliseconds)/blocking mode.
/// \details    Only call when state != OPEN. This method manipulates VMIN and VTIME.
/// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
///             25500ms (another Linux API restriction).
void SerialCommsClient::SetTimeout(int32_t timeout_ms){_timeout_ms = timeout_ms;}


void SerialCommsClient::send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)
{
  const std::vector<uint8_t> frame = encodePacket(seq, cmd, payload);
  if (!frame.empty()) {
    write_bytes(frame.data(), frame.size());
  }
}

bool SerialCommsClient::recv_packet(DecodedPacket& packet)
{
  while(true)
  {
    if(tryDecodePacket(rxBuffer, packet))
      return true;

    uint8_t byte = 0;
    const int bytes_read = recv_bytes(reinterpret_cast<char*>(&byte), sizeof(byte));
    if(bytes_read <= 0)
      return false;
    rxBuffer.push_back(byte);
    trim_rx_buffer(rxBuffer, MAX_TRANSPORT_RX_BUFFER_BYTES);
  }
}
