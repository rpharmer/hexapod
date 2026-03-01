#include "pico/stdlib.h"
#include "serialCommsClient.hpp"

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
}


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


/* functions to send data */

// send a char (1 byte)
void SerialCommsClient::send_char(char data)
{
  putchar_raw(data);
}
// send a uint8_t (1 bytes)
void SerialCommsClient::send_u8(uint8_t data)
{
  putchar_raw(*reinterpret_cast<char*>(&data));
}
// send a uint16_t (2 bytes)
void SerialCommsClient::send_u16(uint16_t data)
{
  putchar_raw(reinterpret_cast<char*>(&data)[0]);
  putchar_raw(reinterpret_cast<char*>(&data)[1]);
}
// send a uint32_t (4 bytes)
void SerialCommsClient::send_u32(uint32_t data)
{
  putchar_raw(reinterpret_cast<char*>(&data)[0]);
  putchar_raw(reinterpret_cast<char*>(&data)[1]);
  putchar_raw(reinterpret_cast<char*>(&data)[2]);
  putchar_raw(reinterpret_cast<char*>(&data)[3]);
}
// send a int16_t  (2 bytes)
void SerialCommsClient::send_i16(int16_t data)
{
  putchar_raw(reinterpret_cast<char*>(&data)[0]);
  putchar_raw(reinterpret_cast<char*>(&data)[1]);
}
// send a int32_t  (4 bytes)
void SerialCommsClient::send_i32(int32_t data)
{
  putchar_raw(reinterpret_cast<char*>(&data)[0]);
  putchar_raw(reinterpret_cast<char*>(&data)[1]);
  putchar_raw(reinterpret_cast<char*>(&data)[2]);
  putchar_raw(reinterpret_cast<char*>(&data)[3]);
}
// send a float    (4 bytes)
void SerialCommsClient::send_f32(float data)
{
  putchar_raw(reinterpret_cast<char*>(&data)[0]);
  putchar_raw(reinterpret_cast<char*>(&data)[1]);
  putchar_raw(reinterpret_cast<char*>(&data)[2]);
  putchar_raw(reinterpret_cast<char*>(&data)[3]);
}

/* functions to recieve data */

// receive a char (1 byte)
int SerialCommsClient::recv_char(char *data)
{
  return recv_bytes(data, sizeof(char));
}
// recieve a uint8_t (1 bytes)
int SerialCommsClient::recv_u8(uint8_t *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(uint8_t));
}
// recieve a uint16_t (2 bytes)
int SerialCommsClient::recv_u16(uint16_t *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(uint16_t));
}
// recieve a uint32_t (4 bytes)
int SerialCommsClient::recv_u32(uint32_t *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(uint32_t));
}
// recieve a int16_t  (2 bytes)
int SerialCommsClient::recv_i16(int16_t *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(int16_t));
}
// recieve a int32_t  (4 bytes)
int SerialCommsClient::recv_i32(int32_t *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(int32_t));
}
// recieve a float    (4 bytes)
int SerialCommsClient::recv_f32(float *data)
{
  return recv_bytes(reinterpret_cast<char*>(data), sizeof(float));
}

void SerialCommsClient::send_packet(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)
{
  const std::vector<uint8_t> frame = encodePacket(seq, cmd, payload);
  for(uint8_t b : frame)
    send_u8(b);
}

bool SerialCommsClient::recv_packet(DecodedPacket& packet)
{
  while(true)
  {
    if(tryDecodePacket(rxBuffer, packet))
      return true;

    uint8_t byte = 0;
    if(recv_u8(&byte) < 0)
      return false;
    rxBuffer.push_back(byte);
  }
}
