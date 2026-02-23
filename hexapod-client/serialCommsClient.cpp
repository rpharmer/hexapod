#include "pico/stdlib.h"
#include "serialCommsClient.hpp"


// constructor
SerialCommsClient::SerialCommsClient(){}
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
void SerialCommsClient::SetTimeout(int32_t timeout_ms){}


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
  *data = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a uint8_t (1 bytes)
int SerialCommsClient::recv_u8(uint8_t *data)
{
  *data = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a uint16_t (2 bytes)
int SerialCommsClient::recv_u16(uint16_t *data)
{
  ((char*)data)[0] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[1] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a uint32_t (4 bytes)
int SerialCommsClient::recv_u32(uint32_t *data)
{
  ((char*)data)[0] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[1] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[2] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[3] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a int16_t  (2 bytes)
int SerialCommsClient::recv_i16(int16_t *data)
{
  ((char*)data)[0] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[1] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a int32_t  (4 bytes)
int SerialCommsClient::recv_i32(int32_t *data)
{
  ((char*)data)[0] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[1] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[2] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[3] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}
// recieve a float    (4 bytes)
int SerialCommsClient::recv_f32(float *data)
{
  ((char*)data)[0] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[1] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[2] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  ((char*)data)[3] = *reinterpret_cast<char*>(getchar_timeout_us(10));
  return 0;
}