#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"
#include "serialCommsServer.hpp"

#include <cstring>

using namespace mn::CppLinuxSerial;

BaudRate SerialCommsServer::int_to_baud_rate(int baud)
{
  switch (baud)
  {
    case 0: return BaudRate::B_0;
    case 50: return BaudRate::B_50;
    case 75: return BaudRate::B_75;
    case 110: return BaudRate::B_110;
    case 134: return BaudRate::B_134;
    case 150: return BaudRate::B_150;
    case 200: return BaudRate::B_200;
    case 300: return BaudRate::B_300;
    case 600: return BaudRate::B_600;
    case 1200: return BaudRate::B_1200;
    case 1800: return BaudRate::B_1800;
    case 2400: return BaudRate::B_2400;
    case 4800: return BaudRate::B_4800;
    case 9600: return BaudRate::B_9600;
    case 19200: return BaudRate::B_19200;
    case 38400: return BaudRate::B_38400;
    case 57600: return BaudRate::B_57600;
    case 115200: return BaudRate::B_115200;
    case 230400: return BaudRate::B_230400;
    case 460800: return BaudRate::B_460800;
    default: return BaudRate::B_CUSTOM;
  }
}

// constructor
SerialCommsServer::SerialCommsServer(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits) : serialport(device,baudRate, numDataBits, parity, numStopBits)
{}

// opens COM port for use, must be called before you configure port
void SerialCommsServer::Open()
{
  serialport.Open();
}
// closes COM port
void SerialCommsServer::Close()
{
  serialport.Close();
}
/// \brief      Sets the read timeout (in milliseconds)/blocking mode.
/// \details    Only call when state != OPEN. This method manipulates VMIN and VTIME.
/// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
///             25500ms (another Linux API restriction).
void SerialCommsServer::SetTimeout(int32_t timeout_ms)
{
  serialport.SetTimeout(timeout_ms);
}


/* functions to send data */

// send a char (1 byte)
void SerialCommsServer::send_char(char data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(char)));
}
// send a uint8_t (1 bytes)
void SerialCommsServer::send_u8(uint8_t data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(uint8_t)));
}
// send a uint16_t (2 bytes)
void SerialCommsServer::send_u16(uint16_t data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(uint16_t)));
}
// send a uint32_t (4 bytes)
void SerialCommsServer::send_u32(uint32_t data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(uint32_t)));
}
// send a int16_t  (2 bytes)
void SerialCommsServer::send_i16(int16_t data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(int16_t)));
}
// send a int32_t  (4 bytes)
void SerialCommsServer::send_i32(int32_t data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(int32_t)));
}
// send a float    (4 bytes)
void SerialCommsServer::send_f32(float data)
{
  serialport.WriteBinary(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(float)));
}

/* functions to recieve data */

void SerialCommsServer::refill_read_buffer()
{
  if(!readBuffer.empty())
    return;

  std::vector<uint8_t> recv_buffer;
  serialport.ReadBinary(recv_buffer);
  if(!recv_buffer.empty())
    readBuffer.insert(readBuffer.end(), recv_buffer.begin(), recv_buffer.end());
}

int SerialCommsServer::recv_bytes(void *data, std::size_t size)
{
  if(size == 0)
    return 0;

  refill_read_buffer();
  if(readBuffer.size() < size)
    return 0;

  std::memcpy(data, readBuffer.data(), size);
  readBuffer.erase(readBuffer.begin(), readBuffer.begin() + static_cast<std::ptrdiff_t>(size));
  return static_cast<int>(size);
}


// receive a char (1 byte)
int SerialCommsServer::recv_char(char *data)
{
  return recv_bytes(data, sizeof(char));
}
// recieve a uint8_t (1 bytes)
int SerialCommsServer::recv_u8(uint8_t *data)
{
  return recv_bytes(data, sizeof(uint8_t));
}    
// recieve a uint16_t (2 bytes)
int SerialCommsServer::recv_u16(uint16_t *data)
{
  return recv_bytes(data, sizeof(uint16_t));
}
// recieve a uint32_t (4 bytes)
int SerialCommsServer::recv_u32(uint32_t *data)
{
  return recv_bytes(data, sizeof(uint32_t));
}
// recieve a int16_t  (2 bytes)
int SerialCommsServer::recv_i16(int16_t *data)
{
  return recv_bytes(data, sizeof(int16_t));
}
// recieve a int32_t  (4 bytes)
int SerialCommsServer::recv_i32(int32_t *data)
{
  return recv_bytes(data, sizeof(int32_t));
}
// recieve a float    (4 bytes)
int SerialCommsServer::recv_f32(float *data)
{
  return recv_bytes(data, sizeof(float));
}

void SerialCommsServer::send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)
{
  const std::vector<uint8_t> frame = encodePacket(seq, cmd, payload);
  serialport.WriteBinary(frame);
}

bool SerialCommsServer::recv_packet(DecodedPacket& packet)
{
  while(true)
  {
    if(tryDecodePacket(rxBuffer, packet))
      return true;

    uint8_t byte = 0;
    const int bytes_read = recv_u8(&byte);
    if(bytes_read <= 0)
      return false;
    rxBuffer.push_back(byte);
  }
}
