#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"
#include "serialCommsServer.hpp"


using namespace mn::CppLinuxSerial;
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
  serialport.WriteBytes(&data, sizeof(char));
}
// send a uint8_t (1 bytes)
void SerialCommsServer::send_u8(uint8_t data)
{
  serialport.WriteBytes(&data, sizeof(uint8_t));
}
// send a uint16_t (2 bytes)
void SerialCommsServer::send_u16(uint16_t data)
{
  serialport.WriteBytes(&data, sizeof(uint16_t));
}
// send a uint32_t (4 bytes)
void SerialCommsServer::send_u32(uint32_t data)
{
  serialport.WriteBytes(&data, sizeof(uint32_t));
}
// send a int16_t  (2 bytes)
void SerialCommsServer::send_i16(int16_t data)
{
  serialport.WriteBytes(&data, sizeof(int16_t));
}
// send a int32_t  (4 bytes)
void SerialCommsServer::send_i32(int32_t data)
{
  serialport.WriteBytes(&data, sizeof(int32_t));
}
// send a float    (4 bytes)
void SerialCommsServer::send_f32(float data)
{
  serialport.WriteBytes(&data, sizeof(float));
}

/* functions to recieve data */

// receive a char (1 byte)
int SerialCommsServer::recv_char(char *data)
{
  return serialport.ReadBytes(data, sizeof(char));
}
// recieve a uint8_t (1 bytes)
int SerialCommsServer::recv_u8(uint8_t *data)
{
  return serialport.ReadBytes(data, sizeof(uint8_t));
}    
// recieve a uint16_t (2 bytes)
int SerialCommsServer::recv_u16(uint16_t *data)
{
  return serialport.ReadBytes(data, sizeof(uint16_t));
}
// recieve a uint32_t (4 bytes)
int SerialCommsServer::recv_u32(uint32_t *data)
{
  return serialport.ReadBytes(data, sizeof(uint32_t));
}
// recieve a int16_t  (2 bytes)
int SerialCommsServer::recv_i16(int16_t *data)
{
  return serialport.ReadBytes(data, sizeof(int16_t));
}
// recieve a int32_t  (4 bytes)
int SerialCommsServer::recv_i32(int32_t *data)
{
  return serialport.ReadBytes(data, sizeof(int32_t));
}
// recieve a float    (4 bytes)
int SerialCommsServer::recv_f32(float *data)
{
  return serialport.ReadBytes(data, sizeof(float));
}

void SerialCommsServer::send_packet(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)
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
    if(recv_u8(&byte) < 0)
      return false;
    rxBuffer.push_back(byte);
  }
}
