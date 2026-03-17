#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"
#include "serialCommsServer.hpp"
#include "logger.hpp"

#include <cstddef>
#include <cstring>

using namespace mn::CppLinuxSerial;


namespace {

void trim_rx_buffer(std::vector<uint8_t>& buffer, const std::size_t max_bytes)
{
  if (buffer.size() <= max_bytes)
    return;

  const std::size_t overflow = buffer.size() - max_bytes;
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(overflow));
  if (auto logger = logging::GetDefaultLogger()) {
    LOG_WARN(logger, "[SerialCommsServer] RX buffer overflow: dropped ", overflow, " bytes");
  }
}


} // namespace

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


void SerialCommsServer::refill_read_buffer()
{
  if (readBufferHead < readBuffer.size())
    return;

  if (!readBuffer.empty()) {
    readBuffer.clear();
    readBufferHead = 0;
  }

  std::vector<uint8_t> recv_buffer;
  serialport.ReadBinary(recv_buffer);
  if (recv_buffer.empty())
    return;

  readBuffer.insert(readBuffer.end(), recv_buffer.begin(), recv_buffer.end());
  trim_rx_buffer(readBuffer, MAX_TRANSPORT_RX_BUFFER_BYTES);
  if (readBufferHead > readBuffer.size())
    readBufferHead = readBuffer.size();
}

int SerialCommsServer::recv_bytes(void *data, std::size_t size)
{
  if (size == 0)
    return 0;

  refill_read_buffer();
  const std::size_t available = readBuffer.size() - readBufferHead;
  if (available < size)
    return 0;

  std::memcpy(data, readBuffer.data() + readBufferHead, size);
  readBufferHead += size;

  if (readBufferHead == readBuffer.size()) {
    readBuffer.clear();
    readBufferHead = 0;
  }

  return static_cast<int>(size);
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
    const int bytes_read = recv_bytes(&byte, sizeof(byte));
    if(bytes_read <= 0)
      return false;
    rxBuffer.push_back(byte);
    trim_rx_buffer(rxBuffer, MAX_TRANSPORT_RX_BUFFER_BYTES);
  }
}
