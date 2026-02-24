#include <toml.hpp>
#include <vector>
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "serialCommsServer.hpp"


using namespace mn::CppLinuxSerial;

int main() {
  
  ///** Test toml **///

    // select TOML version at runtime (optional)
    auto root = toml::parse("config.txt", toml::spec::v(1,1,0));
    
    // Check that the file is a Hexapod Config File
    assert(root.at("title").as_string() == "Hexapod Config File");
    
    auto calibs = toml::find<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations");
    
    // sort calibration data to correct motor order
    sort(calibs.begin(),calibs.end(),
    [](const std::tuple<std::string, int, int>& a,
       const std::tuple<std::string, int, int>& b) -> bool
       {
         std::string _a = std::get<0>(a);
         std::string _b = std::get<0>(b);
         
         // reorder the characters so they can be compared
         //{R|L} {1|2|3} {1|2|3}
         // 2nd    1st     3rd
         //  +      +       -
         std::string _a_ = {_a[1], _a[0], static_cast<char>(-_a[2])};
         std::string _b_ = {_b[1], _b[0], static_cast<char>(-_b[2])};
         
         return _a_ > _b_;
       });
    
    for(auto it = calibs.begin(); it!=calibs.end(); ++it)
    {
      std::cout<<
        std::get<0>(*it)<<", "<<std::get<1>(*it)<<", "<<std::get<2>(*it)<<std::endl;
    }

  ///** End toml test **///
  
	// Create serial port object and open serial port at 115200 baud, 8 data bits, no parity bit, one stop bit (8n1),
	// and no flow control
  SerialCommsServer scs("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  
	scs.SetTimeout(100); // Block for up to 100ms to receive data
	scs.Open();

  /* Run tests
	// Write some ASCII data
	//serialPort.Write("Hello");
  
  // Test read write functions
  char data_c = 'K';
  char data_ret = 0;
  
  
  // send char 'K'
  scs.send_char(data_c);
  printf("sent char: %d\n", data_c);
  
  // receive char
  scs.recv_char(&data_ret);
  printf("received char: %d\n", data_ret);
  
  // send char 'E'
  data_c = 'E';
  scs.send_char(data_c);
  printf("sent char: %d\n", data_c);
  
  // receive char
  scs.recv_char(&data_ret);
  printf("received char: %d\n", data_ret);
  
  uint8_t data_c2 = 120;
  uint8_t data_ret2 = 0;
  
  
  // send uint8_t 'K'
  scs.send_u8(data_c2);
  printf("sent uint8_t: %u\n", data_c2);
  
  // receive uint8_t
  scs.recv_u8(&data_ret2);
  printf("received uint8_t: %u\n", data_ret2);
  
  // send uint8_t 'H'
  data_c2 = 130;
  scs.send_u8(data_c2);
  printf("sent uint8_t: %u\n", data_c2);
  
  // receive uint8_t
  scs.recv_u8(&data_ret2);
  printf("received uint8_t: %u\n", data_ret2);
  
  scs.send_u8(SET_ANGLE_CALIBRATIONS);
  uint8_t c = 0;
  scs.recv_u8(&c);
  printf("Z: %u\n", c);
  c = 38;
  
  
  scs.send_u16(500);
  uint16_t cc = 0;
  scs.recv_u16(&cc);
  printf("ZZ: %u\n", cc);
  
  std::cout<<std::endl;
  std::ios_base::fmtflags f( std::cout.flags() );  // save flags state
  std::cout << std::hex << "0x" << (int)c << std::endl;
  std::cout.flags( f );  // restore flags state */
  
  bool handshakeSuccess = false;
  
  for(int attempt = 0; attempt < 3 && !handshakeSuccess; attempt++)
  {
    handshakeSuccess = do_handshake(scs, 0);
  }
  
  if(!handshakeSuccess)
  {
    printf("couldn't etablish handshake, closing\n");
    return -1;
  }
  else
  {
    printf("handshake successful\n");
  }
  
  std::vector<uint8_t> calibPayload;
  calibPayload.reserve(calibs.size() * 4);
  for(const auto& calib : calibs)
  {
    const uint16_t minPulse = static_cast<uint16_t>(std::get<1>(calib));
    const uint16_t maxPulse = static_cast<uint16_t>(std::get<2>(calib));
    calibPayload.push_back(static_cast<uint8_t>(minPulse & 0xFF));
    calibPayload.push_back(static_cast<uint8_t>((minPulse >> 8) & 0xFF));
    calibPayload.push_back(static_cast<uint8_t>(maxPulse & 0xFF));
    calibPayload.push_back(static_cast<uint8_t>((maxPulse >> 8) & 0xFF));
  }

  scs.send_packet(SET_ANGLE_CALIBRATIONS, calibPayload);

  DecodedPacket response;
  if(scs.recv_packet(response) && response.cmd == ACK)
    printf("calibration packet accepted\n");
  else
    printf("calibration packet failed\n");
  
	// Close the serial port
	scs.Close();
}


bool do_handshake(SerialCommsServer& sc, uint8_t requested_caps)
{
  sc.send_packet(HELLO, {PROTOCOL_VERSION, requested_caps});

  DecodedPacket response;
  if(!sc.recv_packet(response))
  {
    printf("handshake timeout waiting for response\n");
    return false;
  }

  if(response.cmd == NACK)
  {
    if(response.payload.empty())
    {
      printf("NACK without error code\n");
      return false;
    }
    uint8_t errorCode = response.payload[0];
    printf("NACK, Error: %u\n", errorCode);
    return false;
  }
  
  else if(response.cmd == ACK)
  {
    if(response.payload.size() < 3)
    {
      printf("malformed ACK response\n");
      return false;
    }
    const uint8_t version = response.payload[0];
    const uint8_t status = response.payload[1];
    const uint8_t deviceID = response.payload[2];
    (void)deviceID;
    
    if(version != PROTOCOL_VERSION)
    {
      printf("version mismatch\n");
      return false;
    }
    if(status != STATUS_OK)
    {
      printf("device not ready\n");
      return false;
    }
    return true;
  }
  else
  {
    printf("malformed response, recieved: %u\n", response.cmd);
    return false;
  }
  
  return true;
}

