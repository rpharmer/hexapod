#include <toml.hpp>
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
  
  for(auto it = calibs.begin(); it!=calibs.end(); ++it)
  {
    uint16_t a = std::get<1>(*it);
    uint16_t b = std::get<2>(*it);
    printf("send a: %u,b: %u\n", a, b);
    scs.send_u16(a);
    scs.send_u16(b);
    uint16_t ca = 0;
    uint16_t cb = 0;
    scs.recv_u16(&ca);
    scs.recv_u16(&cb);
    printf("recv a: %u,b: %u\n", ca, cb);
  }
	// Close the serial port
	scs.Close();
}