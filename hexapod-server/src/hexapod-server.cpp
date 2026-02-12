#include <CppLinuxSerial/SerialPort.hpp>
#include <toml.hpp>
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"


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
	SerialPort serialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.SetTimeout(100); // Block for up to 100ms to receive data
	serialPort.Open();

	// Write some ASCII data
	//serialPort.Write("Hello");
  
  // Write some chars 
  /*serialPort.WriteChar('H');
  serialPort.WriteChar('e');
  serialPort.WriteChar('l');
  serialPort.WriteChar('1');
  serialPort.WriteChar('o');*/

	// Read some data back (will block for up to 100ms due to the SetTimeout(100) call above)
	/*std::string readData;
  while(1){
	serialPort.Read(readData);
	std::cout << "Read data = \"" << readData << "\"" << std::endl;}*/
  
  /*int c = serialPort.ReadChar();
  while(c > -1)
  {
    std::cout<<(char)c;
    c = serialPort.ReadChar();
  }*/

  // Send motor calibration data to hexapod
  /*
  int testValue = 1502;
  serialPort.WriteChar('C'); // 'C' for calibration
  char b0 = get0_7Lsbits(testValue);
  char b1 = get8_15Lsbits(testValue);
  serialPort.WriteChar(b0);
  serialPort.WriteChar(b1);
  
  int c = serialPort.ReadChar();
  std::cout<<(char)c;
  
  char c0 = serialPort.ReadChar();
  char c1 = serialPort.ReadChar();
  
  int returnedValue = getInt(0, 0, c1, c0);
  */
  serialPort.WriteChar(SET_ANGLE_CALIBRATIONS);
  int c = serialPort.ReadChar();
  
  std::cout<<std::endl;
  std::ios_base::fmtflags f( std::cout.flags() );  // save flags state
  std::cout << std::hex << "0x" << c << std::endl;
  std::cout.flags( f );  // restore flags state
  
  for(auto it = calibs.begin(); it!=calibs.end(); ++it)
  {
    serialPort.WriteChar(MSB0(std::get<1>(*it)));
    serialPort.WriteChar(MSB1(std::get<1>(*it)));
    
    serialPort.WriteChar(MSB0(std::get<2>(*it)));
    serialPort.WriteChar(MSB1(std::get<2>(*it)));
    
    char c00 = serialPort.ReadChar();
    char c01 = serialPort.ReadChar();
      
    char c10 = serialPort.ReadChar();
    char c11 = serialPort.ReadChar();
    
    //std::cout<<(int)c00<<", "<<(int)c01<<";  "<<(int)c10<<", "<<(int)c11<<std::endl;
    std::cout<<GETINT(0,0,c01,c00)<<", "<<GETINT(0,0,c11,c10)<<std::endl;
  }

	// Close the serial port
	serialPort.Close();
}