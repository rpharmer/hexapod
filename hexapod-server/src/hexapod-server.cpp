#include <toml.hpp>
#include <vector>
#include <chrono>
#include <thread>
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
    
    //std::string serialDevice = toml::find<std::string>(root, "SerialDevice");
    
    //int baudInt = toml::find<int>(root, "BaudRate");
    //BaudRate baudRate = SerialCommsServer::int_to_baud_rate(baudInt);
    
    //int timeout = toml::find<int>(root, "Timeout_ms");
    
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
}
