#include <toml.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "serialCommsServer.hpp"


using namespace mn::CppLinuxSerial;

int main() {
  ParsedToml config;
  tomlParser("config.txt", config);
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  // select TOML version at runtime (optional)
  auto root = toml::parse(filename, toml::spec::v(1,1,0));
  
  // Check that the file is a Hexapod Config File
  if(root.at("title").as_string() != "Hexapod Config File")
  {
    printf("incorrect config header. expected \"Hexapod Config File\"\n");
    return false;
  }
  
  std::string serialDevice = toml::find_or<std::string>(root, "SerialDevice", "Error");
  if(serialDevice == "Error")
  {
    printf("SerialDevice definition not found\n");
    return false;
  }
  
  
  int baudInt = toml::find_or<int>(root, "BaudRate", -1);
  if(baudInt == -1)
  {
    printf("baudRate wasn't a positive valid number or not found\n");
    return false;
  }
  BaudRate baudRate = SerialCommsServer::int_to_baud_rate(baudInt);
  
  int timeout = toml::find_or<int>(root, "Timeout_ms", -1);
  if(timeout == -1)
  {
    printf("timeout wasn't a positive valid number or not found\n");
    return false;
  }
  
  auto calibs = toml::find_or<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations", {});
  if(calibs.size() == 0)
  {
    printf("MotorCalibrations wasn't valid or not found\n");
    return false;
  }
  if(calibs.size() != 18)
  {
    printf("invalid number of MotorCalibrations, expected 18\n");
    return false;
  }
  
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
  
  std::vector<float> calibsF;
  calibsF.reserve(36 * sizeof(float));
  
  int index = 0;
  for(auto it = calibs.begin(); it!=calibs.end(); ++it)
  {
    calibsF[index++] = std::get<1>(*it); // min pulse value
    calibsF[index++] = std::get<2>(*it); // max pulse value
  }
  
  out.serialDevice = serialDevice;
  out.baudRate = baudRate;
  out.timeout = timeout;
  out.minMaxPulses = calibsF;
  
  return true;
}
