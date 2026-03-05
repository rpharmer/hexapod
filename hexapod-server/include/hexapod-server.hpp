// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"


struct ParsedToml
{
  std::string serialDevice{"/dev/ttyACM0"};
  int baudRate{115200};
  int timeout{100};
  std::vector<float> minMaxPulses{};
};

bool tomlParser(std::string filename,  ParsedToml& out);

#endif  // #ifndef HEXAPOD_SERVER_HPP