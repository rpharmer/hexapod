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

  int busLoopPeriodUs{2000};
  int estimatorLoopPeriodUs{2000};
  int controlLoopPeriodUs{4000};
  int safetyLoopPeriodUs{2000};
  int diagnosticsPeriodMs{500};
  int commandRefreshPeriodMs{100};
  int standSettlingDelayMs{2000};
  double maxTiltRad{0.70};
  uint64_t commandTimeoutUs{300000};
  double fallbackSpeedMag{0.01};
};

bool tomlParser(std::string filename,  ParsedToml& out);

#endif  // #ifndef HEXAPOD_SERVER_HPP
