#pragma once

#include <string>

namespace visualiser::app {

struct Options {
  int udp_port = 9870;
  bool log_joint_positions = false;
  std::string command_host = "127.0.0.1";
  int command_port = 9872;
};

bool ParsePositiveInt(const char* text, int& out_value);
bool ParseUdpPort(const char* text, int& out_value);
Options ParseArgs(int argc, char** argv);

}  // namespace visualiser::app
