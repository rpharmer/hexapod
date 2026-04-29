#pragma once

namespace visualiser::app {

struct Options {
  int udp_port = 9870;
  bool log_joint_positions = false;
};

bool ParsePositiveInt(const char* text, int& out_value);
bool ParseUdpPort(const char* text, int& out_value);
Options ParseArgs(int argc, char** argv);

}  // namespace visualiser::app
