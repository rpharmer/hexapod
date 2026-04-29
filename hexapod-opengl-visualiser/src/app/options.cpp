#include "visualiser/app/options.hpp"

#include <charconv>
#include <cstdlib>
#include <iostream>
#include <string>

namespace visualiser::app {

bool ParsePositiveInt(const char* text, int& out_value) {
  const char* end = text;
  while (*end != '\0') {
    ++end;
  }
  int value = 0;
  const auto result = std::from_chars(text, end, value);
  if (result.ec != std::errc{} || result.ptr != end || value <= 0) {
    return false;
  }
  out_value = value;
  return true;
}

bool ParseUdpPort(const char* text, int& out_value) {
  if (!ParsePositiveInt(text, out_value)) {
    return false;
  }
  return out_value <= 65535;
}

Options ParseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--udp-port") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --udp-port\n";
        std::exit(1);
      }
      const std::string value = argv[++i];
      if (!ParseUdpPort(value.c_str(), options.udp_port)) {
        std::cerr << "Invalid UDP port: " << value << "\n";
        std::exit(1);
      }
      continue;
    }
    if (arg == "--log-joint-positions") {
      options.log_joint_positions = true;
      continue;
    }
  }
  return options;
}

}  // namespace visualiser::app
