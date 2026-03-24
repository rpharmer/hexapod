#include "transport_section_parser.hpp"

#include "logger.hpp"

using namespace logging;

namespace transport_section_parser {

bool parseTransportSection(const toml::value& root, ParsedToml& out, bool required)
{
  std::string serialDevice = toml::find_or<std::string>(root, "SerialDevice", "");
  if (serialDevice.empty()) {
    if (!required) {
      serialDevice = out.serialDevice;
    } else {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "[transport] SerialDevice definition not found or empty");
      }
      return false;
    }
  }

  const int baudInt = toml::find_or<int>(root, "BaudRate", out.baudRate);
  if (baudInt <= 0 && required) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "[transport] BaudRate must be a positive number");
    }
    return false;
  }
  const int baudRate = baudInt > 0 ? baudInt : out.baudRate;

  const int timeout = toml::find_or<int>(root, "Timeout_ms", out.timeout);
  if (timeout <= 0 && required) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "[transport] Timeout_ms must be a positive number");
    }
    return false;
  }

  out.serialDevice = serialDevice;
  out.baudRate = baudRate;
  out.timeout = timeout > 0 ? timeout : out.timeout;
  return true;
}

} // namespace transport_section_parser
