#include "runtime_section_parser.hpp"

#include <algorithm>
#include <cctype>

#include "config_validation.hpp"
#include "logger.hpp"

using namespace logging;

namespace runtime_section_parser {

bool parseRuntimeSection(const toml::value& root,
                       ParsedToml& out,
                       std::shared_ptr<logging::AsyncLogger> logger)
{
  std::string mode = toml::find_or<std::string>(root, "Runtime", "Mode", "serial");
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode != "serial" && mode != "sim") {
    if (logger) {
      LOG_ERROR(logger, "[runtime] Runtime.Mode must be 'serial' or 'sim', got '", mode, "'");
    }
    return false;
  }
  out.runtimeMode = mode;

  out.simInitialVoltageV = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.InitialVoltageV", 12.0, 0.0, 32.0, "runtime", logger);
  out.simInitialCurrentA = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.InitialCurrentA", 1.0, 0.0, 200.0, "runtime", logger);
  out.simResponseRateHz = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.ResponseRateHz", 50.0, 1.0, 2000.0, "runtime", logger);
  out.simDropBus = toml::find_or<bool>(root, "Runtime", "Sim", "DropBus", false);
  out.simLowVoltage = toml::find_or<bool>(root, "Runtime", "Sim", "LowVoltage", false);
  out.simHighCurrent = toml::find_or<bool>(root, "Runtime", "Sim", "HighCurrent", false);

  out.logFilePath = toml::find_or<std::string>(root, "Runtime", "Log", "FilePath", "app.log");
  out.logToFile = toml::find_or<bool>(root, "Runtime", "Log", "EnableFile", true);
  if (out.logToFile && out.logFilePath.empty()) {
    out.logFilePath = "app.log";
    if (logger) {
      LOG_WARN(logger, "[runtime] Runtime.Log.FilePath was empty, using default app.log");
    }
  }
  return true;
}

} // namespace runtime_section_parser
