#include "calibrations_section_parser.hpp"

#include "logger.hpp"

using namespace logging;

namespace calibrations_section_parser {

bool parseCalibrationsSection(const toml::value& root, std::vector<CalibrationRow>& out)
{
  out = toml::find_or<std::vector<CalibrationRow>>(root, "MotorCalibrations", {});
  if (out.empty()) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "[calibrations] MotorCalibrations wasn't valid or not found");
    }
    return false;
  }
  return true;
}

} // namespace calibrations_section_parser
