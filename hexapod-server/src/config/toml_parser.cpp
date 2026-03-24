#include "toml_parser.hpp"

#include <utility>
#include <vector>

#include "calibrations_section_parser.hpp"
#include "geometry_section_parser.hpp"
#include "logger.hpp"
#include "motor_calibration_validator.hpp"
#include "runtime_section_parser.hpp"
#include "transport_section_parser.hpp"
#include "tuning_section_parser.hpp"

using namespace logging;

namespace {

constexpr const char* kExpectedConfigTitle = "Hexapod Config File";
constexpr const char* kExpectedConfigSchema = "hexapod.server.config";
constexpr int kExpectedConfigSchemaVersion = 1;

} // namespace

TomlParser::TomlParser(std::shared_ptr<logging::AsyncLogger> logger)
    : logger_(std::move(logger)) {}

bool TomlParser::parse(const std::string& filename, ParsedToml& out) const
{
  try {
    const auto root = toml::parse(filename, toml::spec::v(1, 1, 0));

    bool ok = true;
    ok = parseSchemaHeaderSection(root) && ok;
    ok = runtime_section_parser::parseRuntimeSection(root, out, logger_) && ok;
    ok = transport_section_parser::parseTransportSection(root, out, out.runtimeMode == "serial", logger_) && ok;

    std::vector<calibrations_section_parser::CalibrationRow> raw_calibrations;
    ok = calibrations_section_parser::parseCalibrationsSection(root, raw_calibrations, logger_) && ok;
    if (!raw_calibrations.empty()) {
      ok = motor_calibration_validator::validateAndNormalize(
               raw_calibrations, out.minMaxPulses, "calibrations", logger_) &&
           ok;
    }

    tuning_section_parser::parseTuningSection(root, out, logger_);
    geometry_section_parser::parseGeometrySection(root, out, logger_);

    return ok;
  } catch (const std::exception& ex) {
    if (logger_) {
      LOG_ERROR(logger_, "failed to parse '", filename, "': ", ex.what());
    }
    return false;
  }
}

bool TomlParser::parseSchemaHeaderSection(const toml::value& root) const
{
  const std::string title = toml::find_or<std::string>(root, "title", "");
  if (title != kExpectedConfigTitle) {
    if (logger_) {
      LOG_ERROR(logger_, "[schema] incorrect config header. expected '", kExpectedConfigTitle,
                "', got '", title, "'");
    }
    return false;
  }

  const std::string schema = toml::find_or<std::string>(root, "Schema", "");
  if (schema != kExpectedConfigSchema) {
    if (logger_) {
      LOG_ERROR(logger_, "[schema] invalid Schema '", schema, "'. expected '",
                kExpectedConfigSchema, "'");
    }
    return false;
  }

  const int schema_version = toml::find_or<int>(root, "SchemaVersion", -1);
  if (schema_version != kExpectedConfigSchemaVersion) {
    if (logger_) {
      LOG_ERROR(logger_, "[schema] unsupported SchemaVersion=", schema_version,
                ". expected ", kExpectedConfigSchemaVersion);
    }
    return false;
  }

  return true;
}
