#include "runtime_section_parser.hpp"

#include <algorithm>
#include <cctype>
#include <vector>

#include "config_validation.hpp"
#include "logger.hpp"

using namespace logging;

namespace runtime_section_parser {

namespace {

enum class ValueType
{
  String,
  Bool,
  Double,
};

struct ScalarDescriptor
{
  std::string key;
  ValueType type;
  bool required;
  bool has_bounds;
  double min_value;
  double max_value;
  std::string default_string;
  bool default_bool;
  double default_double;
};

constexpr double kNoBoundsMin = 0.0;
constexpr double kNoBoundsMax = 0.0;

bool hasValue(const toml::value& root, const std::string& dotted_key)
{
  const toml::value* current = &root;
  std::size_t start = 0;
  while (start <= dotted_key.size()) {
    const std::size_t dot = dotted_key.find('.', start);
    const std::string key =
        dot == std::string::npos ? dotted_key.substr(start) : dotted_key.substr(start, dot - start);
    if (!current->is_table()) {
      return false;
    }
    const auto& table = current->as_table();
    const auto it = table.find(key);
    if (it == table.end()) {
      return false;
    }
    current = &(it->second);
    if (dot == std::string::npos) {
      break;
    }
    start = dot + 1;
  }
  return true;
}

template <typename T>
T findOrByPath(const toml::value& root, const std::string& dotted_key, const T& default_value)
{
  const toml::value* current = &root;
  std::size_t start = 0;
  while (start <= dotted_key.size()) {
    const std::size_t dot = dotted_key.find('.', start);
    const std::string key =
        dot == std::string::npos ? dotted_key.substr(start) : dotted_key.substr(start, dot - start);
    if (!current->is_table()) {
      return default_value;
    }
    const auto& table = current->as_table();
    const auto it = table.find(key);
    if (it == table.end()) {
      return default_value;
    }
    current = &(it->second);
    if (dot == std::string::npos) {
      break;
    }
    start = dot + 1;
  }

  try {
    return toml::get<T>(*current);
  } catch (const std::exception&) {
    return default_value;
  }
}

} // namespace

bool parseRuntimeSection(const toml::value& root,
                         ParsedToml& out,
                         std::shared_ptr<logging::AsyncLogger> logger,
                         std::vector<config_validation::ConfigDiagnostic>* diagnostics)
{
  const std::vector<ScalarDescriptor> schema = {
      {"Runtime.Mode", ValueType::String, true, false, kNoBoundsMin, kNoBoundsMax, "serial", false, 0.0},
      {"Runtime.Sim.InitialVoltageV", ValueType::Double, false, true, 0.0, 32.0, "", false, 12.0},
      {"Runtime.Sim.InitialCurrentA", ValueType::Double, false, true, 0.0, 200.0, "", false, 1.0},
      {"Runtime.Sim.ResponseRateHz", ValueType::Double, false, true, 1.0, 2000.0, "", false, 50.0},
      {"Runtime.Sim.DropBus", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Sim.LowVoltage", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Sim.HighCurrent", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Log.FilePath", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "app.log", false, 0.0},
      {"Runtime.Log.EnableFile", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", true, 0.0},
      {"Runtime.Telemetry.Enabled", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Telemetry.UdpHost", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "127.0.0.1", false, 0.0},
      {"Runtime.Telemetry.UdpPort", ValueType::Double, false, true, 1.0, 65535.0, "", false, 9870.0},
      {"Runtime.Telemetry.PublishPeriodMs", ValueType::Double, false, true, 1.0, 10000.0, "", false, 50.0},
      {"Runtime.Telemetry.GeometryRefreshPeriodMs", ValueType::Double, false, true, 100.0, 60000.0, "", false, 2000.0},
  };

  const auto* mode_desc = &schema[0];
  if (mode_desc->required && !hasValue(root, mode_desc->key)) {
    config_validation::emitDiagnostic(diagnostics, "runtime", mode_desc->key, "missing_required",
                                      "missing required key, using default");
  }
  std::string mode = findOrByPath<std::string>(root, mode_desc->key, mode_desc->default_string);
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode != "serial" && mode != "sim") {
    const std::string message = "Runtime.Mode must be 'serial' or 'sim'";
    if (logger) {
      LOG_ERROR(logger, "[runtime] Runtime.Mode must be 'serial' or 'sim', got '", mode, "'");
    }
    config_validation::emitDiagnostic(
        diagnostics, "runtime", "Runtime.Mode", "invalid_enum",
        message + ", got '" + mode + "'");
    return false;
  }
  out.runtimeMode = mode;

  const auto* initial_voltage_desc = &schema[1];
  out.simInitialVoltageV = config_validation::parseDoubleWithFallback(
      root, initial_voltage_desc->key, initial_voltage_desc->default_double,
      initial_voltage_desc->min_value, initial_voltage_desc->max_value, "runtime", logger,
      diagnostics);
  const auto* initial_current_desc = &schema[2];
  out.simInitialCurrentA = config_validation::parseDoubleWithFallback(
      root, initial_current_desc->key, initial_current_desc->default_double,
      initial_current_desc->min_value, initial_current_desc->max_value, "runtime", logger,
      diagnostics);
  const auto* response_rate_desc = &schema[3];
  out.simResponseRateHz = config_validation::parseDoubleWithFallback(
      root, response_rate_desc->key, response_rate_desc->default_double,
      response_rate_desc->min_value, response_rate_desc->max_value, "runtime", logger,
      diagnostics);
  out.simDropBus = findOrByPath<bool>(root, schema[4].key, schema[4].default_bool);
  out.simLowVoltage = findOrByPath<bool>(root, schema[5].key, schema[5].default_bool);
  out.simHighCurrent = findOrByPath<bool>(root, schema[6].key, schema[6].default_bool);

  out.logFilePath = findOrByPath<std::string>(root, schema[7].key, schema[7].default_string);
  out.logToFile = findOrByPath<bool>(root, schema[8].key, schema[8].default_bool);
  out.telemetryEnabled = findOrByPath<bool>(root, schema[9].key, schema[9].default_bool);
  out.telemetryUdpHost = findOrByPath<std::string>(root, schema[10].key, schema[10].default_string);
  out.telemetryUdpPort = static_cast<int>(config_validation::parseDoubleWithFallback(
      root, schema[11].key, schema[11].default_double, schema[11].min_value, schema[11].max_value,
      "runtime", logger, diagnostics));
  out.telemetryPublishPeriodMs = static_cast<int>(config_validation::parseDoubleWithFallback(
      root, schema[12].key, schema[12].default_double, schema[12].min_value, schema[12].max_value,
      "runtime", logger, diagnostics));
  out.telemetryGeometryRefreshPeriodMs = static_cast<int>(config_validation::parseDoubleWithFallback(
      root, schema[13].key, schema[13].default_double, schema[13].min_value, schema[13].max_value,
      "runtime", logger, diagnostics));

  if (out.logToFile && out.logFilePath.empty()) {
    out.logFilePath = "app.log";
    config_validation::emitDiagnostic(diagnostics, "runtime", "Runtime.Log.FilePath",
                                      "empty_value",
                                      "Runtime.Log.FilePath was empty, using default app.log");
    if (logger) {
      LOG_WARN(logger, "[runtime] Runtime.Log.FilePath was empty, using default app.log");
    }
  }
  return true;
}

} // namespace runtime_section_parser
