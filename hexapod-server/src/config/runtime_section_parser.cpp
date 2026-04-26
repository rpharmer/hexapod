#include "runtime_section_parser.hpp"

#include <algorithm>
#include <cmath>
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

template <typename T>
T findOrByPathOrDirect(const toml::value& root, const std::string& dotted_key, const T& default_value)
{
  if (hasValue(root, dotted_key)) {
    return findOrByPath<T>(root, dotted_key, default_value);
  }
  try {
    return toml::find<T>(root, dotted_key);
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
      {"Runtime.PhysicsSim.Host", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "127.0.0.1",
       false, 0.0},
      {"Runtime.PhysicsSim.Port", ValueType::Double, false, true, 1.0, 65535.0, "", false, 9871.0},
      {"Runtime.PhysicsSim.SolverIterations", ValueType::Double, false, true, 1.0, 512.0, "", false, 24.0},
      {"Runtime.Log.FilePath", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "app.log", false, 0.0},
      {"Runtime.Log.EnableFile", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", true, 0.0},
      {"Runtime.ReplayLog.EnableFile", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.ReplayLog.FilePath", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Telemetry.Enable", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Telemetry.Host", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "127.0.0.1", false, 0.0},
      {"Runtime.Telemetry.Port", ValueType::Double, false, true, 1.0, 65535.0, "", false, 9870.0},
      {"Runtime.Telemetry.PublishRateHz", ValueType::Double, false, true, 0.1, 1000.0, "", false, 30.0},
      {"Runtime.Telemetry.GeometryResendIntervalSec", ValueType::Double, false, true, 0.1, 3600.0, "", false, 1.0},
      {"Runtime.Telemetry.Enabled", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Telemetry.UdpHost", ValueType::String, false, false, kNoBoundsMin, kNoBoundsMax, "127.0.0.1", false, 0.0},
      {"Runtime.Telemetry.UdpPort", ValueType::Double, false, true, 1.0, 65535.0, "", false, 9870.0},
      {"Runtime.Telemetry.PublishPeriodMs", ValueType::Double, false, true, 1.0, 10000.0, "", false, 50.0},
      {"Runtime.Telemetry.GeometryRefreshPeriodMs", ValueType::Double, false, true, 100.0, 60000.0, "", false, 2000.0},
      {"Runtime.Investigation.DisableTerrainStanceBias", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.DisableTerrainSwingClearance", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.DisableTerrainSwingXYNudge", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.DisableStanceTiltLeveling", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.SuppressFusionCorrections", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.SuppressFusionResets", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.BypassTerrainSnapshotInRuntime", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.BypassNavStopOnStaleMap", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.BypassLocomotionFirstOrderFilter", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.BypassFreshnessGateReject", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
      {"Runtime.Investigation.BypassReachabilityClamp", ValueType::Bool, false, false, kNoBoundsMin, kNoBoundsMax, "", false, 0.0},
  };

  const auto* mode_desc = &schema[0];
  if (mode_desc->required && !hasValue(root, mode_desc->key)) {
    config_validation::emitDiagnostic(diagnostics, "runtime", mode_desc->key, "missing_required",
                                      "missing required key, using default");
  }
  std::string mode = findOrByPath<std::string>(root, mode_desc->key, mode_desc->default_string);
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode != "serial" && mode != "sim" && mode != "physics-sim") {
    const std::string message = "Runtime.Mode must be 'serial', 'sim', or 'physics-sim'";
    if (logger) {
      LOG_ERROR(logger, "[runtime] Runtime.Mode must be 'serial', 'sim', or 'physics-sim', got '", mode,
                "'");
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

  out.physicsSimHost = findOrByPath<std::string>(root, schema[7].key, schema[7].default_string);
  if (out.physicsSimHost.empty()) {
    out.physicsSimHost = schema[7].default_string;
  }
  out.physicsSimPort = static_cast<int>(config_validation::parseDoubleWithFallback(
      root, schema[8].key, schema[8].default_double, schema[8].min_value, schema[8].max_value, "runtime",
      logger, diagnostics));
  out.physicsSimSolverIterations = static_cast<int>(config_validation::parseDoubleWithFallback(
      root, schema[9].key, schema[9].default_double, schema[9].min_value, schema[9].max_value, "runtime",
      logger, diagnostics));

  out.logFilePath = findOrByPath<std::string>(root, schema[10].key, schema[10].default_string);
  out.logToFile = findOrByPath<bool>(root, schema[11].key, schema[11].default_bool);
  out.replayLogToFile = findOrByPath<bool>(root, schema[12].key, schema[12].default_bool);
  out.replayLogFilePath = findOrByPath<std::string>(root, schema[13].key, schema[13].default_string);
  out.telemetryEnabled = findOrByPath<bool>(root, schema[14].key, schema[14].default_bool);
  out.telemetryHost = findOrByPath<std::string>(root, schema[15].key, schema[15].default_string);
  if (out.telemetryHost.empty()) {
    out.telemetryHost = schema[15].default_string;
    config_validation::emitDiagnostic(diagnostics, "runtime", schema[15].key, "empty_value",
                                      "Runtime.Telemetry.Host was empty, using default 127.0.0.1");
    if (logger) {
      LOG_WARN(logger, "[runtime] Runtime.Telemetry.Host was empty, using default 127.0.0.1");
    }
  }
  out.telemetryPort = config_validation::parseIntWithFallback(
      root, schema[16].key, static_cast<int>(schema[16].default_double),
      static_cast<int>(schema[16].min_value), static_cast<int>(schema[16].max_value), "runtime",
      logger, diagnostics);
  out.telemetryPublishRateHz = config_validation::parseDoubleWithFallback(
      root, schema[17].key, schema[17].default_double, schema[17].min_value, schema[17].max_value,
      "runtime", logger, diagnostics);
  out.telemetryGeometryResendIntervalSec = config_validation::parseDoubleWithFallback(
      root, schema[18].key, schema[18].default_double, schema[18].min_value, schema[18].max_value,
      "runtime", logger, diagnostics);

  out.telemetryUdpHost = out.telemetryHost;
  out.telemetryUdpPort = out.telemetryPort;
  out.telemetryPublishPeriodMs =
      static_cast<int>(std::lround(1000.0 / std::max(out.telemetryPublishRateHz, 0.1)));
  out.telemetryGeometryRefreshPeriodMs =
      static_cast<int>(std::lround(out.telemetryGeometryResendIntervalSec * 1000.0));
  out.investigationDisableTerrainStanceBias =
      findOrByPathOrDirect<bool>(root, schema[24].key, schema[24].default_bool);
  out.investigationDisableTerrainSwingClearance =
      findOrByPathOrDirect<bool>(root, schema[25].key, schema[25].default_bool);
  out.investigationDisableTerrainSwingXYNudge =
      findOrByPathOrDirect<bool>(root, schema[26].key, schema[26].default_bool);
  out.investigationDisableStanceTiltLeveling =
      findOrByPathOrDirect<bool>(root, schema[27].key, schema[27].default_bool);
  out.investigationSuppressFusionCorrections =
      findOrByPathOrDirect<bool>(root, schema[28].key, schema[28].default_bool);
  out.investigationSuppressFusionResets =
      findOrByPathOrDirect<bool>(root, schema[29].key, schema[29].default_bool);
  out.investigationBypassTerrainSnapshotInRuntime =
      findOrByPathOrDirect<bool>(root, schema[30].key, schema[30].default_bool);
  out.investigationBypassNavStopOnStaleMap =
      findOrByPathOrDirect<bool>(root, schema[31].key, schema[31].default_bool);
  out.investigationBypassLocomotionFirstOrderFilter =
      findOrByPathOrDirect<bool>(root, schema[32].key, schema[32].default_bool);
  out.investigationBypassFreshnessGateReject =
      findOrByPathOrDirect<bool>(root, schema[33].key, schema[33].default_bool);
  out.investigationBypassReachabilityClamp =
      findOrByPathOrDirect<bool>(root, schema[34].key, schema[34].default_bool);
  return true;
}

} // namespace runtime_section_parser
