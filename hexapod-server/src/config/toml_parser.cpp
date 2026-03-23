#include "toml_parser.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <set>
#include <tuple>
#include <vector>

#include "config_validation.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"
#include "logger.hpp"

using namespace logging;

namespace {

constexpr const char* kExpectedConfigTitle = "Hexapod Config File";
constexpr const char* kExpectedConfigSchema = "hexapod.server.config";
constexpr int kExpectedConfigSchemaVersion = 1;
constexpr int kExpectedJointCount = static_cast<int>(kProtocolJointCount);
constexpr int kMinServoPulse = 500;
constexpr int kMaxServoPulse = 2500;
const std::array<std::string, kExpectedJointCount> kExpectedJointOrder = {
    "R31", "R32", "R33", "L31", "L32", "L33",
    "R21", "R22", "R23", "L21", "L22", "L23",
    "R11", "R12", "R13", "L11", "L12", "L13"};

bool isCalibrationKeyValid(const std::string& key)
{
  return key.size() == 3 && (key[0] == 'R' || key[0] == 'L') && (key[1] >= '1' && key[1] <= '3') &&
         (key[2] >= '1' && key[2] <= '3');
}

} // namespace

bool TomlParser::parse(const std::string& filename, ParsedToml& out) const
{
  try {
    auto root = toml::parse(filename, toml::spec::v(1, 1, 0));
    if (!parseSchemaHeaderSection(root)) {
      return false;
    }
    if (!parseRuntimeSection(root, out)) {
      return false;
    }
    if (!parseTransportSection(root, out, out.runtimeMode == "serial")) {
      return false;
    }
    if (!parseCalibrationsSection(root, out)) {
      return false;
    }
    parseTuningSection(root, out);
    parseGeometrySection(root, out);

    return true;
  } catch (const std::exception& ex) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "failed to parse '", filename, "': ", ex.what());
    }
    return false;
  }
}

bool TomlParser::parseSchemaHeaderSection(const toml::value& root) const
{
  if (root.at("title").as_string() != kExpectedConfigTitle) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "incorrect config header. expected '", kExpectedConfigTitle, "'");
    }
    return false;
  }

  const std::string schema = toml::find_or<std::string>(root, "Schema", "");
  if (schema != kExpectedConfigSchema) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "invalid Schema '", schema, "'. expected '", kExpectedConfigSchema, "'");
    }
    return false;
  }

  const int schema_version = toml::find_or<int>(root, "SchemaVersion", -1);
  if (schema_version != kExpectedConfigSchemaVersion) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "unsupported SchemaVersion=", schema_version,
                ". expected ", kExpectedConfigSchemaVersion);
    }
    return false;
  }

  return true;
}

bool TomlParser::parseRuntimeSection(const toml::value& root, ParsedToml& out) const
{
  std::string mode = toml::find_or<std::string>(root, "Runtime", "Mode", "serial");
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode != "serial" && mode != "sim") {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "Runtime.Mode must be 'serial' or 'sim', got '", mode, "'");
    }
    return false;
  }
  out.runtimeMode = mode;

  out.simInitialVoltageV = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.InitialVoltageV", 12.0, 0.0, 32.0, "runtime");
  out.simInitialCurrentA = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.InitialCurrentA", 1.0, 0.0, 200.0, "runtime");
  out.simResponseRateHz = config_validation::parseDoubleWithFallback(
      root, "Runtime.Sim.ResponseRateHz", 50.0, 1.0, 2000.0, "runtime");
  out.simDropBus = toml::find_or<bool>(root, "Runtime", "Sim", "DropBus", false);
  out.simLowVoltage = toml::find_or<bool>(root, "Runtime", "Sim", "LowVoltage", false);
  out.simHighCurrent = toml::find_or<bool>(root, "Runtime", "Sim", "HighCurrent", false);
  return true;
}

bool TomlParser::parseTransportSection(const toml::value& root, ParsedToml& out, bool required) const
{
  std::string serialDevice = toml::find_or<std::string>(root, "SerialDevice", "");
  if (serialDevice.empty()) {
    if (!required) {
      serialDevice = out.serialDevice;
    } else {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "SerialDevice definition not found or empty");
      }
      return false;
    }
  }

  const int baudInt = toml::find_or<int>(root, "BaudRate", out.baudRate);
  if (baudInt <= 0 && required) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "baudRate wasn't a positive valid number or not found");
    }
    return false;
  }
  const int baudRate = baudInt > 0 ? baudInt : out.baudRate;

  const int timeout = toml::find_or<int>(root, "Timeout_ms", out.timeout);
  if (timeout <= 0 && required) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "timeout wasn't a positive valid number or not found");
    }
    return false;
  }

  out.serialDevice = serialDevice;
  out.baudRate = baudRate;
  out.timeout = timeout > 0 ? timeout : out.timeout;
  return true;
}

bool TomlParser::parseCalibrationsSection(const toml::value& root, ParsedToml& out) const
{
  auto calibs =
      toml::find_or<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations", {});
  if (calibs.empty()) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "MotorCalibrations wasn't valid or not found");
    }
    return false;
  }
  if (calibs.size() != kExpectedJointCount) {
    if (auto logger = GetDefaultLogger()) {
      LOG_ERROR(logger, "invalid number of MotorCalibrations, expected ", kExpectedJointCount);
    }
    return false;
  }

  std::set<std::string> seen_keys;
  std::set<std::string> expected_keys(kExpectedJointOrder.begin(), kExpectedJointOrder.end());

  for (const auto& calib : calibs) {
    const auto& key = std::get<0>(calib);
    const int min_pulse = std::get<1>(calib);
    const int max_pulse = std::get<2>(calib);

    if (!isCalibrationKeyValid(key)) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "invalid motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (!expected_keys.contains(key)) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "unexpected motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (!seen_keys.insert(key).second) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "duplicate motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (min_pulse >= max_pulse) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "invalid pulse bounds for '", key, "': min must be < max");
      }
      return false;
    }
    if (min_pulse < kMinServoPulse || max_pulse > kMaxServoPulse) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "invalid pulse bounds for '", key, "': must be within [", kMinServoPulse,
                  ", ", kMaxServoPulse, "]");
      }
      return false;
    }
  }

  for (const auto& expected_key : expected_keys) {
    if (!seen_keys.contains(expected_key)) {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "missing motor key '", expected_key, "' in MotorCalibrations");
      }
      return false;
    }
  }

  std::sort(calibs.begin(), calibs.end(),
            [](const std::tuple<std::string, int, int>& a,
               const std::tuple<std::string, int, int>& b) -> bool {
              const std::string& key_a = std::get<0>(a);
              const std::string& key_b = std::get<0>(b);

              const std::array<int, 3> sort_key_a = {key_a[1] - '0', key_a[0] == 'R' ? 1 : 0,
                                                     3 - (key_a[2] - '0')};
              const std::array<int, 3> sort_key_b = {key_b[1] - '0', key_b[0] == 'R' ? 1 : 0,
                                                     3 - (key_b[2] - '0')};

              return sort_key_a > sort_key_b;
            });

  std::vector<float> calibsF;
  calibsF.reserve(kProtocolJointCount * kProtocolCalibrationPairsPerJoint);

  for (const auto& calib : calibs) {
    calibsF.push_back(static_cast<float>(std::get<1>(calib)));
    calibsF.push_back(static_cast<float>(std::get<2>(calib)));
  }

  out.minMaxPulses = std::move(calibsF);
  return true;
}

void TomlParser::parseTuningSection(const toml::value& root, ParsedToml& out) const
{
  out.busLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.BusLoopPeriodUs", control_config::kDefaultBusLoopPeriodUs, 500, 50000, "tuning");
  out.estimatorLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.EstimatorLoopPeriodUs", control_config::kDefaultEstimatorLoopPeriodUs,
      500, 50000, "tuning");
  out.controlLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.ControlLoopPeriodUs", control_config::kDefaultControlLoopPeriodUs, 500, 50000,
      "tuning");
  out.safetyLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.SafetyLoopPeriodUs", control_config::kDefaultSafetyLoopPeriodUs, 500, 50000,
      "tuning");
  out.diagnosticsPeriodMs = config_validation::parseIntWithFallback(
      root, "Tuning.DiagnosticsPeriodMs", control_config::kDefaultDiagnosticsPeriodMs, 100, 10000,
      "tuning");
  out.commandRefreshPeriodMs = config_validation::parseIntWithFallback(
      root, "Tuning.CommandRefreshPeriodMs", control_config::kDefaultCommandRefreshPeriodMs, 10,
      1000, "tuning");
  out.standSettlingDelayMs = config_validation::parseIntWithFallback(
      root, "Tuning.StandSettlingDelayMs", control_config::kDefaultStandSettlingDelayMs, 0, 10000,
      "tuning");
  out.maxTiltRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.MaxTiltRad", control_config::kDefaultMaxTiltRad.value, 0.1, 1.5, "tuning");
  out.commandTimeoutUs = config_validation::parseU64WithFallback(
      root, "Tuning.CommandTimeoutUs", control_config::kDefaultCommandTimeoutUs.value, 10000,
      2000000, "tuning");
  out.estimatorMaxAgeUs = config_validation::parseU64WithFallback(
      root, "Tuning.EstimatorMaxAgeUs", control_config::kDefaultEstimatorMaxAgeUs.value, 1000,
      2000000, "tuning");
  out.intentMaxAgeUs = config_validation::parseU64WithFallback(
      root, "Tuning.IntentMaxAgeUs", control_config::kDefaultIntentMaxAgeUs.value, 1000, 2000000,
      "tuning");
  out.estimatorRequireTimestamp =
      config_validation::parseBoolWithFallback(root, "Tuning.EstimatorRequireTimestamp", true);
  out.estimatorRequireSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.EstimatorRequireSampleId", true);
  out.estimatorRequireMonotonicSampleId = config_validation::parseBoolWithFallback(
      root, "Tuning.EstimatorRequireMonotonicSampleId", true);
  out.intentRequireTimestamp =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireTimestamp", true);
  out.intentRequireSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireSampleId", true);
  out.intentRequireMonotonicSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireMonotonicSampleId", true);
  out.fallbackSpeedMag = config_validation::parseDoubleWithFallback(
      root, "Tuning.FallbackSpeedMag", control_config::kDefaultFallbackSpeedMag.value, 0.0, 1.0,
      "tuning");
  out.minBusVoltageV = config_validation::parseDoubleWithFallback(
      root, "Tuning.MinBusVoltageV", control_config::kDefaultMinBusVoltageV, 5.0, 24.0, "tuning");
  out.maxBusCurrentA = config_validation::parseDoubleWithFallback(
      root, "Tuning.MaxBusCurrentA", control_config::kDefaultMaxBusCurrentA, 0.1, 120.0, "tuning");
  out.minFootContacts = config_validation::parseIntWithFallback(
      root, "Tuning.MinFootContacts", control_config::kDefaultMinFootContacts, 0, kNumLegs, "tuning");
  out.maxFootContacts = config_validation::parseIntWithFallback(
      root, "Tuning.MaxFootContacts", control_config::kDefaultMaxFootContacts, 0, kNumLegs, "tuning");
  if (out.minFootContacts > out.maxFootContacts) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "Tuning.MinFootContacts > Tuning.MaxFootContacts, using defaults");
    }
    out.minFootContacts = control_config::kDefaultMinFootContacts;
    out.maxFootContacts = control_config::kDefaultMaxFootContacts;
  }
}

void TomlParser::parseGeometrySection(const toml::value& root, ParsedToml& out) const
{
  const HexapodGeometry default_geometry = geometry_config::buildDefaultHexapodGeometry();
  std::vector<double> default_mount_angles;
  std::vector<Vec3> default_coxa_offsets;
  std::vector<double> default_femur_attach;
  std::vector<double> default_tibia_attach;
  std::vector<double> default_side_sign;
  default_mount_angles.reserve(kNumLegs);
  default_coxa_offsets.reserve(kNumLegs);
  default_femur_attach.reserve(kNumLegs);
  default_tibia_attach.reserve(kNumLegs);
  default_side_sign.reserve(kNumLegs);

  for (const auto& leg : default_geometry.legGeometry) {
    default_mount_angles.push_back(rad2deg(leg.mountAngle));
    default_coxa_offsets.push_back(leg.bodyCoxaOffset);
    default_femur_attach.push_back(rad2deg(leg.servo.femurOffset));
    default_tibia_attach.push_back(rad2deg(leg.servo.tibiaOffset));
    default_side_sign.push_back(leg.servo.femurSign);
  }

  out.coxaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaLengthM", default_geometry.legGeometry[0].coxaLength.value, 0.005, 0.30,
      "geometry");
  out.femurLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.FemurLengthM", default_geometry.legGeometry[0].femurLength.value, 0.005,
      0.30, "geometry");
  out.tibiaLengthM = config_validation::parseDoubleWithFallback(
      root, "Geometry.TibiaLengthM", default_geometry.legGeometry[0].tibiaLength.value, 0.005,
      0.40, "geometry");
  out.bodyToBottomM = config_validation::parseDoubleWithFallback(
      root, "Geometry.BodyToBottomM", default_geometry.toBottom.value, 0.005, 0.30, "geometry");
  out.coxaAttachDeg = config_validation::parseDoubleWithFallback(
      root, "Geometry.CoxaAttachDeg", rad2deg(default_geometry.legGeometry[0].servo.coxaOffset),
      -180.0, 180.0, "geometry");

  out.mountAnglesDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.MountAnglesDeg", default_mount_angles, kNumLegs, -360.0, 360.0, "geometry");
  out.femurAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.FemurAttachDeg", default_femur_attach, kNumLegs, -180.0, 180.0, "geometry");
  out.tibiaAttachDeg = config_validation::parseDoubleListWithFallback(
      root, "Geometry.TibiaAttachDeg", default_tibia_attach, kNumLegs, -180.0, 180.0, "geometry");
  out.sideSign = config_validation::parseDoubleListWithFallback(
      root, "Geometry.SideSign", default_side_sign, kNumLegs, -1.0, 1.0, "geometry");
  out.coxaOffsetsM = config_validation::parseVec3ListWithFallback(
      root, "Geometry.CoxaOffsetsM", default_coxa_offsets, kNumLegs, -0.30, 0.30, "geometry");
}
