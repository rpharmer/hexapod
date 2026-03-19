#include <toml.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <memory>
#include <algorithm>
#include <array>
#include <set>
#include <iostream>

#include "logger.hpp"
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "serialCommsServer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "robot_control.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"





using namespace mn::CppLinuxSerial;
using namespace logging;


static std::atomic<bool> g_exit{false};

namespace {

MotionIntent buildMotionIntent(RobotMode mode, GaitType gait, double body_height_m)
{
  MotionIntent cmd{};
  cmd.requested_mode = mode;
  cmd.gait = gait;
  cmd.twist.twist_pos_rad = {0.0, 0.0, 0.0};
  cmd.twist.body_trans_m = {0.00, 0.00, body_height_m};
  cmd.twist.body_trans_mps = {0.00, 0.00, 0.00};
  cmd.timestamp_us = now_us();
  return cmd;
}

} // namespace

void signalHandler(int) {
    g_exit.store(true);
}



int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  auto logger = std::make_shared<AsyncLogger>("app", LogLevel::Debug, 10000);
  logger->AddSink(std::make_shared<ConsoleSink>());
  logger->AddSink(std::make_shared<FileSink>("app.log"));
  SetDefaultLogger(logger);
  
  ParsedToml config;
  if(!tomlParser("config.txt", config))
  {
    return 1;
  }

  control_config::loadFromParsedToml(config);
  geometry_config::loadFromParsedToml(config);

  auto hw = std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate, config.timeout, config.minMaxPulses);
  auto estimator = std::make_unique<SimpleEstimator>();
  
  RobotControl robot(std::move(hw), std::move(estimator), logger);

  if (!robot.init()) {
      return 1;
  }

  robot.start();

  robot.setMotionIntent(buildMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.20));

  std::this_thread::sleep_for(control_config::kStandSettlingDelay);

  while (!g_exit.load()) {
      robot.setMotionIntent(buildMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.20));

      std::this_thread::sleep_for(control_config::kCommandRefreshPeriod); // refresh command watchdog
  }

  robot.stop();
  
  LOG_WARN(logger, "All workers joined");
  LOG_ERROR(logger, "Dropped messages=", logger->DroppedMessageCount());

  logger->Flush();
  logger->Stop();
  
  return 0;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  static constexpr const char* kExpectedConfigTitle = "Hexapod Config File";
  static constexpr const char* kExpectedConfigSchema = "hexapod.server.config";
  static constexpr int kExpectedConfigSchemaVersion = 1;
  static constexpr int kExpectedJointCount = 18;
  static constexpr int kMinServoPulse = 500;
  static constexpr int kMaxServoPulse = 2500;
  static const std::array<std::string, kExpectedJointCount> kExpectedJointOrder = {
    "R31", "R32", "R33", "L31", "L32", "L33",
    "R21", "R22", "R23", "L21", "L22", "L23",
    "R11", "R12", "R13", "L11", "L12", "L13"
  };

  const HexapodGeometry default_geometry = geometry_config::buildDefaultHexapodGeometry();

  auto parse_int_with_fallback = [](const toml::value& root,
                                    const std::string& key,
                                    int default_value,
                                    int min_value,
                                    int max_value) {
    int value = toml::find_or<int>(root, key, default_value);
    if (value < min_value || value > max_value) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid tuning value for ", key, "=", value,
                 ", using default ", default_value);
      }
      return default_value;
    }
    return value;
  };

  auto parse_u64_with_fallback = [](const toml::value& root,
                                    const std::string& key,
                                    uint64_t default_value,
                                    uint64_t min_value,
                                    uint64_t max_value) {
    const int64_t raw_value = toml::find_or<int64_t>(root, key, static_cast<int64_t>(default_value));
    if (raw_value < static_cast<int64_t>(min_value) ||
        raw_value > static_cast<int64_t>(max_value)) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid tuning value for ", key, "=", raw_value,
                 ", using default ", default_value);
      }
      return default_value;
    }
    return static_cast<uint64_t>(raw_value);
  };

  auto parse_double_with_fallback = [](const toml::value& root,
                                       const std::string& key,
                                       double default_value,
                                       double min_value,
                                       double max_value) {
    double value = toml::find_or<double>(root, key, default_value);
    if (value < min_value || value > max_value) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid tuning value for ", key, "=", value,
                 ", using default ", default_value);
      }
      return default_value;
    }
    return value;
  };

  auto parse_double_list_with_fallback = [](const toml::value& root,
                                            const std::string& key,
                                            const std::vector<double>& defaults,
                                            std::size_t expected_size,
                                            double min_value,
                                            double max_value) {
    const std::vector<double> values = toml::find_or<std::vector<double>>(root, key, defaults);
    if (values.size() != expected_size) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid geometry list size for ", key, ": expected ",
                 expected_size, ", got ", values.size(), ", using defaults");
      }
      return defaults;
    }

    for (std::size_t i = 0; i < values.size(); ++i) {
      if (values[i] < min_value || values[i] > max_value) {
        if (auto logger = GetDefaultLogger()) {
          LOG_WARN(logger, "invalid geometry value for ", key, "[", i, "] = ",
                   values[i], ", using defaults");
        }
        return defaults;
      }
    }
    return values;
  };

  auto parse_vec3_list_with_fallback = [](const toml::value& root,
                                          const std::string& key,
                                          const std::vector<Vec3>& defaults,
                                          std::size_t expected_size,
                                          double min_value,
                                          double max_value) {
    const std::vector<std::array<double, 3>> raw =
        toml::find_or<std::vector<std::array<double, 3>>>(root, key, {});

    if (raw.empty()) {
      return defaults;
    }

    if (raw.size() != expected_size) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid geometry list size for ", key, ": expected ",
                 expected_size, ", got ", raw.size(), ", using defaults");
      }
      return defaults;
    }

    std::vector<Vec3> parsed;
    parsed.reserve(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
      const auto& row = raw[i];
      for (std::size_t axis = 0; axis < row.size(); ++axis) {
        if (row[axis] < min_value || row[axis] > max_value) {
          if (auto logger = GetDefaultLogger()) {
            LOG_WARN(logger, "invalid geometry value for ", key, "[", i, "][",
                     axis, "] = ", row[axis], ", using defaults");
          }
          return defaults;
        }
      }
      parsed.push_back(Vec3{row[0], row[1], row[2]});
    }
    return parsed;
  };

  try
  {
    auto root = toml::parse(filename, toml::spec::v(1,1,0));

    if(root.at("title").as_string() != kExpectedConfigTitle)
    {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "incorrect config header. expected \"", kExpectedConfigTitle, "\"");
      }
      return false;
    }

    const std::string schema = toml::find_or<std::string>(root, "Schema", "");
    if(schema != kExpectedConfigSchema)
    {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "invalid Schema '", schema, "'. expected '", kExpectedConfigSchema, "'");
      }
      return false;
    }

    const int schema_version = toml::find_or<int>(root, "SchemaVersion", -1);
    if(schema_version != kExpectedConfigSchemaVersion)
    {
      if (auto logger = GetDefaultLogger()) {
        LOG_ERROR(logger, "unsupported SchemaVersion=", schema_version,
                  ". expected ", kExpectedConfigSchemaVersion);
      }
      return false;
    }

    std::string serialDevice = toml::find_or<std::string>(root, "SerialDevice", "Error");
    if(serialDevice == "Error" || serialDevice.empty())
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "SerialDevice definition not found or empty"); }
      return false;
    }

    int baudInt = toml::find_or<int>(root, "BaudRate", -1);
    if(baudInt <= 0)
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "baudRate wasn't a positive valid number or not found"); }
      return false;
    }

    int timeout = toml::find_or<int>(root, "Timeout_ms", -1);
    if(timeout <= 0)
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "timeout wasn't a positive valid number or not found"); }
      return false;
    }

    auto calibs = toml::find_or<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations", {});
    if(calibs.empty())
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "MotorCalibrations wasn't valid or not found"); }
      return false;
    }
    if(calibs.size() != kExpectedJointCount)
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "invalid number of MotorCalibrations, expected ", kExpectedJointCount); }
      return false;
    }

    auto is_calibration_key_valid = [](const std::string& key) -> bool
    {
      return key.size() == 3 &&
             (key[0] == 'R' || key[0] == 'L') &&
             (key[1] >= '1' && key[1] <= '3') &&
             (key[2] >= '1' && key[2] <= '3');
    };

    std::set<std::string> seen_keys;
    std::set<std::string> expected_keys(kExpectedJointOrder.begin(), kExpectedJointOrder.end());

    for(const auto& calib : calibs)
    {
      const auto& key = std::get<0>(calib);
      const int min_pulse = std::get<1>(calib);
      const int max_pulse = std::get<2>(calib);

      if(!is_calibration_key_valid(key))
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "invalid motor key '", key, "' in MotorCalibrations"); }
        return false;
      }
      if(!expected_keys.contains(key))
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "unexpected motor key '", key, "' in MotorCalibrations"); }
        return false;
      }
      if(!seen_keys.insert(key).second)
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "duplicate motor key '", key, "' in MotorCalibrations"); }
        return false;
      }
      if(min_pulse >= max_pulse)
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "invalid pulse bounds for '", key, "': min must be < max"); }
        return false;
      }
      if(min_pulse < kMinServoPulse || max_pulse > kMaxServoPulse)
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "invalid pulse bounds for '", key, "': must be within [", kMinServoPulse, ", ", kMaxServoPulse, "]"); }
        return false;
      }
    }

    for(const auto& expected_key : expected_keys)
    {
      if(!seen_keys.contains(expected_key))
      {
        if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "missing motor key '", expected_key, "' in MotorCalibrations"); }
        return false;
      }
    }

    std::sort(calibs.begin(), calibs.end(),
    [](const std::tuple<std::string, int, int>& a,
       const std::tuple<std::string, int, int>& b) -> bool
       {
         const std::string& key_a = std::get<0>(a);
         const std::string& key_b = std::get<0>(b);

         const std::array<int, 3> sort_key_a = {
           key_a[1] - '0',
           key_a[0] == 'R' ? 1 : 0,
           3 - (key_a[2] - '0')
         };
         const std::array<int, 3> sort_key_b = {
           key_b[1] - '0',
           key_b[0] == 'R' ? 1 : 0,
           3 - (key_b[2] - '0')
         };

         return sort_key_a > sort_key_b;
       });

    std::vector<float> calibsF;
    calibsF.reserve(36);

    for(const auto& calib : calibs)
    {
      calibsF.push_back(static_cast<float>(std::get<1>(calib)));
      calibsF.push_back(static_cast<float>(std::get<2>(calib)));
    }

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

    out.serialDevice = serialDevice;
    out.baudRate = baudInt;
    out.timeout = timeout;
    out.minMaxPulses = calibsF;

    out.busLoopPeriodUs = parse_int_with_fallback(root, "Tuning.BusLoopPeriodUs",
                                                  control_config::kDefaultBusLoopPeriodUs, 500, 50000);
    out.estimatorLoopPeriodUs = parse_int_with_fallback(root, "Tuning.EstimatorLoopPeriodUs",
                                                        control_config::kDefaultEstimatorLoopPeriodUs, 500, 50000);
    out.controlLoopPeriodUs = parse_int_with_fallback(root, "Tuning.ControlLoopPeriodUs",
                                                      control_config::kDefaultControlLoopPeriodUs, 500, 50000);
    out.safetyLoopPeriodUs = parse_int_with_fallback(root, "Tuning.SafetyLoopPeriodUs",
                                                     control_config::kDefaultSafetyLoopPeriodUs, 500, 50000);
    out.diagnosticsPeriodMs = parse_int_with_fallback(root, "Tuning.DiagnosticsPeriodMs",
                                                      control_config::kDefaultDiagnosticsPeriodMs, 100, 10000);
    out.commandRefreshPeriodMs = parse_int_with_fallback(root, "Tuning.CommandRefreshPeriodMs",
                                                         control_config::kDefaultCommandRefreshPeriodMs, 10, 1000);
    out.standSettlingDelayMs = parse_int_with_fallback(root, "Tuning.StandSettlingDelayMs",
                                                       control_config::kDefaultStandSettlingDelayMs, 0, 10000);
    out.maxTiltRad = parse_double_with_fallback(root, "Tuning.MaxTiltRad",
                                                control_config::kDefaultMaxTiltRad.value, 0.1, 1.5);
    out.commandTimeoutUs = parse_u64_with_fallback(root, "Tuning.CommandTimeoutUs",
                                                   control_config::kDefaultCommandTimeoutUs.value, 10000, 2000000);
    out.fallbackSpeedMag = parse_double_with_fallback(root, "Tuning.FallbackSpeedMag",
                                                      control_config::kDefaultFallbackSpeedMag.value, 0.0, 1.0);
    out.minBusVoltageV = parse_double_with_fallback(root, "Tuning.MinBusVoltageV",
                                                    control_config::kDefaultMinBusVoltageV, 5.0, 24.0);
    out.maxBusCurrentA = parse_double_with_fallback(root, "Tuning.MaxBusCurrentA",
                                                    control_config::kDefaultMaxBusCurrentA, 0.1, 120.0);
    out.minFootContacts = parse_int_with_fallback(root, "Tuning.MinFootContacts",
                                                  control_config::kDefaultMinFootContacts, 0, kNumLegs);
    out.maxFootContacts = parse_int_with_fallback(root, "Tuning.MaxFootContacts",
                                                  control_config::kDefaultMaxFootContacts, 0, kNumLegs);
    if (out.minFootContacts > out.maxFootContacts) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "Tuning.MinFootContacts > Tuning.MaxFootContacts, using defaults");
      }
      out.minFootContacts = control_config::kDefaultMinFootContacts;
      out.maxFootContacts = control_config::kDefaultMaxFootContacts;
    }

    out.coxaLengthM = parse_double_with_fallback(root, "Geometry.CoxaLengthM", default_geometry.legGeometry[0].coxaLength.value, 0.005, 0.30);
    out.femurLengthM = parse_double_with_fallback(root, "Geometry.FemurLengthM", default_geometry.legGeometry[0].femurLength.value, 0.005, 0.30);
    out.tibiaLengthM = parse_double_with_fallback(root, "Geometry.TibiaLengthM", default_geometry.legGeometry[0].tibiaLength.value, 0.005, 0.40);
    out.bodyToBottomM = parse_double_with_fallback(root, "Geometry.BodyToBottomM", default_geometry.toBottom.value, 0.005, 0.30);
    out.coxaAttachDeg = parse_double_with_fallback(root, "Geometry.CoxaAttachDeg", rad2deg(default_geometry.legGeometry[0].servo.coxaOffset), -180.0, 180.0);

    out.mountAnglesDeg = parse_double_list_with_fallback(root, "Geometry.MountAnglesDeg", default_mount_angles, kNumLegs, -360.0, 360.0);
    out.femurAttachDeg = parse_double_list_with_fallback(root, "Geometry.FemurAttachDeg", default_femur_attach, kNumLegs, -180.0, 180.0);
    out.tibiaAttachDeg = parse_double_list_with_fallback(root, "Geometry.TibiaAttachDeg", default_tibia_attach, kNumLegs, -180.0, 180.0);
    out.sideSign = parse_double_list_with_fallback(root, "Geometry.SideSign", default_side_sign, kNumLegs, -1.0, 1.0);
    out.coxaOffsetsM = parse_vec3_list_with_fallback(root, "Geometry.CoxaOffsetsM", default_coxa_offsets, kNumLegs, -0.30, 0.30);

    return true;
  }
  catch(const std::exception& ex)
  {
    if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "failed to parse '", filename, "': ", ex.what()); }
    return false;
  }
}
