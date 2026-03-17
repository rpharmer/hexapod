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
  static constexpr int kExpectedJointCount = 18;
  static constexpr int kMinServoPulse = 500;
  static constexpr int kMaxServoPulse = 2500;
  static const std::array<std::string, kExpectedJointCount> kExpectedJointOrder = {
    "R31", "R32", "R33", "L31", "L32", "L33",
    "R21", "R22", "R23", "L21", "L22", "L23",
    "R11", "R12", "R13", "L11", "L12", "L13"
  };

  try
  {
    // select TOML version at runtime (optional)
    auto root = toml::parse(filename, toml::spec::v(1,1,0));
    
    // Check that the file is a Hexapod Config File
    if(root.at("title").as_string() != "Hexapod Config File")
    {
      if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "incorrect config header. expected \"Hexapod Config File\""); }
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
    
    // sort calibration data to correct motor order
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
  
  for(auto it = calibs.begin(); it!=calibs.end(); ++it)
  {
    calibsF.push_back(static_cast<float>(std::get<1>(*it))); // min pulse value
    calibsF.push_back(static_cast<float>(std::get<2>(*it))); // max pulse value
  }
  
    out.serialDevice = serialDevice;
    out.baudRate = baudInt;
    out.timeout = timeout;
    out.minMaxPulses = calibsF;
    
    return true;
  }
  catch(const std::exception& ex)
  {
    if (auto logger = GetDefaultLogger()) { LOG_ERROR(logger, "failed to parse '", filename, "': ", ex.what()); }
    return false;
  }
}
