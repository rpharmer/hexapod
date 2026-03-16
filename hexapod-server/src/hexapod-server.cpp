#include <toml.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <memory>
#include <thread>
#include <algorithm>
#include <array>
#include "hexapod-common.hpp"
#include "hexapod-server.hpp"
#include "serialCommsServer.hpp"
#include "estimator.hpp"
#include "hardware_bridge.hpp"
#include "robot_control.hpp"




using namespace std::chrono_literals;
using namespace mn::CppLinuxSerial;


static std::atomic<bool> g_exit{false};

void signalHandler(int) {
    g_exit.store(true);
}



int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  ParsedToml config;
  if(!tomlParser("config.txt", config))
  {
    return 1;
  }

  auto hw = std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate, config.timeout, config.minMaxPulses);
  auto estimator = std::make_unique<SimpleEstimator>();
  
  RobotControl robot(std::move(hw), std::move(estimator));

  if (!robot.init()) {
      return 1;
  }

  robot.start();

  {
      MotionIntent cmd{};
      cmd.requested_mode = RobotMode::STAND;
      cmd.gait = GaitType::TRIPOD;
      cmd.twist.body_trans_m.x = 0.00;
      cmd.twist.body_trans_m.y = 0.00;
      cmd.twist.body_trans_m.z = 0.20;
      cmd.twist.body_trans_mps.x = 0.00;
      cmd.twist.body_trans_mps.y = 0.00;
      cmd.twist.body_trans_mps.z = 0.00;
      cmd.timestamp_us = now_us();
      robot.setMotionIntent(cmd);
  }

  std::this_thread::sleep_for(2s);

  while (!g_exit.load()) {
      MotionIntent cmd{};
      cmd.requested_mode = RobotMode::WALK;
      cmd.gait = GaitType::TRIPOD;

      cmd.twist.twist_pos_rad.x = 0.0;
      cmd.twist.twist_pos_rad.y = 0.0;
      cmd.twist.twist_pos_rad.z = 0.0;
      cmd.twist.body_trans_m.x = 0.00;
      cmd.twist.body_trans_m.y = 0.00;
      cmd.twist.body_trans_m.z = 0.20;
      cmd.twist.body_trans_mps.x = 0.00;
      cmd.twist.body_trans_mps.y = 0.00;
      cmd.twist.body_trans_mps.z = 0.00;
      cmd.timestamp_us = now_us();
      robot.setMotionIntent(cmd);

      std::this_thread::sleep_for(100ms); // refresh command watchdog
  }

  robot.stop();
  return 0;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  try
  {
    // select TOML version at runtime (optional)
    auto root = toml::parse(filename, toml::spec::v(1,1,0));
    
    // Check that the file is a Hexapod Config File
    if(root.at("title").as_string() != "Hexapod Config File")
    {
      printf("incorrect config header. expected \"Hexapod Config File\"\n");
      return false;
    }
    
    std::string serialDevice = toml::find_or<std::string>(root, "SerialDevice", "Error");
    if(serialDevice == "Error")
    {
      printf("SerialDevice definition not found\n");
      return false;
    }
    
    int baudInt = toml::find_or<int>(root, "BaudRate", -1);
    if(baudInt <= 0)
    {
      printf("baudRate wasn't a positive valid number or not found\n");
      return false;
    }
    
    int timeout = toml::find_or<int>(root, "Timeout_ms", -1);
    if(timeout <= 0)
    {
      printf("timeout wasn't a positive valid number or not found\n");
      return false;
    }
    
    auto calibs = toml::find_or<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations", {});
    if(calibs.empty())
    {
      printf("MotorCalibrations wasn't valid or not found\n");
      return false;
    }
    if(calibs.size() != 18)
    {
      printf("invalid number of MotorCalibrations, expected 18\n");
      return false;
    }

    auto is_calibration_key_valid = [](const std::string& key) -> bool
    {
      return key.size() == 3 &&
             (key[0] == 'R' || key[0] == 'L') &&
             (key[1] >= '1' && key[1] <= '3') &&
             (key[2] >= '1' && key[2] <= '3');
    };

    for(const auto& calib : calibs)
    {
      const auto& key = std::get<0>(calib);
      const int min_pulse = std::get<1>(calib);
      const int max_pulse = std::get<2>(calib);

      if(!is_calibration_key_valid(key))
      {
        printf("invalid motor key '%s' in MotorCalibrations\n", key.c_str());
        return false;
      }
      if(min_pulse > max_pulse)
      {
        printf("invalid pulse bounds for '%s': min > max\n", key.c_str());
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
    printf("failed to parse '%s': %s\n", filename.c_str(), ex.what());
    return false;
  }
}
