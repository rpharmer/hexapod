#include <toml.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <memory>
#include <thread>
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

  auto hw = std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate, config.timeout);
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
      cmd.twist.body_height_pos_m = 0.20;
      cmd.twist.body_height_vel_mps = 0;
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
      cmd.twist.body_height_pos_m = 0.20;
      cmd.timestamp_us = now_us();
      robot.setMotionIntent(cmd);

      std::this_thread::sleep_for(100ms); // refresh command watchdog
  }

  robot.stop();
  return 0;
}

bool tomlParser(std::string filename, ParsedToml& out)
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
  if(baudInt == -1)
  {
    printf("baudRate wasn't a positive valid number or not found\n");
    return false;
  }
  
  int timeout = toml::find_or<int>(root, "Timeout_ms", -1);
  if(timeout == -1)
  {
    printf("timeout wasn't a positive valid number or not found\n");
    return false;
  }
  
  auto calibs = toml::find_or<std::vector<std::tuple<std::string, int, int>>>(root, "MotorCalibrations", {});
  if(calibs.size() == 0)
  {
    printf("MotorCalibrations wasn't valid or not found\n");
    return false;
  }
  if(calibs.size() != 18)
  {
    printf("invalid number of MotorCalibrations, expected 18\n");
    return false;
  }
  
  // sort calibration data to correct motor order
  sort(calibs.begin(),calibs.end(),
  [](const std::tuple<std::string, int, int>& a,
     const std::tuple<std::string, int, int>& b) -> bool
     {
       std::string _a = std::get<0>(a);
       std::string _b = std::get<0>(b);
       
       // reorder the characters so they can be compared
       //{R|L} {1|2|3} {1|2|3}
       // 2nd    1st     3rd
       //  +      +       -
       std::string _a_ = {_a[1], _a[0], static_cast<char>(-_a[2])};
       std::string _b_ = {_b[1], _b[0], static_cast<char>(-_b[2])};
       
       return _a_ > _b_;
     });
  
  std::vector<float> calibsF;
  calibsF.reserve(36 * sizeof(float));
  
  int index = 0;
  for(auto it = calibs.begin(); it!=calibs.end(); ++it)
  {
    calibsF[index++] = std::get<1>(*it); // min pulse value
    calibsF[index++] = std::get<2>(*it); // max pulse value
  }
  
  out.serialDevice = serialDevice;
  out.baudRate = baudInt;
  out.timeout = timeout;
  out.minMaxPulses = calibsF;
  
  return true;
}
