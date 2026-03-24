#ifndef MODE_RUNNERS_HPP
#define MODE_RUNNERS_HPP

#include <atomic>
#include <memory>

#include "cli_options.hpp"
#include "control_config.hpp"
#include "logger.hpp"
#include "robot_control.hpp"

class ScenarioRunner
{
public:
  int run(RobotControl& robot, const CliOptions& options, const std::shared_ptr<logging::AsyncLogger>& logger) const;
};

class InteractiveRunner
{
public:
  int run(RobotControl& robot,
          const CliOptions& options,
          const control_config::ControlConfig& control_cfg,
          const std::shared_ptr<logging::AsyncLogger>& logger,
          const std::atomic<bool>& exit_flag) const;
};

#endif // MODE_RUNNERS_HPP
