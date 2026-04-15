#pragma once

#include "scenario_driver.hpp"
#include "types.hpp"

struct PlanarMotionCommand {
    double vx_mps{};
    double vy_mps{};
    double yaw_rate_radps{};
};

PlanarMotionCommand planarMotionCommand(const MotionIntent& intent);

MotionIntent makeMotionIntent(RobotMode mode, GaitType gait, double body_height_m);
MotionIntent makeMotionIntent(const ScenarioMotionIntent& motion);
