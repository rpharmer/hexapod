#pragma once

#include "scenario_driver.hpp"
#include "types.hpp"

MotionIntent makeMotionIntent(RobotMode mode, GaitType gait, double body_height_m);
MotionIntent makeMotionIntent(const ScenarioMotionIntent& motion);
