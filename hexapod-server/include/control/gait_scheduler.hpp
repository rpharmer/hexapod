#pragma once

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

class GaitScheduler {
public:
    explicit GaitScheduler(control_config::GaitConfig config = {});

    GaitState update(const RobotState& est, const MotionIntent& intent, const SafetyState& safety);

private:
    control_config::GaitConfig config_{};
    HexapodGeometry geometry_{defaultHexapodGeometry()};
    double wrap01(double x) const;
    double maxReachUtilization(const RobotState& est) const;
    double phase_accum_{0.0};
    TimePointUs last_update_us_{};
};
