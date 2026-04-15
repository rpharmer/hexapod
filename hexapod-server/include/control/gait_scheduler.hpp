#pragma once

#include "control_config.hpp"
#include "gait_params.hpp"
#include "types.hpp"

class GaitScheduler {
public:
    explicit GaitScheduler(control_config::GaitConfig config = {});

    GaitState update(const RobotState& est, const MotionIntent& intent, const SafetyState& safety);

private:
    control_config::GaitConfig config_{};
    double wrap01(double x) const;
    double phase_accum_{0.0};
    TimePointUs last_update_us_{};

    GaitType committed_gait_{GaitType::TRIPOD};
    bool committed_initialized_{false};
    UnifiedGaitDescription transition_from_snap_{};
    UnifiedGaitDescription last_blended_{};
    TimePointUs transition_start_us_{};
    bool have_last_blended_{false};
};
