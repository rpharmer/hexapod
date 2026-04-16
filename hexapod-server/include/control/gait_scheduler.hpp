#pragma once

#include "control_config.hpp"
#include "gait_params.hpp"
#include "twist_field.hpp"
#include "types.hpp"

/**
 * Parametric gait timing only: which legs are in swing vs stance and when (duty, phase offsets,
 * cadence, transition blending, speed-based preset blends). Does not own spatial foot paths —
 * foothold + `swing_trajectory` + stance integration do.
 */
class GaitScheduler {
public:
    explicit GaitScheduler(control_config::GaitConfig config = {});

    GaitState update(const RobotState& est,
                      const MotionIntent& intent,
                      const SafetyState& safety,
                      const BodyTwist& cmd_twist);

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

    double last_cmd_vx_mps_{0.0};
    double last_cmd_vy_mps_{0.0};
};
