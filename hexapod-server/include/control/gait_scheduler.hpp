#pragma once

#include "control_config.hpp"
#include "gait_policy_planner.hpp"
#include "types.hpp"

class GaitScheduler {
public:
    explicit GaitScheduler(control_config::GaitConfig config = {});

    GaitState update(const RobotState& est, const MotionIntent& intent, const SafetyState& safety);

    GaitState update(const RobotState& est, const MotionIntent& intent, const SafetyState& safety, const RuntimeGaitPolicy& policy);

private:
    control_config::GaitConfig config_{};
    double wrap01(double x) const;
    FrequencyHz applyCadenceSlew(const FrequencyHz& target_rate_hz, const DurationSec& dt);
    double phase_accum_{0.0};
    FrequencyHz cadence_hz_{FrequencyHz{0.0}};
    TimePointUs last_update_us_{};
};
