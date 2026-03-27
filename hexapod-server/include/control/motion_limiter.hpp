#pragma once

#include "gait_policy_planner.hpp"
#include "types.hpp"

struct MotionLimiterDiagnostics {
    bool enabled{false};
    bool intent_modified{false};
    bool gait_policy_modified{false};
    DurationSec loop_dt{DurationSec{0.0}};
};

struct MotionLimiterOutput {
    MotionIntent limited_intent{};
    RuntimeGaitPolicy adapted_gait_policy{};
    MotionLimiterDiagnostics diagnostics{};
};

class MotionLimiter {
public:
    MotionLimiterOutput update(const RobotState& estimated,
                               const MotionIntent& intent,
                               const RuntimeGaitPolicy& gait_policy,
                               const SafetyState& safety_state,
                               const DurationSec& loop_dt) const;
};
