#pragma once

#include "control_config.hpp"
#include "gait_policy_planner.hpp"
#include "types.hpp"

struct MotionLimiterDiagnostics {
    bool enabled{false};
    bool intent_modified{false};
    bool gait_policy_modified{false};
    uint8_t phase{0};
    uint8_t constraint_reason{0};
    double adaptation_scale_linear{1.0};
    double adaptation_scale_yaw{1.0};
    bool hard_clamp_linear{false};
    bool hard_clamp_yaw{false};
    bool hard_clamp_reach{false};
    DurationSec loop_dt{DurationSec{0.0}};
};

struct MotionLimiterOutput {
    MotionIntent limited_intent{};
    RuntimeGaitPolicy adapted_gait_policy{};
    MotionLimiterDiagnostics diagnostics{};
};

class MotionLimiter {
public:
    explicit MotionLimiter(control_config::MotionLimiterConfig config = {});

    MotionLimiterOutput update(const RobotState& estimated,
                               const MotionIntent& intent,
                               const RuntimeGaitPolicy& gait_policy,
                               const SafetyState& safety_state,
                               const DurationSec& loop_dt);

private:
    control_config::MotionLimiterConfig config_{};
    MotionIntent previous_limited_intent_{};
    bool has_previous_limited_intent_{false};
    bool previous_walking_{false};
    double startup_phase_elapsed_s_{0.0};
    double shutdown_phase_elapsed_s_{0.0};
};
