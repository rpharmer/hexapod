#pragma once

#include "body_controller.hpp"
#include "control_config.hpp"
#include "contact_manager.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"
#include "motion_limiter.hpp"
#include "types.hpp"

struct PipelineStepResult {
    LegTargets leg_targets{};
    JointTargets joint_targets{};
    ControlStatus status{};
};

class ControlPipeline {
public:
    explicit ControlPipeline(control_config::GaitConfig config = {});

    PipelineStepResult runStep(const RobotState& estimated,
                               const MotionIntent& intent,
                               const SafetyState& safety_state,
                               const DurationSec& loop_dt,
                               bool bus_ok,
                               uint64_t loop_counter);

private:
    GaitPolicyPlanner planner_;
    MotionLimiter motion_limiter_{};
    GaitScheduler gait_;
    ContactManager contact_manager_;
    BodyController body_;
    LegIK ik_;
};
