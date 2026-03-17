#pragma once

#include "body_controller.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"
#include "types.hpp"

struct PipelineStepResult {
    JointTargets joint_targets{};
    ControlStatus status{};
};

class ControlPipeline {
public:
    PipelineStepResult runStep(const EstimatedState& estimated,
                               const MotionIntent& intent,
                               const SafetyState& safety_state,
                               bool bus_ok,
                               uint64_t loop_counter);

private:
    GaitScheduler gait_;
    BodyController body_;
    LegIK ik_;
};
