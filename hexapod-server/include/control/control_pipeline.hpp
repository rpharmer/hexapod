#pragma once

#include "body_controller.hpp"
#include "control_config.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"
#include "local_map.hpp"
#include "locomotion_command.hpp"
#include "locomotion_stability.hpp"
#include "types.hpp"

struct PipelineStepResult {
    JointTargets joint_targets{};
    ControlStatus status{};
};

class ControlPipeline {
public:
    explicit ControlPipeline(control_config::GaitConfig gait_config = {},
                             control_config::LocomotionCommandConfig loco_config = {},
                             control_config::FootTerrainConfig foot_terrain_config = {});

    void reset();
    PipelineStepResult runStep(const RobotState& estimated,
                               const MotionIntent& intent,
                               const SafetyState& safety_state,
                               bool bus_ok,
                               uint64_t loop_counter,
                               const LocalMapSnapshot* terrain_snapshot = nullptr);

private:
    GaitScheduler gait_;
    LocomotionCommandProcessor loco_cmd_{};
    LocomotionStability locomotion_stability_{};
    BodyController body_;
    LegIK ik_;
};
