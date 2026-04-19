#include "control_pipeline.hpp"

#include "motion_intent_utils.hpp"

ControlPipeline::ControlPipeline(control_config::GaitConfig gait_config,
                                 control_config::LocomotionCommandConfig loco_config,
                                 control_config::FootTerrainConfig foot_terrain_config)
    : gait_(gait_config),
      loco_cmd_(loco_config),
      body_(gait_config, foot_terrain_config) {}

void ControlPipeline::reset() {
    gait_.reset();
    loco_cmd_.reset();
    locomotion_stability_.reset();
}

PipelineStepResult ControlPipeline::runStep(const RobotState& estimated,
                                            const MotionIntent& intent,
                                            const SafetyState& safety_state,
                                            bool bus_ok,
                                            uint64_t loop_counter,
                                            const LocalMapSnapshot* terrain_snapshot) {
    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }

    const PlanarMotionCommand planar = planarMotionCommand(intent);
    TimePointUs command_clock = intent.timestamp_us;
    if (command_clock.isZero()) {
        command_clock = estimated.timestamp_us;
    }
    const BodyTwist cmd_twist = loco_cmd_.update(intent, planar, command_clock);

    GaitState gait_state = gait_.update(estimated, intent, safety_state, cmd_twist);
    locomotion_stability_.apply(intent, gait_state);
    const LegTargets leg_targets =
        body_.update(estimated, intent, gait_state, safety_state, cmd_twist, terrain_snapshot);
    const JointTargets joint_targets = ik_.solve(estimated, leg_targets, safety_state);

    ControlStatus status{};
    status.active_mode = active_mode;
    status.estimator_valid = !estimated.timestamp_us.isZero();
    status.bus_ok = bus_ok;
    status.active_fault = safety_state.active_fault;
    status.loop_counter = loop_counter;

    PipelineStepResult result{};
    result.joint_targets = joint_targets;
    result.status = status;
    return result;
}
