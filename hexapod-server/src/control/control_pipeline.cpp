#include "control_pipeline.hpp"

#include "motion_intent_utils.hpp"

ControlPipeline::ControlPipeline(control_config::GaitConfig gait_config,
                                 control_config::LocomotionCommandConfig loco_config,
                                 control_config::FootTerrainConfig foot_terrain_config,
                                 control_config::InvestigationConfig investigation_config,
                                 runtime_resource_monitoring::Profiler* profiler)
    : profiler_(profiler),
      gait_(gait_config),
      loco_cmd_(loco_config),
      body_(gait_config, foot_terrain_config, investigation_config) {}

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
    const auto pipeline_scope =
        profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipeline))
                  : runtime_resource_monitoring::Profiler::Scope{};

    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }

    const PlanarMotionCommand planar = planarMotionCommand(intent);
    TimePointUs command_clock = intent.timestamp_us;
    if (command_clock.isZero()) {
        command_clock = estimated.timestamp_us;
    }
    BodyTwist cmd_twist{};
    {
        const auto loco_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineLocoCommand))
                      : runtime_resource_monitoring::Profiler::Scope{};
        cmd_twist = loco_cmd_.update(intent, planar, command_clock);
        (void)loco_scope;
    }

    GaitState gait_state{};
    {
        const auto gait_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineGait))
                      : runtime_resource_monitoring::Profiler::Scope{};
        gait_state = gait_.update(estimated, intent, safety_state, cmd_twist);
        (void)gait_scope;
    }

    {
        const auto stability_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineStability))
                      : runtime_resource_monitoring::Profiler::Scope{};
        locomotion_stability_.apply(intent, gait_state);
        (void)stability_scope;
    }

    const LegTargets leg_targets = [&]() {
        const auto body_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineBody))
                      : runtime_resource_monitoring::Profiler::Scope{};
        const LegTargets targets =
            body_.update(estimated, intent, gait_state, safety_state, cmd_twist, terrain_snapshot);
        (void)body_scope;
        return targets;
    }();

    const JointTargets joint_targets = [&]() {
        const auto ik_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineIK))
                      : runtime_resource_monitoring::Profiler::Scope{};
        const JointTargets targets = ik_.solve(estimated, leg_targets, safety_state);
        (void)ik_scope;
        return targets;
    }();
    (void)pipeline_scope;

    ControlStatus status{};
    status.active_mode = active_mode;
    status.estimator_valid = !estimated.timestamp_us.isZero();
    status.bus_ok = bus_ok;
    status.active_fault = safety_state.active_fault;
    status.loop_counter = loop_counter;

    PipelineStepResult result{};
    result.leg_targets = leg_targets;
    result.joint_targets = joint_targets;
    result.status = status;
    result.gait_state = gait_state;
    return result;
}
