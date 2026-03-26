#include "control_pipeline.hpp"

ControlPipeline::ControlPipeline(control_config::GaitConfig config)
    : planner_(config),
      gait_(config) {}

PipelineStepResult ControlPipeline::runStep(const RobotState& estimated,
                                            const MotionIntent& intent,
                                            const SafetyState& safety_state,
                                            bool bus_ok,
                                            uint64_t loop_counter) {
    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }

    const RuntimeGaitPolicy gait_policy = planner_.plan(estimated, intent, safety_state);
    const GaitState gait_state = gait_.update(estimated, intent, safety_state, gait_policy);
    const ContactManagerOutput contact_adjusted = contact_manager_.update(estimated, gait_state, gait_policy);
    const LegTargets leg_targets = body_.update(estimated, intent, contact_adjusted.managed_gait, contact_adjusted.managed_policy, safety_state);
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
