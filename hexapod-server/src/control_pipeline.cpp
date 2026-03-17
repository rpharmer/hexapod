#include "control_pipeline.hpp"

PipelineStepResult ControlPipeline::runStep(const EstimatedState& estimated,
                                            const MotionIntent& intent,
                                            const SafetyState& safety_state,
                                            bool bus_ok,
                                            uint64_t loop_counter) {
    RobotMode active_mode = intent.requested_mode;
    if (safety_state.active_fault != FaultCode::NONE) {
        active_mode = RobotMode::FAULT;
    }

    const GaitState gait_state = gait_.update(estimated, intent, safety_state);
    const LegTargets leg_targets = body_.update(estimated, intent, gait_state, safety_state);
    const JointTargets joint_targets = ik_.solve(estimated, leg_targets, safety_state);

    ControlStatus status{};
    status.active_mode = active_mode;
    status.estimator_valid = (estimated.timestamp_us != 0);
    status.bus_ok = bus_ok;
    status.active_fault = safety_state.active_fault;
    status.loop_counter = loop_counter;

    PipelineStepResult result{};
    result.joint_targets = joint_targets;
    result.status = status;
    return result;
}
