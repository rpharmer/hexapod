#include "control_pipeline.hpp"

#include "geometry_config.hpp"
#include "joint_angle_gravity_feedforward.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"
#include "support_assessment.hpp"

ControlPipeline::ControlPipeline(control_config::GaitConfig gait_config,
                                 control_config::LocomotionCommandConfig loco_config,
                                 control_config::SafetyConfig safety_config,
                                 control_config::CommandGovernorConfig governor_config,
                                 control_config::FootTerrainConfig foot_terrain_config,
                                 control_config::GravityFeedforwardConfig gravity_feedforward_config,
                                 control_config::LocomotionRedesignConfig locomotion_redesign_config,
                                 runtime_resource_monitoring::Profiler* profiler)
    : profiler_(profiler),
      command_governor_(governor_config, safety_config),
      gait_(gait_config),
      loco_cmd_(loco_config),
      body_(gait_config, foot_terrain_config),
      gravity_feedforward_(gravity_feedforward_config),
      locomotion_redesign_(locomotion_redesign_config) {}

void ControlPipeline::reset() {
    command_governor_.reset();
    gait_.reset();
    loco_cmd_.reset();
    locomotion_stability_.reset();
    resetJointAngleGravityFeedforwardState();
    last_gait_state_ = GaitState{};
    have_last_gait_state_ = false;
}

const control_config::CommandGovernorConfig& ControlPipeline::commandGovernorConfig() const {
    return command_governor_.config();
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

    MotionIntent preview_intent = intent;
    const CommandGovernorState preview_governor =
        command_governor_.preview(estimated, preview_intent, safety_state, last_gait_state_);
    const PlanarMotionCommand preview_planar = planarMotionCommand(preview_intent);
    const BodyTwist preview_twist = rawLocomotionTwistFromIntent(preview_intent, preview_planar);
    const GaitState preview_gait = gait_.preview(estimated, preview_intent, safety_state, preview_twist, preview_governor);
    const SupportAssessment current_support = assessSupportState(
        estimated, preview_intent, preview_gait, geometry_config::activeHexapodGeometry());

    MotionIntent governed_intent = intent;
    CommandGovernorState governor =
        command_governor_.apply(estimated, governed_intent, safety_state, last_gait_state_, &current_support);

    const PlanarMotionCommand planar = planarMotionCommand(governed_intent);
    TimePointUs command_clock = intent.timestamp_us;
    if (command_clock.isZero()) {
        command_clock = estimated.timestamp_us;
    }
    BodyTwist cmd_twist{};
    const bool recovery_hold_stage =
        governor.recovery_stage == RecoveryStage::ActiveHold ||
        governor.recovery_stage == RecoveryStage::Settling;
    {
        const auto loco_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineLocoCommand))
                      : runtime_resource_monitoring::Profiler::Scope{};
        cmd_twist = recovery_hold_stage ? loco_cmd_.snapTo(BodyTwist{}, command_clock)
                                        : loco_cmd_.update(governed_intent, planar, command_clock);
        (void)loco_scope;
    }

    GaitState gait_state{};
    {
        const auto gait_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineGait))
                      : runtime_resource_monitoring::Profiler::Scope{};
        gait_state = gait_.update(estimated, governed_intent, safety_state, cmd_twist, governor);
        (void)gait_scope;
    }

    {
        const auto stability_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineStability))
                      : runtime_resource_monitoring::Profiler::Scope{};
        const LocomotionFeasibility pre_stability_feasibility = computeLocomotionFeasibility(
            estimated,
            governed_intent,
            gait_state,
            last_gait_state_,
            safety_state,
            geometry_config::activeHexapodGeometry(),
            locomotion_redesign_);
        locomotion_stability_.apply(
            estimated,
            governed_intent,
            gait_state,
            locomotion_redesign_.enable_feasibility_lift_gating ? &pre_stability_feasibility : nullptr);
        (void)stability_scope;
    }

    const bool no_leg_safe_to_lift = std::none_of(
        gait_state.support_liftoff_safe_to_lift.begin(),
        gait_state.support_liftoff_safe_to_lift.end(),
        [](const bool safe) { return safe; });
    const bool all_legs_held = std::all_of(
        gait_state.stability_hold_stance.begin(),
        gait_state.stability_hold_stance.end(),
        [](const bool hold) { return hold; });
    const SupportAssessment post_stability_support = assessSupportState(
        estimated, governed_intent, gait_state, geometry_config::activeHexapodGeometry());
    LocomotionFeasibility locomotion_feasibility = computeLocomotionFeasibility(
        estimated,
        governed_intent,
        gait_state,
        last_gait_state_,
        safety_state,
        geometry_config::activeHexapodGeometry(),
        locomotion_redesign_);
    const bool high_demand_walk =
        governor.requested_planar_speed_mps >= 0.18 ||
        governor.requested_yaw_rate_radps >= 0.35;
    const bool support_deadlocked = no_leg_safe_to_lift && all_legs_held;
    const bool sparse_support_deadlocked =
        post_stability_support.support_count <= 2 &&
        post_stability_support.uncertain_support_count == 0;
    const bool legacy_force_recovery_hold =
        governed_intent.requested_mode == RobotMode::WALK &&
        high_demand_walk &&
        sparse_support_deadlocked &&
        gait_state.static_stability_margin_m <= 0.0 &&
        support_deadlocked;
    const bool force_recovery_hold = locomotion_redesign_.enable_feasibility_recovery
                                         ? locomotion_feasibility.recovery_recommended
                                         : legacy_force_recovery_hold;
    if (force_recovery_hold &&
        governor.recovery_stage != RecoveryStage::ActiveHold &&
        governor.recovery_stage != RecoveryStage::Settling) {
        command_governor_.latchRecoveryHold(command_clock);
    }

    const bool hold_like_stage_before_finalize =
        force_recovery_hold ||
        governor.recovery_stage == RecoveryStage::ActiveHold ||
        governor.recovery_stage == RecoveryStage::Settling;
    if (hold_like_stage_before_finalize) {
        gait_state.in_stance.fill(true);
        gait_state.stability_hold_stance.fill(true);
        gait_state.step_length_m = 0.0;
        gait_state.stride_phase_rate_hz = FrequencyHz{0.0};
    }

    const SupportAssessment final_support = assessSupportState(
        estimated, governed_intent, gait_state, geometry_config::activeHexapodGeometry());
    governor = command_governor_.finalizeRecovery(
        estimated, governed_intent, safety_state, gait_state, final_support, governor, command_clock);
    locomotion_feasibility.height = evaluateHeightPolicySnapshot(
        estimated, governed_intent, gait_state, governor, locomotion_redesign_);

    MotionIntent body_intent = governed_intent;
    const bool recovery_force_stand =
        governor.recovery_stage == RecoveryStage::ActiveHold ||
        governor.recovery_stage == RecoveryStage::Settling;
    if (recovery_force_stand) {
        cmd_twist = loco_cmd_.snapTo(BodyTwist{}, command_clock);
        gait_state.in_stance.fill(true);
        gait_state.stability_hold_stance.fill(true);
        gait_state.step_length_m = 0.0;
        gait_state.stride_phase_rate_hz = FrequencyHz{0.0};
        body_intent.requested_mode = RobotMode::STAND;
        body_intent.speed_mps = LinearRateMps{};
        body_intent.cmd_vx_mps = LinearRateMps{};
        body_intent.cmd_vy_mps = LinearRateMps{};
        body_intent.cmd_yaw_radps = AngularRateRadPerSec{};
        body_intent.twist.body_trans_m.x = 0.0;
        body_intent.twist.body_trans_m.y = 0.0;
        body_intent.twist.body_trans_mps = VelocityMps3{};
        body_intent.twist.twist_vel_radps = AngularVelocityRadPerSec3{};
    }

    const LegTargets leg_targets = [&]() {
        const auto body_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineBody))
                      : runtime_resource_monitoring::Profiler::Scope{};
        const auto* contact_modes =
            locomotion_redesign_.enable_contact_mode_planning ? &locomotion_feasibility.contact : nullptr;
        const LegTargets targets =
            body_.update(estimated, body_intent, gait_state, safety_state, cmd_twist, terrain_snapshot, contact_modes);
        (void)body_scope;
        return targets;
    }();

    JointTargets joint_targets = [&]() {
        const auto ik_scope =
            profiler_ ? profiler_->scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlPipelineIK))
                      : runtime_resource_monitoring::Profiler::Scope{};
        JointTargets targets = ik_.solve(estimated, leg_targets, safety_state);
        (void)ik_scope;
        return targets;
    }();
    applyJointAngleGravityFeedforward(
        gravity_feedforward_,
        geometry_config::activeHexapodGeometry(),
        estimated,
        gait_state,
        joint_targets,
        locomotion_redesign_.enable_contact_mode_planning ? &locomotion_feasibility.contact : nullptr);
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
    result.command_governor = governor;
    result.locomotion_feasibility = locomotion_feasibility;
    last_gait_state_ = gait_state;
    have_last_gait_state_ = true;
    return result;
}
