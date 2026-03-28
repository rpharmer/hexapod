#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "gait_policy_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RobotState nominalEstimatedState() {
    RobotState est{};
    est.timestamp_us = now_us();
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }
    return est;
}

MotionIntent walkingIntent() {
    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.speed_mps = LinearRateMps{0.12};
    walk.timestamp_us = now_us();
    return walk;
}

bool unit_body_accel_limiter_and_phase_transitions() {
    control_config::MotionLimiterConfig limiter_config{};
    limiter_config.foot_velocity_limit_mps = 0.05;
    limiter_config.startup_phase_threshold = std::chrono::milliseconds{200};
    limiter_config.shutdown_phase_threshold = std::chrono::milliseconds{200};
    ControlPipeline pipeline(control_config::ControlConfig{
        .gait = control_config::GaitConfig{},
        .motion_limiter = limiter_config});

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent idle{};
    idle.requested_mode = RobotMode::SAFE_IDLE;
    const DurationSec loop_dt{0.02};
    (void)pipeline.runStep(est, idle, safety, loop_dt, true, 1);

    MotionIntent walk = walkingIntent();
    const PipelineStepResult start = pipeline.runStep(est, walk, safety, loop_dt, true, 2);

    walk.body_pose_setpoint.body_trans_m.x = 0.25;
    const PipelineStepResult limited = pipeline.runStep(est, walk, safety, loop_dt, true, 3);

    MotionIntent stop = walk;
    stop.requested_mode = RobotMode::SAFE_IDLE;
    const PipelineStepResult stop_result = pipeline.runStep(est, stop, safety, loop_dt, true, 4);

    return expect(start.status.dynamic_gait.limiter_enabled, "limiter telemetry should be enabled during transition") &&
           expect(start.status.dynamic_gait.limiter_phase == 1, "start transition should report body-leads phase") &&
           expect(limited.status.dynamic_gait.hard_clamp_linear, "large body step should activate linear hard clamp") &&
           expect(limited.status.dynamic_gait.adaptation_scale_linear < 1.0, "linear adaptation scale should drop below unity when clamped") &&
           expect(stop_result.status.dynamic_gait.limiter_phase == 2, "stop transition should report legs-lead phase");
}

bool unit_gait_adaptation_scaling_behavior() {
    GaitPolicyPlanner planner;
    RobotState est = nominalEstimatedState();
    MotionIntent walk = walkingIntent();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    for (auto& leg : est.leg_states) {
        for (auto& joint : leg.joint_state) {
            joint.vel_radps = AngularRateRadPerSec{20.0};
        }
    }
    const RuntimeGaitPolicy constrained = planner.plan(est, walk, safety);

    for (auto& leg : est.leg_states) {
        for (auto& joint : leg.joint_state) {
            joint.vel_radps = AngularRateRadPerSec{0.1};
        }
    }
    const RuntimeGaitPolicy nominal = planner.plan(est, walk, safety);

    return expect(constrained.adaptation_scale_cadence < 1.0,
                  "high observed joint velocity should down-scale cadence") &&
           expect(constrained.adaptation_scale_step < 1.0,
                  "high observed joint velocity should down-scale step length") &&
           expect(constrained.hard_clamp_cadence,
                  "down-scaled cadence should mark hard-clamp flag") &&
           expect(nominal.adaptation_scale_cadence >= constrained.adaptation_scale_cadence,
                  "nominal velocity should not be more constrained than overspeed case");
}

bool integration_bounded_ramp_response_to_step_commands() {
    ControlPipeline pipeline;
    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    const DurationSec loop_dt{0.03};

    MotionIntent walk = walkingIntent();
    const PipelineStepResult first = pipeline.runStep(est, walk, safety, loop_dt, true, 10);

    walk.body_pose_setpoint.body_trans_m.x = 0.35;
    const PipelineStepResult second = pipeline.runStep(est, walk, safety, loop_dt, true, 11);
    const double dx = std::abs(second.leg_targets.feet[0].pos_body_m.x - first.leg_targets.feet[0].pos_body_m.x);
    const double observed_speed = dx / loop_dt.value;

    return expect(observed_speed <= 0.5,
                  "step command response should remain bounded by transition ramp speed");
}

bool scenario_motion_limiter_config_changes_clamp_behavior() {
    control_config::ControlConfig strict_cfg{};
    strict_cfg.motion_limiter.foot_velocity_limit_mps = 0.02;

    control_config::ControlConfig relaxed_cfg{};
    relaxed_cfg.motion_limiter.foot_velocity_limit_mps = 20.0;

    ControlPipeline strict_pipeline(strict_cfg);
    ControlPipeline relaxed_pipeline(relaxed_cfg);

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    const DurationSec loop_dt{0.02};

    MotionIntent walk = walkingIntent();
    const PipelineStepResult strict_base = strict_pipeline.runStep(est, walk, safety, loop_dt, true, 21);
    const PipelineStepResult relaxed_base = relaxed_pipeline.runStep(est, walk, safety, loop_dt, true, 21);

    walk.body_pose_setpoint.body_trans_m.x = 0.25;
    const PipelineStepResult strict_step = strict_pipeline.runStep(est, walk, safety, loop_dt, true, 22);
    const PipelineStepResult relaxed_step = relaxed_pipeline.runStep(est, walk, safety, loop_dt, true, 22);

    const double strict_dx = std::abs(
        strict_step.leg_targets.feet[0].pos_body_m.x - strict_base.leg_targets.feet[0].pos_body_m.x);
    const double relaxed_dx = std::abs(
        relaxed_step.leg_targets.feet[0].pos_body_m.x - relaxed_base.leg_targets.feet[0].pos_body_m.x);

    return expect(strict_step.status.dynamic_gait.hard_clamp_linear,
                  "strict limiter config should clamp linear command") &&
           expect(!relaxed_step.status.dynamic_gait.hard_clamp_linear,
                  "relaxed limiter config should avoid linear clamp for same command") &&
           expect(relaxed_dx > strict_dx + 1e-4,
                  "relaxed limiter should allow larger per-step displacement than strict limiter");
}

bool scenario_body_leads_on_start_and_legs_lead_on_stop() {
    ControlPipeline pipeline;
    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent idle{};
    idle.requested_mode = RobotMode::SAFE_IDLE;
    const DurationSec loop_dt{0.02};
    (void)pipeline.runStep(est, idle, safety, loop_dt, true, 1);

    MotionIntent walk = walkingIntent();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const PipelineStepResult started = pipeline.runStep(est, walk, safety, loop_dt, true, 2);

    MotionIntent stop = walk;
    stop.requested_mode = RobotMode::SAFE_IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const PipelineStepResult stopped = pipeline.runStep(est, stop, safety, loop_dt, true, 3);

    return expect(started.status.dynamic_gait.limiter_phase == 1,
                  "pipeline telemetry should report body-leads phase on start") &&
           expect(stopped.status.dynamic_gait.limiter_phase == 2,
                  "pipeline telemetry should report legs-lead phase on stop") &&
           expect(started.status.dynamic_gait.limiter_enabled,
                  "pipeline telemetry should mark limiter enabled");
}

} // namespace

int main() {
    if (!unit_body_accel_limiter_and_phase_transitions()) {
        return EXIT_FAILURE;
    }
    if (!unit_gait_adaptation_scaling_behavior()) {
        return EXIT_FAILURE;
    }
    if (!integration_bounded_ramp_response_to_step_commands()) {
        return EXIT_FAILURE;
    }
    if (!scenario_body_leads_on_start_and_legs_lead_on_stop()) {
        return EXIT_FAILURE;
    }
    if (!scenario_motion_limiter_config_changes_clamp_behavior()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
