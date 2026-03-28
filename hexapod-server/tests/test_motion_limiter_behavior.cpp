#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "motion_limiter.hpp"

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

bool exactlyEqual(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool exactlyEqual(const MotionIntent& lhs, const MotionIntent& rhs) {
    return lhs.requested_mode == rhs.requested_mode &&
           lhs.gait == rhs.gait &&
           lhs.speed_mps.value == rhs.speed_mps.value &&
           lhs.heading_rad.value == rhs.heading_rad.value &&
           exactlyEqual(lhs.body_pose_setpoint.orientation_rad, rhs.body_pose_setpoint.orientation_rad) &&
           exactlyEqual(lhs.body_pose_setpoint.angular_velocity_radps, rhs.body_pose_setpoint.angular_velocity_radps) &&
           exactlyEqual(lhs.body_pose_setpoint.body_trans_m, rhs.body_pose_setpoint.body_trans_m) &&
           exactlyEqual(lhs.body_pose_setpoint.body_trans_mps, rhs.body_pose_setpoint.body_trans_mps) &&
           lhs.sample_id == rhs.sample_id &&
           lhs.timestamp_us.value == rhs.timestamp_us.value;
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

bool unit_motion_limiter_scales_with_dt_and_stays_bounded() {
    control_config::MotionLimiterConfig limiter_cfg{};
    limiter_cfg.foot_velocity_limit_mps = 0.05;
    MotionLimiter limiter{limiter_cfg};

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    RuntimeGaitPolicy gait_policy{};
    MotionIntent walk = walkingIntent();

    (void)limiter.update(est, walk, gait_policy, safety, DurationSec{0.02});

    walk.body_pose_setpoint.body_trans_m.x = 0.5;
    const MotionLimiterOutput tiny_dt =
        limiter.update(est, walk, gait_policy, safety, DurationSec{1e-9});

    walk.body_pose_setpoint.body_trans_m.x = 1.0;
    const MotionLimiterOutput large_dt =
        limiter.update(est, walk, gait_policy, safety, DurationSec{0.4});

    const double tiny_dx =
        std::abs(tiny_dt.limited_intent.body_pose_setpoint.body_trans_m.x - 0.0);
    const double large_dx = std::abs(
        large_dt.limited_intent.body_pose_setpoint.body_trans_m.x - tiny_dt.limited_intent.body_pose_setpoint.body_trans_m.x);
    const double expected_large_limit = limiter_cfg.foot_velocity_limit_mps * 0.4;

    return expect(tiny_dt.diagnostics.hard_clamp_linear,
                  "tiny dt with large body step should still register linear clamp") &&
           expect(tiny_dx <= limiter_cfg.foot_velocity_limit_mps * 1e-9 + 1e-10,
                  "tiny dt should produce a near-zero linear step") &&
           expect(large_dt.diagnostics.hard_clamp_linear,
                  "large dt with larger body step should still clamp linearly") &&
           expect(large_dx <= expected_large_limit + 1e-9,
                  "large dt response must stay bounded by velocity limit times dt") &&
           expect(large_dx > tiny_dx + 1e-5,
                  "larger dt should allow a proportionally larger bounded step");
}

bool unit_motion_limiter_replay_is_deterministic_for_fixed_dt() {
    control_config::MotionLimiterConfig limiter_cfg{};
    limiter_cfg.foot_velocity_limit_mps = 0.05;
    MotionLimiter first_pass_limiter{limiter_cfg};
    MotionLimiter second_pass_limiter{limiter_cfg};

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    RuntimeGaitPolicy gait_policy{};
    const DurationSec fixed_dt{0.02};

    std::array<MotionIntent, 4> sequence{};
    sequence[0] = walkingIntent();
    sequence[0].body_pose_setpoint.body_trans_m = Vec3{0.00, 0.00, 0.00};
    sequence[1] = sequence[0];
    sequence[1].body_pose_setpoint.body_trans_m = Vec3{0.20, -0.08, 0.03};
    sequence[1].body_pose_setpoint.orientation_rad.z = 0.6;
    sequence[2] = sequence[1];
    sequence[2].body_pose_setpoint.body_trans_m = Vec3{0.50, -0.08, 0.03};
    sequence[2].body_pose_setpoint.body_trans_mps = Vec3{0.40, -0.20, 0.10};
    sequence[3] = sequence[2];
    sequence[3].body_pose_setpoint.angular_velocity_radps = Vec3{0.5, -0.3, 2.0};

    std::array<MotionLimiterOutput, 4> first_outputs{};
    std::array<MotionLimiterOutput, 4> second_outputs{};
    for (size_t i = 0; i < sequence.size(); ++i) {
        first_outputs[i] = first_pass_limiter.update(est, sequence[i], gait_policy, safety, fixed_dt);
        second_outputs[i] = second_pass_limiter.update(est, sequence[i], gait_policy, safety, fixed_dt);
    }

    for (size_t i = 0; i < sequence.size(); ++i) {
        if (!expect(exactlyEqual(first_outputs[i].limited_intent, second_outputs[i].limited_intent),
                    "identical intent replay with fixed dt should produce identical limited_intent")) {
            return false;
        }
    }

    return true;
}

bool integration_gait_scheduler_progress_tracks_elapsed_dt() {
    GaitScheduler scheduler;
    RobotState est = nominalEstimatedState();
    MotionIntent walk = walkingIntent();
    SafetyState safety{};
    RuntimeGaitPolicy policy{};
    policy.cadence_hz = FrequencyHz{3.0};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        policy.per_leg[leg].phase_offset = 0.0;
        policy.per_leg[leg].duty_cycle = 0.5;
    }

    (void)scheduler.update(est, walk, safety, policy);
    const GaitState tiny_elapsed = scheduler.update(est, walk, safety, policy);
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
    const GaitState larger_elapsed = scheduler.update(est, walk, safety, policy);

    return expect(tiny_elapsed.stride_phase_rate_hz.value >= 0.0,
                  "gait scheduler cadence should remain non-negative for tiny dt") &&
           expect(larger_elapsed.stride_phase_rate_hz.value >= tiny_elapsed.stride_phase_rate_hz.value,
                  "larger elapsed dt should not reduce cadence slew progress in steady walk") &&
           expect(larger_elapsed.phase[0] >= tiny_elapsed.phase[0],
                  "larger elapsed dt should advance phase progression");
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
    if (!unit_motion_limiter_scales_with_dt_and_stays_bounded()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_replay_is_deterministic_for_fixed_dt()) {
        return EXIT_FAILURE;
    }
    if (!integration_gait_scheduler_progress_tracks_elapsed_dt()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
