#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "motion_limiter.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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

bool allFiniteVec3(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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

bool scenario_quick_mode_toggles_preserve_phase_order_with_elapsed_gating() {
    constexpr uint8_t kLimiterPhaseTracking = 0;
    constexpr uint8_t kLimiterPhaseBodyLeadsOnStart = 1;
    constexpr uint8_t kLimiterPhaseLegsLeadOnStop = 2;

    control_config::MotionLimiterConfig limiter_cfg{};
    limiter_cfg.startup_phase_threshold = std::chrono::milliseconds{60};
    limiter_cfg.shutdown_phase_threshold = std::chrono::milliseconds{60};

    ControlPipeline pipeline(control_config::ControlConfig{
        .gait = control_config::GaitConfig{},
        .motion_limiter = limiter_cfg});

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent idle{};
    idle.requested_mode = RobotMode::SAFE_IDLE;
    MotionIntent walk = walkingIntent();
    const DurationSec loop_dt{0.02};

    const PipelineStepResult idle_prime = pipeline.runStep(est, idle, safety, loop_dt, true, 100);
    const PipelineStepResult walk_start = pipeline.runStep(est, walk, safety, loop_dt, true, 101);
    const PipelineStepResult walk_gate_1 = pipeline.runStep(est, walk, safety, loop_dt, true, 102);
    const PipelineStepResult walk_gate_2 = pipeline.runStep(est, walk, safety, loop_dt, true, 103);
    const PipelineStepResult walk_gate_3 = pipeline.runStep(est, walk, safety, loop_dt, true, 104);
    const PipelineStepResult walk_tracking = pipeline.runStep(est, walk, safety, loop_dt, true, 105);
    const PipelineStepResult stop_start = pipeline.runStep(est, idle, safety, loop_dt, true, 106);
    const PipelineStepResult stop_gate_1 = pipeline.runStep(est, idle, safety, loop_dt, true, 107);
    const PipelineStepResult stop_gate_2 = pipeline.runStep(est, idle, safety, loop_dt, true, 108);
    const PipelineStepResult stop_gate_3 = pipeline.runStep(est, idle, safety, loop_dt, true, 109);
    const PipelineStepResult stop_tracking = pipeline.runStep(est, idle, safety, loop_dt, true, 110);

    return expect(idle_prime.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop,
                  "idle priming should begin in stop-gating phase") &&
           expect(walk_start.status.dynamic_gait.limiter_phase == kLimiterPhaseBodyLeadsOnStart,
                  "walk toggle should enter body-leads-on-start phase immediately") &&
           expect(walk_gate_1.status.dynamic_gait.limiter_phase == kLimiterPhaseBodyLeadsOnStart &&
                      walk_gate_2.status.dynamic_gait.limiter_phase == kLimiterPhaseBodyLeadsOnStart &&
                      walk_gate_3.status.dynamic_gait.limiter_phase == kLimiterPhaseBodyLeadsOnStart,
                  "startup gating should persist until elapsed time reaches threshold") &&
           expect(walk_tracking.status.dynamic_gait.limiter_phase == kLimiterPhaseTracking,
                  "startup gating should exit to tracking only after threshold elapsed") &&
           expect(stop_start.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop,
                  "stop toggle should enter legs-lead-on-stop phase immediately") &&
           expect(stop_gate_1.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop &&
                      stop_gate_2.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop &&
                      stop_gate_3.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop,
                  "shutdown gating should persist until elapsed time reaches threshold") &&
           expect(stop_tracking.status.dynamic_gait.limiter_phase == kLimiterPhaseTracking,
                  "shutdown gating should exit to tracking only after threshold elapsed") &&
           expect(walk_start.status.dynamic_gait.limiter_phase == kLimiterPhaseBodyLeadsOnStart &&
                      walk_tracking.status.dynamic_gait.limiter_phase == kLimiterPhaseTracking &&
                      stop_start.status.dynamic_gait.limiter_phase == kLimiterPhaseLegsLeadOnStop,
                  "phase ordering should be body-leads -> tracking -> legs-lead across quick toggles");
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

bool unit_motion_limiter_yaw_clamp_uses_rate_limit_semantics() {
    control_config::MotionLimiterConfig limiter_cfg{};
    limiter_cfg.body_yaw_rate_limit_radps = 0.4;
    limiter_cfg.body_angular_accel_limit_radps2 = Vec3{100.0, 100.0, 100.0};
    MotionLimiter limiter{limiter_cfg};

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    RuntimeGaitPolicy gait_policy{};

    MotionIntent walk = walkingIntent();
    walk.body_pose_setpoint.orientation_rad.z = 0.0;
    (void)limiter.update(est, walk, gait_policy, safety, DurationSec{0.02});

    walk.body_pose_setpoint.orientation_rad.z = 1.0;
    const DurationSec small_dt{0.02};
    const MotionLimiterOutput small_step =
        limiter.update(est, walk, gait_policy, safety, small_dt);

    walk.body_pose_setpoint.orientation_rad.z = 2.0;
    const DurationSec large_dt{0.10};
    const MotionLimiterOutput large_step =
        limiter.update(est, walk, gait_policy, safety, large_dt);

    const double first_yaw = small_step.limited_intent.body_pose_setpoint.orientation_rad.z;
    const double second_yaw = large_step.limited_intent.body_pose_setpoint.orientation_rad.z;
    const double small_delta = std::abs(first_yaw - 0.0);
    const double large_delta = std::abs(second_yaw - first_yaw);
    const double small_limit = limiter_cfg.body_yaw_rate_limit_radps * small_dt.value;
    const double large_limit = limiter_cfg.body_yaw_rate_limit_radps * large_dt.value;

    return expect(small_step.diagnostics.hard_clamp_yaw,
                  "small dt yaw step should clamp against yaw-rate bound") &&
           expect(large_step.diagnostics.hard_clamp_yaw,
                  "large dt yaw step should clamp against yaw-rate bound") &&
           expect(small_delta <= small_limit + 1e-12,
                  "small dt yaw delta should be bounded by yaw-rate * dt") &&
           expect(large_delta <= large_limit + 1e-12,
                  "large dt yaw delta should be bounded by yaw-rate * dt") &&
           expect(large_delta > small_delta + 1e-6,
                  "larger dt should allow a proportionally larger bounded yaw step");
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

bool unit_motion_limiter_ignores_estimated_angular_velocity_spikes() {
    control_config::MotionLimiterConfig limiter_cfg{};
    limiter_cfg.foot_velocity_limit_mps = 100.0;
    limiter_cfg.body_linear_accel_limit_xy_mps2 = 100.0;
    limiter_cfg.body_linear_accel_limit_z_mps2 = 100.0;
    limiter_cfg.body_angular_accel_limit_radps2 = Vec3{0.4, 0.6, 1.0};
    MotionLimiter limiter{limiter_cfg};

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    RuntimeGaitPolicy gait_policy{};
    const DurationSec loop_dt{0.05};

    MotionIntent intent = walkingIntent();
    intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.0};
    (void)limiter.update(est, intent, gait_policy, safety, loop_dt);

    intent.body_pose_setpoint.angular_velocity_radps = Vec3{10.0, -10.0, 8.0};

    est.body_pose_state.angular_velocity_radps = AngularVelocityRadPerSec3{1200.0, -900.0, 700.0};
    const MotionLimiterOutput first_limited =
        limiter.update(est, intent, gait_policy, safety, loop_dt);

    est.body_pose_state.angular_velocity_radps = AngularVelocityRadPerSec3{-1500.0, 1100.0, -950.0};
    const MotionLimiterOutput second_limited =
        limiter.update(est, intent, gait_policy, safety, loop_dt);

    const Vec3 max_step = limiter_cfg.body_angular_accel_limit_radps2 * loop_dt.value;
    const Vec3 first_ang = first_limited.limited_intent.body_pose_setpoint.angular_velocity_radps;
    const Vec3 second_ang = second_limited.limited_intent.body_pose_setpoint.angular_velocity_radps;
    const Vec3 observed_step = second_ang - first_ang;
    const Vec3 requested_ang = intent.body_pose_setpoint.angular_velocity_radps;
    const Vec3 expected_second = Vec3{
        std::clamp(requested_ang.x - first_ang.x, -max_step.x, max_step.x) + first_ang.x,
        std::clamp(requested_ang.y - first_ang.y, -max_step.y, max_step.y) + first_ang.y,
        std::clamp(requested_ang.z - first_ang.z, -max_step.z, max_step.z) + first_ang.z};

    return expect(first_limited.diagnostics.hard_clamp_yaw,
                  "first step toward requested angular velocity should be accel-clamped") &&
           expect(second_limited.diagnostics.hard_clamp_yaw,
                  "second step should remain accel-clamped despite extreme estimated angular velocity") &&
           expect(std::abs(observed_step.x) <= max_step.x + 1e-12 &&
                      std::abs(observed_step.y) <= max_step.y + 1e-12 &&
                      std::abs(observed_step.z) <= max_step.z + 1e-12,
                  "limited angular velocity delta should be bounded only by configured accel limits") &&
           expect(std::abs(second_ang.x - expected_second.x) < 1e-12 &&
                      std::abs(second_ang.y - expected_second.y) < 1e-12 &&
                      std::abs(second_ang.z - expected_second.z) < 1e-12,
                  "limited angular velocity should depend only on previous limited intent and accel bounds");
}

bool unit_motion_limiter_rejects_non_finite_commands() {
    MotionLimiter limiter{};
    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    RuntimeGaitPolicy gait_policy{};

    MotionIntent baseline = walkingIntent();
    baseline.body_pose_setpoint.body_trans_m = Vec3{0.01, -0.01, 0.2};
    baseline.body_pose_setpoint.body_trans_mps = Vec3{0.0, 0.0, 0.0};
    baseline.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.0};
    (void)limiter.update(est, baseline, gait_policy, safety, DurationSec{0.02});

    MotionIntent non_finite = baseline;
    non_finite.body_pose_setpoint.body_trans_m = Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        0.2};
    non_finite.body_pose_setpoint.body_trans_mps = Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    non_finite.body_pose_setpoint.angular_velocity_radps = Vec3{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(),
        -std::numeric_limits<double>::infinity()};
    non_finite.body_pose_setpoint.orientation_rad.z = std::numeric_limits<double>::quiet_NaN();

    const MotionLimiterOutput out =
        limiter.update(est, non_finite, gait_policy, safety, DurationSec{0.02});

    const Vec3 limited_pos = out.limited_intent.body_pose_setpoint.body_trans_m;
    const Vec3 limited_vel = out.limited_intent.body_pose_setpoint.body_trans_mps;
    const Vec3 limited_ang = out.limited_intent.body_pose_setpoint.angular_velocity_radps;
    const bool finite_pose = allFiniteVec3(limited_pos) &&
                             allFiniteVec3(limited_vel) &&
                             allFiniteVec3(limited_ang) &&
                             std::isfinite(out.limited_intent.body_pose_setpoint.orientation_rad.z);

    return expect(finite_pose,
                  "non-finite motion intent inputs should be clamped or rejected to finite limiter outputs") &&
           expect(out.diagnostics.intent_modified,
                  "rejecting non-finite commands should mark limiter intent as modified");
}

bool integration_multi_clamp_same_cycle_remains_bounded_without_spikes() {
    control_config::ControlConfig cfg{};
    cfg.motion_limiter.foot_velocity_limit_mps = 0.03;
    cfg.motion_limiter.body_angular_accel_limit_radps2 = Vec3{0.8, 0.8, 0.8};
    ControlPipeline pipeline(cfg);

    RobotState est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    const DurationSec loop_dt{0.02};

    MotionIntent walk = walkingIntent();
    const PipelineStepResult baseline = pipeline.runStep(est, walk, safety, loop_dt, true, 300);

    walk.body_pose_setpoint.body_trans_m = Vec3{1.2, 0.0, 0.2};
    walk.body_pose_setpoint.orientation_rad.z = 2.2;

    std::array<PipelineStepResult, 5> sequence{};
    for (std::size_t i = 0; i < sequence.size(); ++i) {
        sequence[i] = pipeline.runStep(est, walk, safety, loop_dt, true, 301 + i);
        const auto& gait = sequence[i].status.dynamic_gait;
        if (!expect(gait.hard_clamp_linear,
                    "extreme translation should trigger motion limiter clamp while walking")) {
            return false;
        }
        if (!expect(gait.hard_clamp_reach,
                    "extreme translation should also trigger reach-envelope clamp in the same cycle")) {
            return false;
        }
        if (!expect(std::isfinite(gait.adaptation_scale_linear) &&
                        gait.adaptation_scale_linear >= 0.0 &&
                        gait.adaptation_scale_linear <= 1.0,
                    "combined clamp path should keep linear adaptation scale bounded to [0, 1]")) {
            return false;
        }
        if (!expect(std::isfinite(gait.adaptation_scale_yaw) &&
                        gait.adaptation_scale_yaw >= 0.0 &&
                        gait.adaptation_scale_yaw <= 1.0,
                    "combined clamp path should keep yaw adaptation scale bounded to [0, 1]")) {
            return false;
        }
        if (!expect(std::isfinite(gait.adaptation_scale_cadence) &&
                        gait.adaptation_scale_cadence >= 0.0 &&
                        gait.adaptation_scale_cadence <= 1.0 &&
                        std::isfinite(gait.adaptation_scale_step) &&
                        gait.adaptation_scale_step >= 0.0 &&
                        gait.adaptation_scale_step <= 1.0,
                    "combined clamp path should keep cadence/step adaptation scales bounded to [0, 1]")) {
            return false;
        }
    }

    const double first_dx = sequence[0].leg_targets.feet[0].pos_body_m.x - baseline.leg_targets.feet[0].pos_body_m.x;
    if (!expect(std::abs(first_dx) > 1e-9,
                "multi-clamp stress sequence should produce a measurable command update")) {
        return false;
    }

    double max_abs_step = std::abs(first_dx);
    double oscillation_budget = 0.0;
    for (std::size_t i = 1; i < sequence.size(); ++i) {
        const double dx =
            sequence[i].leg_targets.feet[0].pos_body_m.x - sequence[i - 1].leg_targets.feet[0].pos_body_m.x;
        max_abs_step = std::max(max_abs_step, std::abs(dx));
        oscillation_budget += std::abs(dx);
        if (!expect(std::isfinite(dx),
                    "multi-clamp command deltas must stay finite across repeated cycles")) {
            return false;
        }
    }

    return expect(max_abs_step <= std::abs(first_dx) + 1e-6,
                  "clamp ordering should not create larger downstream command spikes after the initial transient") &&
           expect(oscillation_budget <= std::abs(first_dx) * static_cast<double>(sequence.size()) + 1e-6,
                  "combined clamp loop should stay bounded without runaway oscillation energy");
}

bool scenario_first_motion_startup_transients_not_history_sensitive() {
    ControlPipeline fresh_pipeline;
    ControlPipeline history_pipeline;

    RobotState fresh_est = nominalEstimatedState();
    RobotState history_est = nominalEstimatedState();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.stable = true;
    safety.support_contact_count = 6;

    const DurationSec loop_dt{0.02};

    MotionIntent idle{};
    idle.requested_mode = RobotMode::SAFE_IDLE;

    auto runStep = [&](ControlPipeline& pipeline,
                       RobotState& est,
                       MotionIntent& intent,
                       uint64_t cycle,
                       bool update_estimate) {
        intent.sample_id += 1;
        intent.timestamp_us = now_us();
        est.timestamp_us = now_us();
        const PipelineStepResult result = pipeline.runStep(est, intent, safety, loop_dt, true, cycle);
        if (update_estimate) {
            est.leg_states = result.joint_targets.leg_states;
        }
        return result;
    };

    (void)runStep(fresh_pipeline, fresh_est, idle, 1, true);
    (void)runStep(history_pipeline, history_est, idle, 1, true);

    MotionIntent history_stand = idle;
    history_stand.requested_mode = RobotMode::STAND;
    history_stand.body_pose_setpoint.body_trans_m.z = 0.20;
    MotionIntent history_walk = walkingIntent();
    history_walk.speed_mps = LinearRateMps{0.18};
    history_walk.heading_rad = AngleRad{0.35};
    history_walk.body_pose_setpoint.body_trans_m = Vec3{0.03, -0.02, 0.22};

    for (uint64_t cycle = 2; cycle <= 13; ++cycle) {
        (void)runStep(history_pipeline, history_est, history_stand, cycle, true);
    }
    for (uint64_t cycle = 14; cycle <= 31; ++cycle) {
        (void)runStep(history_pipeline, history_est, history_walk, cycle, true);
    }
    for (uint64_t cycle = 32; cycle <= 43; ++cycle) {
        (void)runStep(history_pipeline, history_est, history_stand, cycle, true);
    }

    MotionIntent first_motion = walkingIntent();
    first_motion.speed_mps = LinearRateMps{0.16};
    first_motion.heading_rad = AngleRad{0.20};
    first_motion.body_pose_setpoint.body_trans_m = Vec3{0.02, 0.01, 0.21};
    first_motion.body_pose_setpoint.body_trans_mps = Vec3{0.06, 0.00, 0.00};
    first_motion.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.45};

    MotionIntent fresh_first_motion = first_motion;
    MotionIntent history_first_motion = first_motion;

    RobotState fresh_pre_first = fresh_est;
    RobotState history_pre_first = history_est;

    const PipelineStepResult fresh_first = runStep(fresh_pipeline, fresh_est, fresh_first_motion, 44, true);
    const PipelineStepResult history_first = runStep(history_pipeline, history_est, history_first_motion, 44, true);

    constexpr int kTransientCycles = 6;
    double fresh_peak_foot_vel = 0.0;
    double history_peak_foot_vel = 0.0;
    double fresh_peak_joint_vel = 0.0;
    double history_peak_joint_vel = 0.0;
    double fresh_first_cycle_joint_vel = 0.0;
    double history_first_cycle_joint_vel = 0.0;
    double fresh_followup_peak_joint_vel = 0.0;
    double history_followup_peak_joint_vel = 0.0;

    auto stepPeakJointVelocity = [&](const PipelineStepResult& result, const RobotState& est_before) {
        double step_peak_joint_vel = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double prev_pos = est_before.leg_states[leg].joint_state[joint].pos_rad.value;
                const double next_pos = result.joint_targets.leg_states[leg].joint_state[joint].pos_rad.value;
                step_peak_joint_vel = std::max(step_peak_joint_vel, std::abs(next_pos - prev_pos) / loop_dt.value);
            }
        }
        return step_peak_joint_vel;
    };

    auto updateFootVelocityPeak = [&](const PipelineStepResult& result, double& peak_foot_vel) {
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3 v = result.leg_targets.feet[leg].vel_body_mps;
            peak_foot_vel = std::max(peak_foot_vel, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
        }
    };

    updateFootVelocityPeak(fresh_first, fresh_peak_foot_vel);
    fresh_first_cycle_joint_vel = stepPeakJointVelocity(fresh_first, fresh_pre_first);
    fresh_peak_joint_vel = std::max(fresh_peak_joint_vel, fresh_first_cycle_joint_vel);

    updateFootVelocityPeak(history_first, history_peak_foot_vel);
    history_first_cycle_joint_vel = stepPeakJointVelocity(history_first, history_pre_first);
    history_peak_joint_vel = std::max(history_peak_joint_vel, history_first_cycle_joint_vel);

    for (int i = 1; i < kTransientCycles; ++i) {
        RobotState fresh_prev = fresh_est;
        const PipelineStepResult fresh_step =
            runStep(fresh_pipeline, fresh_est, fresh_first_motion, 44 + static_cast<uint64_t>(i), true);
        updateFootVelocityPeak(fresh_step, fresh_peak_foot_vel);
        const double fresh_step_peak_joint_vel = stepPeakJointVelocity(fresh_step, fresh_prev);
        fresh_peak_joint_vel = std::max(fresh_peak_joint_vel, fresh_step_peak_joint_vel);
        fresh_followup_peak_joint_vel = std::max(fresh_followup_peak_joint_vel, fresh_step_peak_joint_vel);

        RobotState history_prev = history_est;
        const PipelineStepResult history_step =
            runStep(history_pipeline, history_est, history_first_motion, 44 + static_cast<uint64_t>(i), true);
        updateFootVelocityPeak(history_step, history_peak_foot_vel);
        const double history_step_peak_joint_vel = stepPeakJointVelocity(history_step, history_prev);
        history_peak_joint_vel = std::max(history_peak_joint_vel, history_step_peak_joint_vel);
        history_followup_peak_joint_vel = std::max(history_followup_peak_joint_vel, history_step_peak_joint_vel);
    }

    const double foot_vel_budget = control_config::kDefaultMotionFootVelocityLimitMps * 1.15;
    const double startup_joint_vel_budget = 12.0;
    const double followup_joint_vel_budget = control_config::kDefaultMotionJointSoftVelocityLimitRadps * 1.30;
    const double peak_foot_vel_delta = std::abs(fresh_peak_foot_vel - history_peak_foot_vel);
    const double first_cycle_joint_vel_delta =
        std::abs(fresh_first_cycle_joint_vel - history_first_cycle_joint_vel);
    const double followup_peak_joint_vel_delta =
        std::abs(fresh_followup_peak_joint_vel - history_followup_peak_joint_vel);
    const double first_command_delta_m = std::sqrt(
        std::pow(fresh_first.leg_targets.feet[0].pos_body_m.x - history_first.leg_targets.feet[0].pos_body_m.x, 2.0) +
        std::pow(fresh_first.leg_targets.feet[0].pos_body_m.y - history_first.leg_targets.feet[0].pos_body_m.y, 2.0) +
        std::pow(fresh_first.leg_targets.feet[0].pos_body_m.z - history_first.leg_targets.feet[0].pos_body_m.z, 2.0));
    return expect(fresh_first.status.active_fault == FaultCode::NONE,
                  "fresh runtime first-motion command should not trip safety faults") &&
           expect(history_first.status.active_fault == FaultCode::NONE,
                  "history-conditioned first-motion command should not trip safety faults") &&
           expect(fresh_peak_foot_vel <= foot_vel_budget,
                  "fresh runtime first-motion startup transient should stay within foot-velocity budget") &&
           expect(history_peak_foot_vel <= foot_vel_budget,
                  "history-conditioned first-motion startup transient should stay within foot-velocity budget") &&
           expect(fresh_peak_joint_vel <= startup_joint_vel_budget,
                  "fresh runtime first-motion startup transient should stay within startup joint-velocity budget") &&
           expect(history_peak_joint_vel <= startup_joint_vel_budget,
                  "history-conditioned first-motion startup transient should stay within startup joint-velocity budget") &&
           expect(fresh_followup_peak_joint_vel <= followup_joint_vel_budget,
                  "fresh runtime post-startup transient should stay within follow-up joint-velocity budget") &&
           expect(history_followup_peak_joint_vel <= followup_joint_vel_budget,
                  "history-conditioned post-startup transient should stay within follow-up joint-velocity budget") &&
           expect(peak_foot_vel_delta <= 0.12,
                  "startup transient peak foot velocity should be insensitive to prior stand/walk/stand history") &&
           expect(first_cycle_joint_vel_delta <= 8.0,
                  "first-cycle startup joint transient may differ by history but should remain bounded") &&
           expect(followup_peak_joint_vel_delta <= 2.5,
                  "post-startup joint transient peak should remain bounded despite expected gait-phase history effects") &&
           expect(first_command_delta_m <= 0.05,
                  "first motion command output should only vary modestly with expected stateful history effects");
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
    if (!scenario_quick_mode_toggles_preserve_phase_order_with_elapsed_gating()) {
        return EXIT_FAILURE;
    }
    if (!scenario_motion_limiter_config_changes_clamp_behavior()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_scales_with_dt_and_stays_bounded()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_yaw_clamp_uses_rate_limit_semantics()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_replay_is_deterministic_for_fixed_dt()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_ignores_estimated_angular_velocity_spikes()) {
        return EXIT_FAILURE;
    }
    if (!unit_motion_limiter_rejects_non_finite_commands()) {
        return EXIT_FAILURE;
    }
    if (!integration_multi_clamp_same_cycle_remains_bounded_without_spikes()) {
        return EXIT_FAILURE;
    }
    if (!scenario_first_motion_startup_transients_not_history_sensitive()) {
        return EXIT_FAILURE;
    }
    if (!integration_gait_scheduler_progress_tracks_elapsed_dt()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
