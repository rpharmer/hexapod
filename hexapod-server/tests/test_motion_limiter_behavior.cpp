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
    BodyController controller;
    RobotState est = nominalEstimatedState();
    GaitState gait{};
    gait.in_stance.fill(true);
    RuntimeGaitPolicy policy{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent idle{};
    idle.requested_mode = RobotMode::SAFE_IDLE;
    controller.update(est, idle, gait, policy, safety);

    MotionIntent walk = walkingIntent();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    controller.update(est, walk, gait, policy, safety);
    const auto start_diag = controller.lastMotionLimiterTelemetry();

    walk.body_pose_setpoint.body_trans_m.x = 0.25;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    controller.update(est, walk, gait, policy, safety);
    const auto limited_diag = controller.lastMotionLimiterTelemetry();

    MotionIntent stop = walk;
    stop.requested_mode = RobotMode::SAFE_IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    controller.update(est, stop, gait, policy, safety);
    const auto stop_diag = controller.lastMotionLimiterTelemetry();

    return expect(start_diag.enabled, "limiter telemetry should be enabled during transition") &&
           expect(limited_diag.hard_clamp_linear, "large body step should activate linear hard clamp") &&
           expect(limited_diag.adaptation_scale_linear < 1.0, "linear adaptation scale should drop below unity when clamped") &&
           expect(stop_diag.phase == 2, "stop transition should report legs-lead phase");
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
    BodyController controller;
    RobotState est = nominalEstimatedState();
    GaitState gait{};
    gait.in_stance.fill(true);
    RuntimeGaitPolicy policy{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent walk = walkingIntent();
    const LegTargets first = controller.update(est, walk, gait, policy, safety);

    walk.body_pose_setpoint.body_trans_m.x = 0.35;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const LegTargets second = controller.update(est, walk, gait, policy, safety);

    const double dt = (second.timestamp_us.value - first.timestamp_us.value) / 1'000'000.0;
    const double dx = std::abs(second.feet[0].pos_body_m.x - first.feet[0].pos_body_m.x);
    const double observed_speed = dx / std::max(dt, 1e-6);

    return expect(observed_speed <= 0.5,
                  "step command response should remain bounded by transition ramp speed");
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
    return EXIT_SUCCESS;
}
