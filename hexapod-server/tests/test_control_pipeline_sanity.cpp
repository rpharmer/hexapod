#include "control_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool allFinite(const JointTargets& targets) {
    for (const auto& leg : targets.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool allTrue(const std::array<bool, kNumLegs>& values) {
    return std::all_of(values.begin(), values.end(), [](const bool value) { return value; });
}

} // namespace

int main() {
    control_config::SafetyConfig pipeline_safety{};
    pipeline_safety.body_height_collapse_margin_m = 0.05;
    ControlPipeline pipeline{{}, {}, pipeline_safety};
    control_config::CommandGovernorConfig canary_governor{};
    canary_governor.support_margin_soft_m = 0.12345;
    canary_governor.active_min_scale = 0.23456;
    ControlPipeline canary_pipeline{{}, {}, pipeline_safety, canary_governor};
    if (!expect(canary_pipeline.commandGovernorConfig().support_margin_soft_m == canary_governor.support_margin_soft_m,
                "pipeline should retain parsed governor support margin config") ||
        !expect(canary_pipeline.commandGovernorConfig().active_min_scale == canary_governor.active_min_scale,
                "pipeline should retain parsed governor active min scale config")) {
        return EXIT_FAILURE;
    }

    RobotState estimated{};
    estimated.timestamp_us = now_us();

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.gait = GaitType::TRIPOD;
    walk_intent.cmd_vx_mps = LinearRateMps{0.08};
    walk_intent.cmd_yaw_radps = AngularRateRadPerSec{0.0};
    walk_intent.timestamp_us = now_us();

    SafetyState healthy_safety{};
    healthy_safety.inhibit_motion = false;
    healthy_safety.active_fault = FaultCode::NONE;

    const PipelineStepResult nominal = pipeline.runStep(
        estimated, walk_intent, healthy_safety, true, 42);

    if (!expect(nominal.status.active_mode == RobotMode::WALK, "pipeline should keep WALK mode when safety is healthy") ||
        !expect(nominal.status.estimator_valid, "estimator should be valid for non-zero timestamp") ||
        !expect(nominal.status.bus_ok, "status should propagate bus health") ||
        !expect(nominal.command_governor.command_scale > 0.9,
                "healthy walk should pass through the command governor") ||
        !expect(nominal.status.loop_counter == 42, "status should preserve loop counter") ||
        !expect(allFinite(nominal.joint_targets), "joint target outputs should remain finite")) {
        return EXIT_FAILURE;
    }

    MotionIntent aggressive_intent = walk_intent;
    aggressive_intent.cmd_vx_mps = LinearRateMps{0.34};
    aggressive_intent.cmd_yaw_radps = AngularRateRadPerSec{0.72};
    aggressive_intent.timestamp_us = now_us();

    RobotState stressed_estimated = estimated;
    stressed_estimated.valid = true;
    stressed_estimated.has_body_twist_state = true;
    stressed_estimated.body_twist_state.twist_pos_rad.x = 0.24;
    stressed_estimated.body_twist_state.twist_pos_rad.y = 0.22;
    stressed_estimated.has_imu = true;
    stressed_estimated.imu.valid = true;
    stressed_estimated.imu.gyro_radps.x = 1.05;
    stressed_estimated.imu.gyro_radps.y = 0.95;
    stressed_estimated.has_fusion_diagnostics = true;
    stressed_estimated.fusion.model_trust = 0.55;
    stressed_estimated.fusion.residuals.contact_mismatch_ratio = 0.31;

    const PipelineStepResult attenuated = pipeline.runStep(
        stressed_estimated, aggressive_intent, healthy_safety, true, 43);

    if (!expect(attenuated.status.active_mode == RobotMode::WALK,
                "governor should attenuate aggressive commands without changing WALK mode") ||
        !expect(attenuated.command_governor.command_scale <= nominal.command_governor.command_scale,
                "aggressive command should not get a larger governor scale than the nominal case") ||
        !expect(attenuated.command_governor.command_scale > 0.25 || attenuated.command_governor.recovery_hold_active,
                "aggressive command should either slow down progressively or enter recovery hold") ||
        !expect(allFinite(attenuated.joint_targets), "attenuated run should still produce finite joint targets")) {
        return EXIT_FAILURE;
    }

    SafetyState faulted_safety = healthy_safety;
    faulted_safety.active_fault = FaultCode::MOTOR_FAULT;

    const PipelineStepResult faulted = pipeline.runStep(
        estimated, walk_intent, faulted_safety, true, 44);

    if (!expect(faulted.status.active_mode == RobotMode::FAULT, "active safety fault should force FAULT mode") ||
        !expect(faulted.status.active_fault == FaultCode::MOTOR_FAULT, "status should expose active safety fault") ||
        !expect(allFinite(faulted.joint_targets), "faulted run should still produce finite joint targets")) {
        return EXIT_FAILURE;
    }

    ControlPipeline recovery_pipeline{{}, {}, pipeline_safety};
    RobotState sparse_estimated = stressed_estimated;
    sparse_estimated.timestamp_us = TimePointUs{1'000'000};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const bool support = leg < 2;
        sparse_estimated.foot_contacts[static_cast<std::size_t>(leg)] = support;
        sparse_estimated.foot_contact_fusion[static_cast<std::size_t>(leg)].phase =
            support ? ContactPhase::ConfirmedStance : ContactPhase::Search;
    }

    MotionIntent recovery_intent = aggressive_intent;
    recovery_intent.timestamp_us = TimePointUs{1'000'000};
    const PipelineStepResult held = recovery_pipeline.runStep(
        sparse_estimated, recovery_intent, healthy_safety, true, 45);
    if (!expect(held.command_governor.recovery_stage == RecoveryStage::ActiveHold,
                "sparse unstable walk should enter active recovery hold in the pipeline") ||
        !expect(held.command_governor.recovery_hold_active,
                "pipeline recovery hold should surface as active") ||
        !expect(allTrue(held.gait_state.in_stance),
                "active recovery hold should force all legs into stance") ||
        !expect(held.gait_state.step_length_m == 0.0,
                "active recovery hold should zero the current gait step length")) {
        return EXIT_FAILURE;
    }

    RobotState healthy_support_estimated = estimated;
    healthy_support_estimated.valid = true;
    healthy_support_estimated.timestamp_us = TimePointUs{1'350'000};
    healthy_support_estimated.has_body_twist_state = true;
    healthy_support_estimated.body_twist_state.body_trans_m.z = 0.14;
    healthy_support_estimated.has_imu = true;
    healthy_support_estimated.imu.valid = true;
    healthy_support_estimated.has_fusion_diagnostics = true;
    healthy_support_estimated.fusion.model_trust = 0.95;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        healthy_support_estimated.foot_contacts[static_cast<std::size_t>(leg)] = true;
        healthy_support_estimated.foot_contact_fusion[static_cast<std::size_t>(leg)].phase =
            ContactPhase::ConfirmedStance;
    }

    MotionIntent gentle_walk = walk_intent;
    gentle_walk.cmd_vx_mps = LinearRateMps{0.05};
    gentle_walk.cmd_yaw_radps = AngularRateRadPerSec{0.0};
    gentle_walk.timestamp_us = TimePointUs{1'350'000};
    const PipelineStepResult healthy_hold = recovery_pipeline.runStep(
        healthy_support_estimated, gentle_walk, healthy_safety, true, 46);
    if (!expect(healthy_hold.command_governor.recovery_stage == RecoveryStage::ActiveHold,
                "the first healthy post-recovery frame should keep active hold while starting the healthy timer") ||
        !expect(allTrue(healthy_hold.gait_state.in_stance),
                "the first healthy post-recovery frame should still keep all legs in stance")) {
        return EXIT_FAILURE;
    }

    healthy_support_estimated.timestamp_us = TimePointUs{1'700'000};
    gentle_walk.timestamp_us = TimePointUs{1'700'000};
    const PipelineStepResult settling = recovery_pipeline.runStep(
        healthy_support_estimated, gentle_walk, healthy_safety, true, 47);
    if (!expect(settling.command_governor.recovery_stage == RecoveryStage::Settling,
                "healthy post-recovery frame should advance the pipeline into settling") ||
        !expect(allTrue(settling.gait_state.in_stance),
                "settling should keep all legs in stance")) {
        return EXIT_FAILURE;
    }

    healthy_support_estimated.timestamp_us = TimePointUs{2'000'000};
    gentle_walk.timestamp_us = TimePointUs{2'000'000};
    const PipelineStepResult ramp_out = recovery_pipeline.runStep(
        healthy_support_estimated, gentle_walk, healthy_safety, true, 48);
    if (!expect(ramp_out.command_governor.recovery_stage == RecoveryStage::RampOut,
                "settling should advance into ramp-out after the dwell window") ||
        !expect(!ramp_out.command_governor.freeze_phase,
                "ramp-out should release gait phase")) {
        return EXIT_FAILURE;
    }

    RobotState ramp_out_walk_estimated = healthy_support_estimated;
    ramp_out_walk_estimated.timestamp_us = TimePointUs{2'200'000};
    ramp_out_walk_estimated.body_twist_state.twist_pos_rad.x = 0.02;
    ramp_out_walk_estimated.body_twist_state.twist_pos_rad.y = 0.02;
    ramp_out_walk_estimated.imu.gyro_radps.x = 0.25;
    ramp_out_walk_estimated.imu.gyro_radps.y = 0.20;
    ramp_out_walk_estimated.fusion.model_trust = 0.95;
    ramp_out_walk_estimated.fusion.residuals.contact_mismatch_ratio = 0.03;
    ramp_out_walk_estimated.foot_contacts[0] = false;
    ramp_out_walk_estimated.foot_contact_fusion[0].phase = ContactPhase::Swing;

    MotionIntent ramp_out_walk_intent = gentle_walk;
    ramp_out_walk_intent.cmd_vx_mps = LinearRateMps{0.12};
    ramp_out_walk_intent.speed_mps = LinearRateMps{0.12};
    ramp_out_walk_intent.timestamp_us = TimePointUs{2'200'000};
    const PipelineStepResult ramp_out_walk = recovery_pipeline.runStep(
        ramp_out_walk_estimated, ramp_out_walk_intent, healthy_safety, true, 49);
    if (!expect(ramp_out_walk.command_governor.recovery_stage == RecoveryStage::RampOut,
                "healthy walking support during ramp-out should not bounce back into active hold") ||
        !expect(!ramp_out_walk.command_governor.recovery_hold_active,
                "healthy walking support during ramp-out should remain out of hold") ||
        !expect(!allTrue(ramp_out_walk.gait_state.in_stance),
                "ramp-out should allow gait progression instead of forcing all-stance")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
