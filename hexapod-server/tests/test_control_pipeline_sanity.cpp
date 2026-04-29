#include "control_pipeline.hpp"

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

} // namespace

int main() {
    ControlPipeline pipeline;

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
        !expect(attenuated.command_governor.command_scale > 0.25,
                "aggressive command should be reduced progressively rather than stopped") ||
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

    return EXIT_SUCCESS;
}
