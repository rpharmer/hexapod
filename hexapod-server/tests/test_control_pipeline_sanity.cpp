#include "control_pipeline.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

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

bool allFinite(const LegTargets& targets) {
    for (const auto& foot : targets.feet) {
        if (!std::isfinite(foot.pos_body_m.x) ||
            !std::isfinite(foot.pos_body_m.y) ||
            !std::isfinite(foot.pos_body_m.z) ||
            !std::isfinite(foot.vel_body_mps.x) ||
            !std::isfinite(foot.vel_body_mps.y) ||
            !std::isfinite(foot.vel_body_mps.z)) {
            return false;
        }
    }
    return true;
}

bool dutyCycleClampCasesRemainStable() {
    constexpr std::array<double, 4> kDutyCycleCases{0.05, 0.10, 0.90, 0.95};
    constexpr double kMaxJointMagnitudeRad = 3.2;
    constexpr double kMaxFootCommandMagnitudeM = 0.70;
    constexpr int kCycleSteps = 360;
    setenv("HEXAPOD_BYPASS_CONTACT_MANAGER", "1", 1);

    for (const double configured_duty_cycle : kDutyCycleCases) {
        control_config::ControlConfig cfg{};
        cfg.gait.duty.tripod = configured_duty_cycle;

        ControlPipeline pipeline{cfg};
        RobotState estimated{};
        estimated.timestamp_us = now_us();
        estimated.foot_contacts = {true, true, true, true, true, true};
        for (auto& leg : estimated.leg_states) {
            leg.joint_state[COXA].pos_rad = AngleRad{0.0};
            leg.joint_state[FEMUR].pos_rad = AngleRad{-0.6};
            leg.joint_state[TIBIA].pos_rad = AngleRad{-0.8};
        }

        MotionIntent walk_intent{};
        walk_intent.requested_mode = RobotMode::WALK;
        walk_intent.gait = GaitType::TRIPOD;
        walk_intent.speed_mps = LinearRateMps{0.16};
        walk_intent.body_pose_setpoint.body_trans_mps = VelocityMps3{0.16, 0.0, 0.0};

        SafetyState safety{};
        safety.inhibit_motion = false;
        safety.torque_cut = false;
        safety.stable = true;
        safety.active_fault = FaultCode::NONE;

        int stance_samples = 0;
        int swing_samples = 0;

        for (int step = 0; step < kCycleSteps; ++step) {
            walk_intent.timestamp_us = now_us();
            const PipelineStepResult result =
                pipeline.runStep(estimated, walk_intent, safety, DurationSec{0.02}, true, static_cast<uint64_t>(step));

            if (!expect(result.status.active_fault == FaultCode::NONE,
                        "nominal locomotion should not raise safety faults") ||
                !expect(result.status.active_mode != RobotMode::FAULT,
                        "nominal locomotion should never enter FAULT mode") ||
                !expect(allFinite(result.leg_targets), "foot command targets should remain finite") ||
                !expect(allFinite(result.joint_targets), "joint command targets should remain finite")) {
                return false;
            }

            for (int leg = 0; leg < kNumLegs; ++leg) {
                const double phase = result.status.dynamic_gait.leg_phase[leg];
                const double duty =
                    std::clamp(result.status.dynamic_gait.leg_duty_cycle[leg], 0.05, 0.95);
                const bool in_stance = result.status.dynamic_gait.leg_in_stance[leg];
                const bool expected_in_stance = (phase < duty);

                const Vec3 foot_position = static_cast<Vec3>(result.leg_targets.feet[leg].pos_body_m);
                const double foot_command_mag =
                    std::sqrt((foot_position.x * foot_position.x) +
                              (foot_position.y * foot_position.y) +
                              (foot_position.z * foot_position.z));

                if (!expect(phase >= 0.0 && phase <= 1.0, "phase should remain in [0, 1]") ||
                    !expect(duty >= 0.05 && duty <= 0.95, "duty cycle should remain clamped to [0.05, 0.95]") ||
                    !expect(in_stance == expected_in_stance,
                            "stance/swing partition should follow phase-vs-duty rule") ||
                    !expect(foot_command_mag < kMaxFootCommandMagnitudeM,
                            "foot command magnitude should remain bounded")) {
                    return false;
                }

                stance_samples += in_stance ? 1 : 0;
                swing_samples += in_stance ? 0 : 1;

                for (const auto& joint : result.joint_targets.leg_states[leg].joint_state) {
                    if (!expect(std::abs(joint.pos_rad.value) < kMaxJointMagnitudeRad,
                                "joint command magnitude should remain bounded")) {
                        return false;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }

        if (!expect(stance_samples > 0, "short locomotion cycle should include stance samples") ||
            !expect(swing_samples > 0, "short locomotion cycle should include swing samples")) {
            return false;
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
    walk_intent.timestamp_us = now_us();

    SafetyState healthy_safety{};
    healthy_safety.inhibit_motion = false;
    healthy_safety.active_fault = FaultCode::NONE;

    const DurationSec loop_dt{0.02};

    const PipelineStepResult nominal = pipeline.runStep(
        estimated, walk_intent, healthy_safety, loop_dt, true, 42);

    if (!expect(nominal.status.active_mode == RobotMode::WALK, "pipeline should keep WALK mode when safety is healthy") ||
        !expect(nominal.status.estimator_valid, "estimator should be valid for non-zero timestamp") ||
        !expect(nominal.status.bus_ok, "status should propagate bus health") ||
        !expect(nominal.status.loop_counter == 42, "status should preserve loop counter") ||
        !expect(allFinite(nominal.joint_targets), "joint target outputs should remain finite")) {
        return EXIT_FAILURE;
    }

    SafetyState faulted_safety = healthy_safety;
    faulted_safety.active_fault = FaultCode::MOTOR_FAULT;

    const PipelineStepResult faulted = pipeline.runStep(
        estimated, walk_intent, faulted_safety, loop_dt, true, 43);

    if (!expect(faulted.status.active_mode == RobotMode::FAULT, "active safety fault should force FAULT mode") ||
        !expect(faulted.status.active_fault == FaultCode::MOTOR_FAULT, "status should expose active safety fault") ||
        !expect(allFinite(faulted.joint_targets), "faulted run should still produce finite joint targets")) {
        return EXIT_FAILURE;
    }

    if (!dutyCycleClampCasesRemainStable()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
