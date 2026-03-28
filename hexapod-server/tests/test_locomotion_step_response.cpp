#include "control_config.hpp"
#include "control_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RobotState nominalEstimate() {
    RobotState est{};
    est.timestamp_us = now_us();
    est.bus_ok = true;
    est.valid = true;
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }
    return est;
}

MotionIntent walkingIntent() {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = GaitType::TRIPOD;
    intent.speed_mps = LinearRateMps{0.16};
    intent.heading_rad = AngleRad{0.0};
    intent.body_pose_setpoint.body_trans_m.z = 0.20;
    intent.body_pose_setpoint.angular_velocity_radps.z = 0.0;
    intent.timestamp_us = now_us();
    intent.sample_id = 1;
    return intent;
}

SafetyState nominalSafety() {
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.stable = true;
    safety.support_contact_count = 6;
    safety.stability_margin_m = 0.03;
    safety.active_fault = FaultCode::NONE;
    safety.leg_enabled = {true, true, true, true, true, true};
    return safety;
}

double footVelocityMagnitude(const FootTarget& target) {
    const Vec3 vel = target.vel_body_mps;
    return std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
}

double commandEnergy(const PipelineStepResult& result) {
    double sum = 0.0;
    for (const auto& foot : result.leg_targets.feet) {
        sum += footVelocityMagnitude(foot);
    }
    return sum / static_cast<double>(kNumLegs);
}

bool commandSettlesWithinCycles(const std::vector<double>& signal,
                                int step_cycle,
                                int segment_end_cycle,
                                int hold_cycles,
                                int max_settle_cycles,
                                double tolerance) {
    if (segment_end_cycle - step_cycle <= hold_cycles + 15) {
        return false;
    }

    const int reference_window_start = std::max(step_cycle + 1, segment_end_cycle - 15);
    const int reference_window_count = segment_end_cycle - reference_window_start;
    if (reference_window_count <= 0) {
        return false;
    }

    const double reference =
        std::accumulate(signal.begin() + reference_window_start, signal.begin() + segment_end_cycle, 0.0) /
        static_cast<double>(reference_window_count);

    for (int cycle = step_cycle; cycle + hold_cycles < segment_end_cycle; ++cycle) {
        bool stable = true;
        for (int k = 0; k < hold_cycles; ++k) {
            if (std::abs(signal[cycle + k] - reference) > tolerance) {
                stable = false;
                break;
            }
        }
        if (stable) {
            return (cycle - step_cycle) <= max_settle_cycles;
        }
    }

    return false;
}

bool testWalkHeadingAndYawRateStepResponse() {
    ControlPipeline pipeline{};
    RobotState est = nominalEstimate();
    MotionIntent intent = walkingIntent();
    SafetyState safety = nominalSafety();

    constexpr int kTotalCycles = 150;
    constexpr int kStep1Cycle = 35;
    constexpr int kStep2Cycle = 90;
    constexpr DurationSec kLoopDt{0.02};

    constexpr double kLegVelocityPeakThresholdMps =
        control_config::kDefaultMotionFootVelocityLimitMps * 1.10;
    constexpr double kJointVelocityPeakThresholdRadps =
        control_config::kDefaultMotionJointSoftVelocityLimitRadps * 1.25;

    constexpr int kSettlingHoldCycles = 5;
    constexpr int kMaxSettlingCycles = 45;
    constexpr double kCommandSettlingTolerance = 0.08;

    std::vector<double> command_signal;
    command_signal.reserve(kTotalCycles);

    std::array<std::array<double, kJointsPerLeg>, kNumLegs> previous_joint_positions{};
    bool has_previous_joint_positions = false;

    double peak_leg_velocity_mps = 0.0;
    double peak_joint_velocity_radps = 0.0;

    for (int cycle = 0; cycle < kTotalCycles; ++cycle) {
        if (cycle == kStep1Cycle) {
            intent.heading_rad = AngleRad{0.70};
            intent.body_pose_setpoint.angular_velocity_radps.z = 0.65;
        } else if (cycle == kStep2Cycle) {
            intent.heading_rad = AngleRad{-0.45};
            intent.body_pose_setpoint.angular_velocity_radps.z = -0.35;
        }

        intent.sample_id += 1;
        intent.timestamp_us = now_us();
        est.timestamp_us = now_us();

        const PipelineStepResult step_result =
            pipeline.runStep(est, intent, safety, kLoopDt, true, static_cast<uint64_t>(cycle));

        if (!expect(step_result.status.active_mode == RobotMode::WALK,
                    "nominal walking step test should stay in WALK mode") ||
            !expect(step_result.status.active_fault == FaultCode::NONE,
                    "nominal walking step test should not introduce safety faults") ||
            !expect(step_result.status.bus_ok,
                    "nominal walking step test should keep bus health true")) {
            return false;
        }

        for (int leg = 0; leg < kNumLegs; ++leg) {
            peak_leg_velocity_mps = std::max(peak_leg_velocity_mps,
                                             footVelocityMagnitude(step_result.leg_targets.feet[leg]));

            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double position = step_result.joint_targets.leg_states[leg].joint_state[joint].pos_rad.value;
                if (has_previous_joint_positions) {
                    const double vel_radps =
                        std::abs(position - previous_joint_positions[leg][joint]) / kLoopDt.value;
                    peak_joint_velocity_radps = std::max(peak_joint_velocity_radps, vel_radps);
                }
                previous_joint_positions[leg][joint] = position;
            }
        }

        has_previous_joint_positions = true;
        command_signal.push_back(commandEnergy(step_result));

        est.leg_states = step_result.joint_targets.leg_states;
    }

    const bool settled_after_step1 = commandSettlesWithinCycles(
        command_signal, kStep1Cycle, kStep2Cycle, kSettlingHoldCycles, kMaxSettlingCycles, kCommandSettlingTolerance);
    const bool settled_after_step2 = commandSettlesWithinCycles(
        command_signal, kStep2Cycle, kTotalCycles, kSettlingHoldCycles, kMaxSettlingCycles, kCommandSettlingTolerance);

    return expect(peak_leg_velocity_mps < kLegVelocityPeakThresholdMps,
                  "leg target velocity peaks should remain under step-response threshold") &&
           expect(peak_joint_velocity_radps < kJointVelocityPeakThresholdRadps,
                  "joint command velocity peaks should remain under step-response threshold") &&
           expect(settled_after_step1,
                  "command should settle within expected cycles after first heading+yaw-rate step") &&
           expect(settled_after_step2,
                  "command should settle within expected cycles after second heading+yaw-rate step");
}

} // namespace

int main() {
    if (!testWalkHeadingAndYawRateStepResponse()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
