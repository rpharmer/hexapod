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

double absMaxVerticalFootPositionDelta(const LegTargets& previous, const LegTargets& current) {
    double max_delta = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        max_delta = std::max(max_delta,
                             std::abs(current.feet[leg].pos_body_m.z - previous.feet[leg].pos_body_m.z));
    }
    return max_delta;
}

double absMaxVerticalFootVelocity(const LegTargets& current) {
    double max_abs_velocity = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        max_abs_velocity = std::max(max_abs_velocity, std::abs(current.feet[leg].vel_body_mps.z));
    }
    return max_abs_velocity;
}

struct TrajectoryMetrics {
    double distance_proxy_m{0.0};
    double cadence_min_hz{std::numeric_limits<double>::infinity()};
    double cadence_max_hz{0.0};
    double cadence_mean_hz{0.0};
    double peak_leg_velocity_mps{0.0};
    bool saw_fault{false};
};

double percentile(std::vector<double>& values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    const double clamped = std::clamp(p, 0.0, 1.0);
    const std::size_t index =
        static_cast<std::size_t>(std::lround(clamped * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

TrajectoryMetrics runEquivalentTrajectoryAtRate(DurationSec loop_dt, int total_cycles) {
    ControlPipeline pipeline{};
    RobotState est = nominalEstimate();
    MotionIntent intent = walkingIntent();
    SafetyState safety = nominalSafety();

    double cadence_sum_hz = 0.0;
    std::vector<double> leg_velocity_samples;
    leg_velocity_samples.reserve(static_cast<std::size_t>(total_cycles) * kNumLegs);

    TrajectoryMetrics metrics{};

    for (int cycle = 0; cycle < total_cycles; ++cycle) {
        const double t = static_cast<double>(cycle) * loop_dt.value;
        intent.speed_mps = LinearRateMps{0.16 + 0.02 * std::sin(2.0 * kPi * 0.40 * t)};
        intent.heading_rad = AngleRad{0.35 * std::sin(2.0 * kPi * 0.23 * t)};
        intent.body_pose_setpoint.angular_velocity_radps.z = 0.45 * std::sin(2.0 * kPi * 0.31 * t);
        intent.body_pose_setpoint.body_trans_m.z = 0.20 + 0.008 * std::sin(2.0 * kPi * 0.17 * t);

        intent.sample_id += 1;
        intent.timestamp_us = now_us();
        est.timestamp_us = now_us();

        const PipelineStepResult step_result =
            pipeline.runStep(est, intent, safety, loop_dt, true, static_cast<uint64_t>(cycle));
        metrics.saw_fault = metrics.saw_fault || (step_result.status.active_fault != FaultCode::NONE);

        const double cadence_hz = std::max(0.0, step_result.status.dynamic_gait.cadence_hz);
        metrics.cadence_min_hz = std::min(metrics.cadence_min_hz, cadence_hz);
        metrics.cadence_max_hz = std::max(metrics.cadence_max_hz, cadence_hz);
        cadence_sum_hz += cadence_hz;

        double command_sum = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double foot_vel = footVelocityMagnitude(step_result.leg_targets.feet[leg]);
            leg_velocity_samples.push_back(foot_vel);
            command_sum += foot_vel;
        }

        const double mean_leg_command_mps = command_sum / static_cast<double>(kNumLegs);
        metrics.distance_proxy_m += mean_leg_command_mps * loop_dt.value;
        est.leg_states = step_result.joint_targets.leg_states;
    }

    if (total_cycles > 0) {
        metrics.cadence_mean_hz = cadence_sum_hz / static_cast<double>(total_cycles);
    }
    if (!std::isfinite(metrics.cadence_min_hz)) {
        metrics.cadence_min_hz = 0.0;
    }
    metrics.peak_leg_velocity_mps = percentile(leg_velocity_samples, 0.99);

    return metrics;
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

bool testBodyHeightStepAndRampBoundedInStandAndWalk() {
    ControlPipeline pipeline{};
    RobotState est = nominalEstimate();
    MotionIntent intent = walkingIntent();
    SafetyState safety = nominalSafety();

    constexpr DurationSec kLoopDt{0.02};
    constexpr int kStandCycles = 80;
    constexpr int kWalkCycles = 80;
    constexpr int kStepCycle = 18;
    constexpr int kRampStartCycle = 45;
    constexpr int kRampDurationCycles = 20;
    constexpr double kStandHeightM = 0.20;
    constexpr double kStepHeightM = 0.245;
    constexpr double kRampTargetHeightM = 0.175;
    constexpr double kLegVerticalVelocityLimitMps = control_config::kDefaultMotionFootVelocityLimitMps * 1.10;
    constexpr double kJointVelocityLimitRadps =
        control_config::kDefaultMotionJointSoftVelocityLimitRadps * 1.25;
    constexpr double kVerticalPosDeltaBoundM = 0.060;

    struct SegmentMetrics {
        double peak_abs_vertical_vel_mps{0.0};
        double peak_abs_vertical_pos_delta_m{0.0};
        double peak_joint_velocity_radps{0.0};
        bool saw_fault{false};
    };

    auto runSegment = [&](RobotMode mode, int cycles, uint64_t cycle_offset) {
        std::array<std::array<double, kJointsPerLeg>, kNumLegs> previous_joint_positions{};
        bool has_previous_joint_positions = false;
        LegTargets previous_targets{};
        bool has_previous_targets = false;

        SegmentMetrics metrics{};

        intent.requested_mode = mode;
        if (mode == RobotMode::WALK) {
            intent.gait = GaitType::TRIPOD;
            intent.speed_mps = LinearRateMps{0.16};
            intent.heading_rad = AngleRad{0.0};
        } else {
            intent.speed_mps = LinearRateMps{0.0};
        }

        for (int cycle = 0; cycle < cycles; ++cycle) {
            if (cycle == 0) {
                intent.body_pose_setpoint.body_trans_m.z = kStandHeightM;
            } else if (cycle == kStepCycle) {
                intent.body_pose_setpoint.body_trans_m.z = kStepHeightM;
            } else if (cycle >= kRampStartCycle && cycle < (kRampStartCycle + kRampDurationCycles)) {
                const double t =
                    static_cast<double>(cycle - kRampStartCycle + 1) / static_cast<double>(kRampDurationCycles);
                intent.body_pose_setpoint.body_trans_m.z = kStepHeightM + ((kRampTargetHeightM - kStepHeightM) * t);
            } else if (cycle >= (kRampStartCycle + kRampDurationCycles)) {
                intent.body_pose_setpoint.body_trans_m.z = kRampTargetHeightM;
            }

            intent.sample_id += 1;
            intent.timestamp_us = now_us();
            est.timestamp_us = now_us();

            const PipelineStepResult step_result = pipeline.runStep(
                est, intent, safety, kLoopDt, true, cycle_offset + static_cast<uint64_t>(cycle));

            metrics.saw_fault = metrics.saw_fault || (step_result.status.active_fault != FaultCode::NONE);
            metrics.peak_abs_vertical_vel_mps =
                std::max(metrics.peak_abs_vertical_vel_mps, absMaxVerticalFootVelocity(step_result.leg_targets));
            if (has_previous_targets) {
                metrics.peak_abs_vertical_pos_delta_m =
                    std::max(metrics.peak_abs_vertical_pos_delta_m,
                             absMaxVerticalFootPositionDelta(previous_targets, step_result.leg_targets));
            }

            for (int leg = 0; leg < kNumLegs; ++leg) {
                for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                    const double position = step_result.joint_targets.leg_states[leg].joint_state[joint].pos_rad.value;
                    if (has_previous_joint_positions) {
                        const double vel_radps =
                            std::abs(position - previous_joint_positions[leg][joint]) / kLoopDt.value;
                        metrics.peak_joint_velocity_radps = std::max(metrics.peak_joint_velocity_radps, vel_radps);
                    }
                    previous_joint_positions[leg][joint] = position;
                }
            }
            has_previous_joint_positions = true;
            previous_targets = step_result.leg_targets;
            has_previous_targets = true;
            est.leg_states = step_result.joint_targets.leg_states;
        }

        return metrics;
    };

    const auto stand_metrics = runSegment(RobotMode::STAND, kStandCycles, 0);
    const auto walk_metrics = runSegment(RobotMode::WALK, kWalkCycles, static_cast<uint64_t>(kStandCycles));

    return expect(!stand_metrics.saw_fault,
                  "stand body height step/ramp should not trigger transient safety faults") &&
           expect(stand_metrics.peak_abs_vertical_vel_mps < kLegVerticalVelocityLimitMps,
                  "stand body height step/ramp should keep vertical foot velocity bounded") &&
           expect(stand_metrics.peak_abs_vertical_pos_delta_m < kVerticalPosDeltaBoundM,
                  "stand body height step/ramp should keep vertical foot target transitions bounded") &&
           expect(stand_metrics.peak_joint_velocity_radps < kJointVelocityLimitRadps,
                  "stand body height step/ramp should not create joint spikes") &&
           expect(!walk_metrics.saw_fault,
                  "walk body height step/ramp should not trigger transient safety faults") &&
           expect(walk_metrics.peak_abs_vertical_vel_mps < kLegVerticalVelocityLimitMps,
                  "walk body height step/ramp should keep vertical foot velocity bounded") &&
           expect(walk_metrics.peak_abs_vertical_pos_delta_m < kVerticalPosDeltaBoundM,
                  "walk body height step/ramp should keep vertical foot target transitions bounded") &&
           expect(walk_metrics.peak_joint_velocity_radps < kJointVelocityLimitRadps,
                  "walk body height step/ramp should not create joint spikes");
}

bool testEquivalentTrajectoryMetricsStayStableAcrossLoopRates() {
    constexpr double kTrajectoryDurationSec = 3.0;
    constexpr DurationSec kFastDt{1.0 / 250.0};
    constexpr DurationSec kSlowDt{1.0 / 125.0};
    const int fast_cycles = static_cast<int>(std::lround(kTrajectoryDurationSec / kFastDt.value));
    const int slow_cycles = static_cast<int>(std::lround(kTrajectoryDurationSec / kSlowDt.value));

    const TrajectoryMetrics fast = runEquivalentTrajectoryAtRate(kFastDt, fast_cycles);
    const TrajectoryMetrics slow = runEquivalentTrajectoryAtRate(kSlowDt, slow_cycles);

    const auto ratio = [](double a, double b) {
        const double denom = std::max(std::abs(a), std::abs(b));
        if (denom <= 1e-9) {
            return 1.0;
        }
        return std::abs(a - b) / denom;
    };

    constexpr double kDistanceProxyRatioTolerance = 0.14;
    constexpr double kCadenceEnvelopeTolerance = 0.10;
    constexpr double kCadenceMeanTolerance = 0.08;
    constexpr double kPeakLegRateTolerance = 0.12;

    return expect(!fast.saw_fault && !slow.saw_fault,
                  "equivalent trajectory replay at 250 Hz and 125 Hz should not trigger faults") &&
           expect(ratio(fast.distance_proxy_m, slow.distance_proxy_m) < kDistanceProxyRatioTolerance,
                  "distance proxy should remain proportionally consistent across equivalent trajectory loop rates") &&
           expect(ratio(fast.cadence_min_hz, slow.cadence_min_hz) < kCadenceEnvelopeTolerance,
                  "cadence envelope minimum should stay stable across equivalent trajectory loop rates") &&
           expect(ratio(fast.cadence_max_hz, slow.cadence_max_hz) < kCadenceEnvelopeTolerance,
                  "cadence envelope maximum should stay stable across equivalent trajectory loop rates") &&
           expect(ratio(fast.cadence_mean_hz, slow.cadence_mean_hz) < kCadenceMeanTolerance,
                  "cadence envelope mean should remain proportionally consistent across equivalent trajectory loop rates") &&
           expect(ratio(fast.peak_leg_velocity_mps, slow.peak_leg_velocity_mps) < kPeakLegRateTolerance,
                  "peak commanded foot velocity should remain stable across equivalent trajectory loop rates");
}

} // namespace

int main() {
    if (!testWalkHeadingAndYawRateStepResponse() ||
        !testBodyHeightStepAndRampBoundedInStandAndWalk() ||
        !testEquivalentTrajectoryMetricsStayStableAcrossLoopRates()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
