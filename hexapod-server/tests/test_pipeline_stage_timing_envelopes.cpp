#include "control_pipeline.hpp"
#include "estimator.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectBudget(const std::string& stage_name, const StageTimingEnvelope& envelope) {
    return expect(envelope.sample_count > 0, stage_name + " should produce timing samples") &&
           expect(envelope.p95_within_budget,
                  stage_name + " p95 exceeded budget: p95=" + std::to_string(envelope.p95_us) +
                      "us budget=" + std::to_string(envelope.budget_p95_us) + "us") &&
           expect(envelope.p99_within_budget,
                  stage_name + " p99 exceeded budget: p99=" + std::to_string(envelope.p99_us) +
                      "us budget=" + std::to_string(envelope.budget_p99_us) + "us");
}

RobotState makeRepresentativeRawState(int sample) {
    RobotState raw{};
    raw.sample_id = static_cast<uint64_t>(sample + 1);
    raw.timestamp_us = TimePointUs{1'000'000 + static_cast<uint64_t>(sample) * 2'000};
    raw.bus_ok = true;
    raw.has_measured_body_pose_state = true;
    raw.has_inferred_body_pose_state = false;
    raw.has_body_pose_state = true;
    raw.body_pose_state.orientation_rad = Vec3{
        0.04 * std::sin(static_cast<double>(sample) * 0.013),
        0.03 * std::cos(static_cast<double>(sample) * 0.017),
        0.02 * std::sin(static_cast<double>(sample) * 0.009)};
    raw.body_pose_state.body_trans_m = Vec3{0.0, 0.0, 0.20};
    raw.body_pose_state.body_trans_mps = Vec3{0.0, 0.0, 0.0};
    raw.body_pose_state.angular_velocity_radps = Vec3{0.0, 0.0, 0.0};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        raw.foot_contacts[leg] = ((sample + leg) % 3) != 0;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double phase = static_cast<double>(sample) * 0.02 + static_cast<double>(leg * 3 + joint) * 0.07;
            raw.leg_states[leg].joint_state[joint].pos_rad =
                AngleRad{0.25 * std::sin(phase) + 0.05 * std::cos(phase * 0.5)};
        }
    }
    return raw;
}

MotionIntent makeRepresentativeIntent(int sample) {
    MotionIntent intent{};
    intent.sample_id = static_cast<uint64_t>(sample + 1);
    intent.timestamp_us = TimePointUs{1'000'000 + static_cast<uint64_t>(sample) * 2'000};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = (sample % 5 == 0) ? GaitType::RIPPLE : GaitType::TRIPOD;
    intent.body_pose_setpoint.body_trans_m = Vec3{
        0.02 * std::sin(sample * 0.011),
        0.015 * std::cos(sample * 0.007),
        0.20 + 0.01 * std::sin(sample * 0.005)};
    intent.body_pose_setpoint.orientation_rad = Vec3{
        0.03 * std::sin(sample * 0.01),
        0.02 * std::cos(sample * 0.009),
        0.10 * std::sin(sample * 0.006)};
    intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.20 * std::sin(sample * 0.01)};
    return intent;
}

} // namespace

int main() {
    control_config::ControlConfig cfg{};
    cfg.pipeline_stage_timing.rolling_window_samples = 512;
    cfg.pipeline_stage_timing.estimator = control_config::StageTimingBudgetConfig{15'000, 20'000};
    cfg.pipeline_stage_timing.limiter = control_config::StageTimingBudgetConfig{8'000, 12'000};
    cfg.pipeline_stage_timing.gait = control_config::StageTimingBudgetConfig{8'000, 12'000};
    cfg.pipeline_stage_timing.body = control_config::StageTimingBudgetConfig{10'000, 15'000};
    cfg.pipeline_stage_timing.ik = control_config::StageTimingBudgetConfig{10'000, 15'000};

    ControlPipeline pipeline(cfg);
    SimpleEstimator estimator{};

    SafetyState safety{};
    safety.active_fault = FaultCode::NONE;
    safety.inhibit_motion = false;

    constexpr int kWarmupSamples = 64;
    constexpr int kMeasuredSamples = 700;
    const DurationSec loop_dt{0.002};

    for (int sample = 0; sample < (kWarmupSamples + kMeasuredSamples); ++sample) {
        const RobotState raw = makeRepresentativeRawState(sample);
        const auto estimator_start = std::chrono::steady_clock::now();
        const RobotState estimated = estimator.update(raw);
        const auto estimator_finish = std::chrono::steady_clock::now();
        if (sample >= kWarmupSamples) {
            pipeline.recordStageDuration(
                PipelineStage::Estimator,
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(estimator_finish - estimator_start).count()));
        }

        const MotionIntent intent = makeRepresentativeIntent(sample);
        (void)pipeline.runStep(estimated,
                               intent,
                               safety,
                               loop_dt,
                               true,
                               static_cast<uint64_t>(sample + 1));
    }

    const PipelineTimingSnapshot snapshot = pipeline.timingSnapshot();
    return (expectBudget("estimator", snapshot.estimator) &&
            expectBudget("limiter", snapshot.limiter) &&
            expectBudget("gait", snapshot.gait) &&
            expectBudget("body", snapshot.body) &&
            expectBudget("ik", snapshot.ik))
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
