#pragma once

#include "locomotion_metrics.hpp"
#include "motion_intent_utils.hpp"
#include "robot_runtime.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace locomotion_test {

struct MotionPhase {
    std::string label{};
    ScenarioMotionIntent motion{};
    std::size_t steps{0};
    bool refresh_each_step{true};
    /** When set, passed to `RobotRuntime::setSafetyLegEnabledTestMask` each control step in this phase. */
    std::optional<std::array<bool, kNumLegs>> safety_leg_enabled_mask{};
};

inline Vec3 positionFromState(const RobotState& state) {
    return Vec3{
        state.body_twist_state.body_trans_m.x,
        state.body_twist_state.body_trans_m.y,
        state.body_twist_state.body_trans_m.z,
    };
}

inline double horizontalSpeedFromState(const RobotState& state) {
    return std::hypot(state.body_twist_state.body_trans_mps.x,
                      state.body_twist_state.body_trans_mps.y);
}

inline double bodyTiltFromState(const RobotState& state) {
    if (!state.has_body_twist_state) {
        return 0.0;
    }
    return std::hypot(state.body_twist_state.twist_pos_rad.x, state.body_twist_state.twist_pos_rad.y);
}

inline double bodyRateFromState(const RobotState& state) {
    if (state.has_imu && state.imu.valid) {
        return std::hypot(state.imu.gyro_radps.x, state.imu.gyro_radps.y);
    }
    return std::hypot(state.body_twist_state.twist_vel_radps.x,
                      state.body_twist_state.twist_vel_radps.y);
}

/** Runs phased motion intent steps; fills samples/metrics (stride_count updated from phase rate). */
inline bool runMotionSequence(RobotRuntime& runtime,
                              const std::vector<MotionPhase>& phases,
                              std::vector<MotionSample>& samples,
                              LocomotionMetrics& metrics) {
    std::size_t step_index = 0;
    double stride_cycles_accum{0.0};
    for (std::size_t phase_index = 0; phase_index < phases.size(); ++phase_index) {
        const MotionPhase& phase = phases[phase_index];
        if (phase.steps == 0) {
            continue;
        }

        for (std::size_t step_in_phase = 0; step_in_phase < phase.steps; ++step_in_phase) {
            if (phase.refresh_each_step || step_in_phase == 0) {
                runtime.setMotionIntent(makeMotionIntent(phase.motion));
            }
            runtime.setSafetyLegEnabledTestMask(phase.safety_leg_enabled_mask);
            runtime.busStep();
            runtime.estimatorStep();
            runtime.safetyStep();
            runtime.controlStep();

            const RobotState estimated = runtime.estimatedSnapshot();
            const GaitState gait = runtime.gaitSnapshot();
            const ControlStatus status = runtime.getStatus();
            const SafetyState safety = runtime.getSafetyState();
            const CommandGovernorState governor = runtime.commandGovernorSnapshot();
            MotionSample sample{};
            sample.step_index = step_index;
            sample.phase_index = phase_index;
            sample.phase_label = phase.label;
            sample.requested_motion = phase.motion;
            sample.status = status;
            sample.estimated = estimated;
            sample.gait = gait;
            sample.governor = governor;
            sample.safety = safety;
            sample.position = positionFromState(estimated);
            sample.horizontal_speed_mps = horizontalSpeedFromState(estimated);
            sample.body_tilt_rad = bodyTiltFromState(estimated);
            sample.body_rate_radps = bodyRateFromState(estimated);
            sample.yaw_rate_radps = estimated.body_twist_state.twist_vel_radps.z;
            sample.support_margin_m = gait.static_stability_margin_m;
            sample.stride_phase_rate_hz = gait.stride_phase_rate_hz.value;
            sample.step_length_m = gait.step_length_m;
            sample.swing_height_m = gait.swing_height_m;
            sample.duty_factor = gait.duty_factor;
            sample.model_trust = estimated.has_fusion_diagnostics ? estimated.fusion.model_trust : 1.0;
            sample.contact_mismatch_ratio =
                estimated.has_fusion_diagnostics ? estimated.fusion.residuals.contact_mismatch_ratio : 0.0;
            sample.locomotion_debug = runtime.locomotionDebugSnapshot();
            sample.sample_period_s = metrics.sample_period_s;

            if (sample.status.active_mode == RobotMode::WALK && gait.stride_phase_rate_hz.value > 0.05) {
                stride_cycles_accum += gait.stride_phase_rate_hz.value * metrics.sample_period_s;
            }
            appendSample(metrics, samples, sample);
            ++step_index;
        }
    }

    finalizeMetrics(metrics, samples);
    metrics.stride_count = std::max<std::size_t>(
        metrics.stride_count,
        static_cast<std::size_t>(std::floor(stride_cycles_accum)));
    return true;
}

} // namespace locomotion_test
