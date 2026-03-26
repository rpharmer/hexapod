#include "gait_scheduler.hpp"

#include "control_config.hpp"
#include "reach_envelope.hpp"
#include "stability_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr std::array<double, kNumLegs> kTripodPhaseOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
constexpr std::array<double, kNumLegs> kRipplePhaseOffsets = {0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};
constexpr std::array<double, kNumLegs> kWavePhaseOffsets = {0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};

const std::array<double, kNumLegs>& gait_phase_offsets(const GaitType gait)
{
    switch (gait) {
        case GaitType::TRIPOD:
            return kTripodPhaseOffsets;
        case GaitType::RIPPLE:
            return kRipplePhaseOffsets;
        case GaitType::WAVE:
            return kWavePhaseOffsets;
    }

    return kTripodPhaseOffsets;
}

} // namespace

GaitScheduler::GaitScheduler(control_config::GaitConfig config)
    : config_(config) {}

double GaitScheduler::wrap01(double x) const {
    while (x >= 1.0) x -= 1.0;
    while (x < 0.0) x += 1.0;
    return x;
}


double GaitScheduler::maxReachUtilization(const RobotState& est) const {
    double max_utilization = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const LegState joint_frame = leg_geo.servo.toJointAngles(est.leg_states[leg]);

        const double q1 = joint_frame.joint_state[0].pos_rad.value;
        const double q2 = joint_frame.joint_state[1].pos_rad.value;
        const double q3 = joint_frame.joint_state[2].pos_rad.value;

        const double rho =
            leg_geo.femurLength.value * std::cos(q2) +
            leg_geo.tibiaLength.value * std::cos(q2 + q3);
        const double z =
            leg_geo.femurLength.value * std::sin(q2) +
            leg_geo.tibiaLength.value * std::sin(q2 + q3);
        const double r = leg_geo.coxaLength.value + rho;
        const Vec3 foot_leg{r * std::cos(q1), r * std::sin(q1), z};

        max_utilization = std::max(max_utilization, kinematics::legReachUtilization(foot_leg, leg_geo));
    }

    return max_utilization;
}

GaitState GaitScheduler::update(const RobotState& est,
                                const MotionIntent& intent,
                                const SafetyState& safety) {
    GaitState out{};
    out.timestamp_us = now_us();

    const StabilityAssessment stability = assessStability(est);
    out.stable = stability.com_inside_support_polygon;
    out.support_contact_count = stability.support_contact_count;
    out.stability_margin_m = stability.stability_margin_m;

    const double reach_utilization = maxReachUtilization(est);
    const bool envelope_allows_progression = reach_utilization < 1.02;

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut &&
        out.stable &&
        envelope_allows_progression;

    if (!walking) {
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
        }
        out.stride_phase_rate_hz = FrequencyHz{0.0};
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    constexpr double kNominalMaxSpeedMps = 0.25;
    const double commanded_speed = std::abs(intent.speed_mps.value);
    const double normalized_command = std::clamp(commanded_speed / kNominalMaxSpeedMps, 0.0, 1.0);
    const double speed_mag = std::max(normalized_command, config_.fallback_speed_mag.value);
    const double envelope_speed_scale =
        std::clamp((1.0 - reach_utilization) / 0.15, 0.25, 1.0);

    const double step_hz = std::clamp(0.5 + 2.0 * speed_mag * envelope_speed_scale, 0.5, 2.5);
    const FrequencyHz step_rate_hz{step_hz};
    out.stride_phase_rate_hz = step_rate_hz;

    phase_accum_ = wrap01(phase_accum_ + dt.value * step_rate_hz.value);

    const std::array<double, kNumLegs>& phase_offsets = gait_phase_offsets(intent.gait);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double p = wrap01(phase_accum_ + phase_offsets[leg]);
        out.phase[leg] = p;

        // 0.0..0.5 stance, 0.5..1.0 swing
        out.in_stance[leg] = (p < 0.5);
    }

    return out;
}
