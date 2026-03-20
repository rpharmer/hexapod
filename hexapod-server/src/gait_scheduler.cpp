#include "gait_scheduler.hpp"

#include "control_config.hpp"

#include <algorithm>
#include <array>

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

GaitState GaitScheduler::update(const EstimatedState&,
                                const MotionIntent& intent,
                                const SafetyState& safety) {
    GaitState out{};
    out.timestamp_us = now_us();

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    if (!walking) {
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
        }
        last_update_us_ = out.timestamp_us;
        return out;
    }

    const TimePointUs now = out.timestamp_us;
    if (last_update_us_.isZero()) {
        last_update_us_ = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us_).value) * 1e-6};
    last_update_us_ = now;

    // TODO(gait): replace fallback speed estimate with measured command magnitude.
    const double speed_mag = config_.fallback_speed_mag.value;

    const double step_hz = std::clamp(0.5 + 2.0 * speed_mag, 0.5, 2.5);
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
