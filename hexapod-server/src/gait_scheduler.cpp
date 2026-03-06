#include "gait_scheduler.hpp"

#include <algorithm>

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

    const uint64_t now = out.timestamp_us;
    if (last_update_us_ == 0) {
        last_update_us_ = now;
    }

    const double dt = static_cast<double>(now - last_update_us_) * 1e-6;
    last_update_us_ = now;

    // Simple speed-dependent step frequency.
    /*const double speed_mag =
        std::sqrt(intent.twist.vx_mps * intent.twist.vx_mps +
                  intent.twist.vy_mps * intent.twist.vy_mps) +
        std::abs(intent.twist.wz_radps) * 0.1;*/
    const double speed_mag = 0.01;   

    const double step_hz = std::clamp(0.5 + 2.0 * speed_mag, 0.5, 2.5);
    out.stride_phase_rate_hz = step_hz;

    phase_accum_ = wrap01(phase_accum_ + dt * step_hz);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        double offset = 0.0;

        switch (intent.gait) {
            case GaitType::TRIPOD:
                offset = (leg % 2 == 0) ? 0.0 : 0.5;
                break;
            case GaitType::RIPPLE:
                offset = leg / 6.0;
                break;
            case GaitType::WAVE:
                offset = leg / 6.0;
                break;
        }

        const double p = wrap01(phase_accum_ + offset);
        out.phase[leg] = p;

        // 0.0..0.5 stance, 0.5..1.0 swing
        out.in_stance[leg] = (p < 0.5);
    }

    return out;
}