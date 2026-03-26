#include "joint_oscillation_tracker.hpp"

#include <cmath>

namespace {

double shortestAngularDelta(double current, double previous) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    double delta = current - previous;
    delta = std::fmod(delta + kPi, kTwoPi);
    if (delta < 0.0) {
        delta += kTwoPi;
    }
    return delta - kPi;
}

} // namespace

JointOscillationTracker::JointOscillationTracker(double min_delta_rad_for_reversal,
                                                 uint64_t min_reversal_interval_us)
    : min_delta_rad_for_reversal_(min_delta_rad_for_reversal > 0.0 ? min_delta_rad_for_reversal : 0.0),
      min_reversal_interval_us_(min_reversal_interval_us) {}

void JointOscillationTracker::reset() {
    has_previous_ = false;
    last_timestamp_ = TimePointUs{};
    previous_targets_ = JointTargets{};
    previous_direction_.fill(0);
    previous_delta_rad_.fill(0.0);
    last_reversal_timestamp_.fill(TimePointUs{});
    metrics_ = JointOscillationMetrics{};
}

std::size_t JointOscillationTracker::flattenIndex(std::size_t leg_idx, std::size_t joint_idx) const {
    return leg_idx * kJointsPerLeg + joint_idx;
}

void JointOscillationTracker::observe(const JointTargets& targets, TimePointUs now) {
    if (!has_previous_) {
        previous_targets_ = targets;
        last_timestamp_ = now;
        has_previous_ = true;
        return;
    }

    const int64_t dt_us_signed = static_cast<int64_t>(now.value) - static_cast<int64_t>(last_timestamp_.value);
    if (dt_us_signed <= 0) {
        previous_targets_ = targets;
        last_timestamp_ = now;
        return;
    }
    const double dt_s = static_cast<double>(dt_us_signed) / 1'000'000.0;

    for (std::size_t leg_idx = 0; leg_idx < kNumLegs; ++leg_idx) {
        for (std::size_t joint_idx = 0; joint_idx < kJointsPerLeg; ++joint_idx) {
            const std::size_t idx = flattenIndex(leg_idx, joint_idx);

            const double current = targets.leg_states[leg_idx].joint_state[joint_idx].pos_rad.value;
            const double previous = previous_targets_.leg_states[leg_idx].joint_state[joint_idx].pos_rad.value;
            const double delta = shortestAngularDelta(current, previous);
            const double abs_delta = std::abs(delta);
            const double velocity_radps = abs_delta / dt_s;
            if (velocity_radps > metrics_.peak_joint_velocity_radps) {
                metrics_.peak_joint_velocity_radps = velocity_radps;
            }

            int8_t direction = 0;
            if (delta > 0.0) {
                direction = 1;
            } else if (delta < 0.0) {
                direction = -1;
            }

            const bool significant_delta = abs_delta >= min_delta_rad_for_reversal_;
            const bool previous_significant_delta = std::abs(previous_delta_rad_[idx]) >= min_delta_rad_for_reversal_;
            const bool has_direction_reversal = direction != 0 &&
                                                previous_direction_[idx] != 0 &&
                                                direction != previous_direction_[idx];
            const bool interval_elapsed =
                min_reversal_interval_us_ == 0 ||
                last_reversal_timestamp_[idx].isZero() ||
                now.value >= (last_reversal_timestamp_[idx].value + min_reversal_interval_us_);
            if (has_direction_reversal && significant_delta && previous_significant_delta && interval_elapsed) {
                ++metrics_.direction_reversal_events;
                last_reversal_timestamp_[idx] = now;
            }

            if (direction != 0) {
                previous_direction_[idx] = direction;
                previous_delta_rad_[idx] = delta;
            }
        }
    }

    previous_targets_ = targets;
    last_timestamp_ = now;
}

JointOscillationMetrics JointOscillationTracker::metrics() const {
    return metrics_;
}
