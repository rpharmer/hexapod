#include "joint_oscillation_tracker.hpp"

#include <cmath>

JointOscillationTracker::JointOscillationTracker(double min_delta_rad_for_reversal)
    : min_delta_rad_for_reversal_(min_delta_rad_for_reversal > 0.0 ? min_delta_rad_for_reversal : 0.0) {}

void JointOscillationTracker::reset() {
    has_previous_ = false;
    last_timestamp_ = TimePointUs{};
    previous_targets_ = JointTargets{};
    previous_direction_.fill(0);
    previous_delta_rad_.fill(0.0);
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
            const double delta = current - previous;
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
            if (has_direction_reversal && significant_delta && previous_significant_delta) {
                ++metrics_.direction_reversal_events;
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
