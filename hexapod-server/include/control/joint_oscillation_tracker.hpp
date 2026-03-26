#pragma once

#include "types.hpp"

#include <array>
#include <cstdint>

struct JointOscillationMetrics {
    uint64_t direction_reversal_events{0};
    double peak_joint_velocity_radps{0.0};
};

class JointOscillationTracker {
public:
    explicit JointOscillationTracker(double min_delta_rad_for_reversal = 0.002);

    void reset();
    void observe(const JointTargets& targets, TimePointUs now);
    JointOscillationMetrics metrics() const;

private:
    static constexpr std::size_t kJointCount = kNumLegs * kJointsPerLeg;

    std::size_t flattenIndex(std::size_t leg_idx, std::size_t joint_idx) const;

    double min_delta_rad_for_reversal_{0.002};
    bool has_previous_{false};
    TimePointUs last_timestamp_{};
    JointTargets previous_targets_{};
    std::array<int8_t, kJointCount> previous_direction_{};
    std::array<double, kJointCount> previous_delta_rad_{};
    JointOscillationMetrics metrics_{};
};
