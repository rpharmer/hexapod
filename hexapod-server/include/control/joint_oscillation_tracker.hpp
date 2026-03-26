#pragma once

#include "types.hpp"

#include <array>
#include <cstdint>

struct JointOscillationMetrics {
    static constexpr std::size_t kJointCount = kNumLegs * kJointsPerLeg;
    uint64_t direction_reversal_events{0};
    double peak_joint_velocity_radps{0.0};
    std::array<uint64_t, kJointCount> direction_reversal_events_by_joint{};
    std::array<double, kJointCount> peak_joint_velocity_radps_by_joint{};
};

class JointOscillationTracker {
public:
    explicit JointOscillationTracker(double min_delta_rad_for_reversal = 0.01,
                                     uint64_t min_reversal_interval_us = 80'000);

    void reset();
    void observe(const JointTargets& targets, TimePointUs now);
    JointOscillationMetrics metrics() const;

private:
    static constexpr std::size_t kJointCount = kNumLegs * kJointsPerLeg;

    std::size_t flattenIndex(std::size_t leg_idx, std::size_t joint_idx) const;

    double min_delta_rad_for_reversal_{0.01};
    bool has_previous_{false};
    TimePointUs last_timestamp_{};
    JointTargets previous_targets_{};
    std::array<int8_t, kJointCount> previous_direction_{};
    std::array<double, kJointCount> previous_delta_rad_{};
    std::array<TimePointUs, kJointCount> last_reversal_timestamp_{};
    uint64_t min_reversal_interval_us_{80'000};
    JointOscillationMetrics metrics_{};
};
