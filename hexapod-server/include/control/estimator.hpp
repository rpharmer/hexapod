#pragma once

#include <array>

#include "types.hpp"

class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual RobotState update(const RobotState& raw) = 0;
};

class SimpleEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override;

private:
    static constexpr uint64_t kContactMemoryWindowUs = 250000;

    std::array<Vec3, kNumLegs> last_contact_points_body_m_{};
    std::array<TimePointUs, kNumLegs> last_contact_timestamps_{};
    std::array<Vec3, kNumLegs> last_stance_points_body_m_{};
    std::array<bool, kNumLegs> last_stance_valid_{};
    std::array<LegState, kNumLegs> last_leg_states_{};
    TimePointUs last_leg_timestamp_{};
    TimePointUs last_body_pose_timestamp_{};
    Vec3 last_orientation_rad_{};
    Vec3 last_body_translation_m_{};
};
