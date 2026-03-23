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
    std::array<LegState, kNumLegs> last_leg_states_{};
    TimePointUs last_leg_timestamp_{};
    TimePointUs last_twist_timestamp_{};
    Vec3 last_twist_pos_rad_{};
    Vec3 last_body_trans_m_{};
};
