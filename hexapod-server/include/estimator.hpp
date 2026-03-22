#pragma once

#include <array>

#include "types.hpp"

class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual EstimatedState update(const RawHardwareState& raw) = 0;
};

class SimpleEstimator final : public IEstimator {
public:
    EstimatedState update(const RawHardwareState& raw) override;

private:
    static constexpr uint64_t kContactMemoryWindowUs = 250000;

    std::array<Vec3, kNumLegs> last_contact_points_body_m_{};
    std::array<TimePointUs, kNumLegs> last_contact_timestamps_{};
    TimePointUs last_twist_timestamp_{};
    Vec3 last_twist_pos_rad_{};
};
