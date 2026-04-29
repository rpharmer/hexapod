#pragma once

#include <array>

#include "control_config.hpp"
#include "state_fusion.hpp"
#include "types.hpp"

class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual RobotState update(const RobotState& raw) = 0;
    virtual void configure(const control_config::FusionConfig&) {}
    virtual void reset() {}
};

class SimpleEstimator final : public IEstimator {
public:
    void configure(const control_config::FusionConfig& config) override;
    void reset() override;
    RobotState update(const RobotState& raw) override;

private:
    static constexpr uint64_t kDefaultContactMemoryWindowUs = 250000;

    state_fusion::StateFusion fusion_{};
    uint64_t contact_memory_window_us_{kDefaultContactMemoryWindowUs};
    std::array<Vec3, kNumLegs> last_contact_points_body_m_{};
    std::array<TimePointUs, kNumLegs> last_contact_timestamps_{};
    std::array<LegState, kNumLegs> last_continuous_leg_states_{};
    std::array<bool, kNumLegs> have_last_continuous_leg_state_{};
    TimePointUs last_leg_timestamp_{};
    TimePointUs last_twist_timestamp_{};
    Vec3 last_twist_pos_rad_{};
    Vec3 last_body_trans_m_{};
};
