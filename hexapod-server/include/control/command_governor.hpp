#pragma once

#include "control_config.hpp"
#include "gait_params.hpp"
#include "types.hpp"

#include <cstdint>

enum class CommandGovernorReason : std::uint32_t {
    None = 0,
    LowSpeedRegime = 1u << 0,
    LowSupportMargin = 1u << 1,
    HighTilt = 1u << 2,
    HighBodyRate = 1u << 3,
    LowFusionTrust = 1u << 4,
    HighContactMismatch = 1u << 5,
    HighCommandAccel = 1u << 6,
};

inline constexpr CommandGovernorReason operator|(CommandGovernorReason lhs, CommandGovernorReason rhs) {
    return static_cast<CommandGovernorReason>(static_cast<std::uint32_t>(lhs) |
                                              static_cast<std::uint32_t>(rhs));
}

inline constexpr CommandGovernorReason operator&(CommandGovernorReason lhs, CommandGovernorReason rhs) {
    return static_cast<CommandGovernorReason>(static_cast<std::uint32_t>(lhs) &
                                              static_cast<std::uint32_t>(rhs));
}

inline constexpr CommandGovernorReason& operator|=(CommandGovernorReason& lhs, CommandGovernorReason rhs) {
    lhs = lhs | rhs;
    return lhs;
}

inline constexpr bool hasCommandGovernorReason(CommandGovernorReason mask, CommandGovernorReason flag) {
    return static_cast<std::uint32_t>(mask & flag) != 0u;
}

struct CommandGovernorState {
    double requested_planar_speed_mps{0.0};
    double governed_planar_speed_mps{0.0};
    double requested_yaw_rate_radps{0.0};
    double governed_yaw_rate_radps{0.0};
    double requested_body_height_m{0.0};
    double governed_body_height_m{0.0};
    double support_margin_m{0.0};
    double body_tilt_rad{0.0};
    double body_rate_radps{0.0};
    double fusion_trust{1.0};
    double contact_mismatch_ratio{0.0};
    double command_accel_mps2{0.0};
    double severity{0.0};
    double command_scale{1.0};
    double cadence_scale{1.0};
    double swing_height_floor_m{0.0};
    double body_height_delta_m{0.0};
    bool saturated{false};
    CommandGovernorReason reasons{CommandGovernorReason::None};
};

class CommandGovernor {
public:
    explicit CommandGovernor(control_config::CommandGovernorConfig config = {});

    void reset();

    CommandGovernorState apply(const RobotState& est,
                               MotionIntent& intent,
                               const SafetyState& safety,
                               const GaitState& previous_gait);

private:
    control_config::CommandGovernorConfig config_{};
    TimePointUs last_clock_{};
    double last_requested_planar_speed_mps_{0.0};
    double last_requested_yaw_rate_radps_{0.0};
};

