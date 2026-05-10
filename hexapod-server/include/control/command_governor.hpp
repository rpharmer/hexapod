#pragma once

#include "control_config.hpp"
#include "gait_params.hpp"
#include "types.hpp"

#include <cstdint>

struct SupportAssessment;

enum class CommandGovernorReason : std::uint32_t {
    None = 0,
    LowSpeedRegime = 1u << 0,
    LowSupportMargin = 1u << 1,
    HighTilt = 1u << 2,
    HighBodyRate = 1u << 3,
    LowFusionTrust = 1u << 4,
    HighContactMismatch = 1u << 5,
    HighCommandAccel = 1u << 6,
    RecoveryHold = 1u << 7,
};

enum class RecoveryStage : std::uint8_t {
    None = 0,
    ActiveHold = 1,
    Settling = 2,
    RampOut = 3,
};

const char* recoveryStageName(RecoveryStage stage);

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
    double current_support_margin_m{0.0};
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
    int current_support_count{0};
    int confirmed_support_count{0};
    int uncertain_support_count{0};
    int search_leg_count{0};
    int lost_candidate_count{0};
    int expected_touchdown_count{0};
    bool saturated{false};
    bool recovery_hold_active{false};
    bool freeze_phase{false};
    bool all_support_confirmed{false};
    bool recovery_release_ready{false};
    double recovery_collapse_headroom_m{0.0};
    RecoveryStage recovery_stage{RecoveryStage::None};
    CommandGovernorReason reasons{CommandGovernorReason::None};
};

class CommandGovernor {
public:
    explicit CommandGovernor(control_config::CommandGovernorConfig config = {},
                             control_config::SafetyConfig safety_config = {});

    void reset();
    void latchRecoveryHold(TimePointUs now = {});

    CommandGovernorState preview(const RobotState& est,
                                 MotionIntent& intent,
                                 const SafetyState& safety,
                                 const GaitState& previous_gait,
                                 const SupportAssessment* current_support = nullptr) const;

    CommandGovernorState apply(const RobotState& est,
                               MotionIntent& intent,
                               const SafetyState& safety,
                               const GaitState& previous_gait,
                               const SupportAssessment* current_support = nullptr);

    CommandGovernorState finalizeRecovery(const RobotState& est,
                                          const MotionIntent& intent,
                                          const SafetyState& safety,
                                          const GaitState& gait,
                                          const SupportAssessment& current_support,
                                          CommandGovernorState state,
                                          TimePointUs now = {});

private:
    void enterRecoveryHold(TimePointUs now);
    void setRecoveryStage(RecoveryStage stage, TimePointUs now);
    void applyRecoveryStageState(TimePointUs now, CommandGovernorState& out) const;

    control_config::CommandGovernorConfig config_{};
    control_config::SafetyConfig safety_config_{};
    TimePointUs last_clock_{};
    double last_requested_planar_speed_mps_{0.0};
    double last_requested_yaw_rate_radps_{0.0};
    RecoveryStage recovery_stage_{RecoveryStage::None};
    TimePointUs recovery_stage_started_{};
    TimePointUs recovery_healthy_since_{};
    // Asymmetric slew on body-height squat: protective squat engages instantly, release
    // (return toward 0) is slew-rate-limited so gait transitions release the sag
    // continuously rather than requiring the recovery FSM to mediate.
    double body_height_delta_state_{0.0};
    TimePointUs body_height_delta_clock_{};
    // Filtered RampOut/Settling health margin: min over normalised tilt, body-rate, support
    // and headroom margins, EWMA-filtered. Hysteretic two-threshold gate plus min dwell and
    // sustained-breach window prevent the FSM from cycling on stride-period noise.
    double ramp_out_health_filtered_{1.0};
    TimePointUs ramp_out_health_clock_{};
    TimePointUs ramp_out_breach_since_{};
};
