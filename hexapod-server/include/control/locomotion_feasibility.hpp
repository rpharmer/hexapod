#pragma once

#include "command_governor.hpp"
#include "control_config.hpp"
#include "support_assessment.hpp"
#include "types.hpp"

#include <array>
#include <cstdint>

enum class LegContactMode : std::uint8_t {
    PlannedStance = 0,
    PlannedSwing = 1,
    HeldStance = 2,
    ContactGrace = 3,
    LostCandidate = 4,
    RecoveryTouchdown = 5,
    Disabled = 6,
};

const char* legContactModeName(LegContactMode mode);
const char* controlMarginSourceName(control_config::ControlMarginSource source);

struct LegContactDecision {
    LegContactMode mode{LegContactMode::PlannedSwing};
    ContactPhase fused_phase{ContactPhase::Search};
    double fused_confidence{0.0};
    double phase{0.0};
    double swing_tau{0.0};
    bool planned_stance{false};
    bool raw_contact{false};
    bool stability_hold{false};
    bool effective_stance{false};
    bool use_stance_kinematics{false};
};

struct HeightPolicySnapshot {
    bool valid{false};
    bool enabled{false};
    double commanded_body_height_m{0.0};
    double measured_body_height_m{0.0};
    double governor_delta_m{0.0};
    double compliance_sag_m{0.0};
    double tilt_squat_request_m{0.0};
    double swing_clearance_request_m{0.0};
    double terrain_stance_request_m{0.0};
    double policy_body_height_m{0.0};
};

struct LocomotionFeasibility {
    bool valid{false};
    bool enabled{false};
    control_config::ControlMarginSource control_margin_source{control_config::ControlMarginSource::Nominal};
    SupportAssessment support{};
    std::array<LegContactDecision, kNumLegs> contact{};
    std::array<double, kNumLegs> lift_clearance_m{};
    std::array<bool, kNumLegs> safe_to_lift{};
    double nominal_margin_m{0.0};
    double actual_margin_m{0.0};
    double control_margin_m{0.0};
    double body_tilt_rad{0.0};
    double body_rate_radps{0.0};
    bool high_demand{false};
    bool dynamic_risk{false};
    bool sparse_support{false};
    bool deadlocked{false};
    bool recovery_recommended{false};
    HeightPolicySnapshot height{};
};

std::array<LegContactDecision, kNumLegs> computeLegContactDecisions(const RobotState& est,
                                                                    const GaitState& gait,
                                                                    const SafetyState& safety);

LocomotionFeasibility computeLocomotionFeasibility(const RobotState& est,
                                                   const MotionIntent& intent,
                                                   const GaitState& gait,
                                                   const GaitState& previous_gait,
                                                   const SafetyState& safety,
                                                   const HexapodGeometry& geometry,
                                                   const control_config::LocomotionRedesignConfig& config);

HeightPolicySnapshot evaluateHeightPolicySnapshot(const RobotState& est,
                                                  const MotionIntent& intent,
                                                  const GaitState& gait,
                                                  const CommandGovernorState& governor,
                                                  const control_config::LocomotionRedesignConfig& config);
