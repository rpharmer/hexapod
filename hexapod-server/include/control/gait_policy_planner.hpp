#pragma once

#include <array>

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

struct GaitSuppressionFlags {
    bool suppress_stride_progression{false};
    bool suppress_turning{false};
    bool prioritize_stability{true};
};

enum class TurnMode {
    CRAB,
    IN_PLACE
};

enum class DynamicGaitRegion {
    ARC,
    PIVOT,
    REORIENTATION
};

enum class GaitFallbackStage : uint8_t {
    NONE = 0,
    STABILITY,
    DEGRADED_LOCOMOTION,
    SAFE_STOP,
    FAULT_HOLD
};

struct DynamicSafetyEnvelope {
    double max_speed_normalized{1.0};
    double max_yaw_normalized{1.0};
    bool allow_tripod{true};
    double max_roll_pitch_rad{0.15};
};

struct LegDynamicGaitParams {
    double phase_offset{0.0};
    double duty_cycle{0.5};
    LengthM swing_height_m{LengthM{0.03}};
    LengthM step_length_m{LengthM{0.06}};
};

struct RuntimeGaitPolicy {
    GaitType gait_family{GaitType::TRIPOD};
    TurnMode turn_mode{TurnMode::CRAB};
    DynamicGaitRegion region{DynamicGaitRegion::ARC};
    DynamicSafetyEnvelope envelope{};
    GaitFallbackStage fallback_stage{GaitFallbackStage::NONE};
    std::array<LegDynamicGaitParams, kNumLegs> per_leg{};
    FrequencyHz cadence_hz{FrequencyHz{0.0}};
    GaitSuppressionFlags suppression{};
    double reach_utilization{0.0};
};

class GaitPolicyPlanner {
public:
    explicit GaitPolicyPlanner(control_config::GaitConfig config = {});

    RuntimeGaitPolicy plan(const RobotState& est,
                           const MotionIntent& intent,
                           const SafetyState& safety);

    GaitType selectGaitFamily(const MotionIntent& intent) const;
    TurnMode selectTurnMode(const MotionIntent& intent);
    std::array<LegDynamicGaitParams, kNumLegs> computePerLegDynamicParameters(GaitType family) const;
    GaitSuppressionFlags computeSuppressionFlags(const RobotState& est,
                                                 const MotionIntent& intent,
                                                 const SafetyState& safety,
                                                 TurnMode turn_mode) const;

private:
    DynamicGaitRegion classifyRegion(double speed_normalized, double yaw_normalized);
    DynamicSafetyEnvelope envelopeForRegion(DynamicGaitRegion region) const;
    GaitFallbackStage selectFallbackStage(const SafetyState& safety,
                                          const DynamicSafetyEnvelope& envelope,
                                          double roll_pitch_abs_rad,
                                          double speed_normalized,
                                          double yaw_normalized,
                                          double reach_utilization) const;
    void applyFallback(RuntimeGaitPolicy& policy) const;
    double maxReachUtilization(const RobotState& est) const;

    control_config::GaitConfig config_{};
    HexapodGeometry geometry_{defaultHexapodGeometry()};
    TurnMode last_turn_mode_{TurnMode::CRAB};
    DynamicGaitRegion last_region_{DynamicGaitRegion::ARC};
};
