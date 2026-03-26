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

struct LegDynamicGaitParams {
    double phase_offset{0.0};
    double duty_cycle{0.5};
    LengthM swing_height_m{LengthM{0.03}};
    LengthM step_length_m{LengthM{0.06}};
};

struct RuntimeGaitPolicy {
    GaitType gait_family{GaitType::TRIPOD};
    TurnMode turn_mode{TurnMode::CRAB};
    std::array<LegDynamicGaitParams, kNumLegs> per_leg{};
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
    double maxReachUtilization(const RobotState& est) const;

    control_config::GaitConfig config_{};
    HexapodGeometry geometry_{defaultHexapodGeometry()};
    TurnMode last_turn_mode_{TurnMode::CRAB};
};
