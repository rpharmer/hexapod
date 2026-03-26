#pragma once

#include "foothold_planner.hpp"
#include "gait_policy_planner.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

class BodyController {
public:
    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety);

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const RuntimeGaitPolicy& policy,
                      const SafetyState& safety);

private:
    std::array<Vec3, kNumLegs> nominalStance() const;
    Vec3 clampToReachEnvelope(int leg, const Vec3& target_body) const;

    HexapodGeometry geometry_{defaultHexapodGeometry()};
    FootholdPlanner foothold_planner_{geometry_};
};
