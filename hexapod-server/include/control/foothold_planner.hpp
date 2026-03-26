#pragma once

#include "gait_policy_planner.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

struct PlannedFoothold {
    Vec3 pos_body_m{};
    Vec3 vel_body_mps{};
};

class FootholdPlanner {
public:
    explicit FootholdPlanner(HexapodGeometry geometry = defaultHexapodGeometry());

    PlannedFoothold plan(int leg,
                         const Vec3& nominal_body,
                         const MotionIntent& intent,
                         const GaitState& gait,
                         const RuntimeGaitPolicy& policy,
                         bool walking) const;

private:
    Vec3 modeStepDirection(int leg,
                           const MotionIntent& intent,
                           const RuntimeGaitPolicy& policy) const;
    double innerOuterScale(int leg,
                           const MotionIntent& intent,
                           const RuntimeGaitPolicy& policy) const;

    HexapodGeometry geometry_{};
};
