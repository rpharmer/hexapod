#pragma once

#include "geometry_config.hpp"
#include "types.hpp"

class BodyController {
public:
    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety);

private:
    std::array<Vec3, kNumLegs> nominalStance() const;

    HexapodGeometry geometry_{defaultHexapodGeometry()};
};
