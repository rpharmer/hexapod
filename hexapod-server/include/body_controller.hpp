#pragma once

#include "types.hpp"

class BodyController {
public:
    LegTargets update(const EstimatedState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety);

private:
    std::array<Vec3, kNumLegs> nominal_stance_ {{
        {+0.20, +0.14, -0.20},
        { 0.00, +0.16, -0.20},
        {-0.20, +0.14, -0.20},
        {+0.20, -0.14, -0.20},
        { 0.00, -0.16, -0.20},
        {-0.20, -0.14, -0.20}
    }};
};