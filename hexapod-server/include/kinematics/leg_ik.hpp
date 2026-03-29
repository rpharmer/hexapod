#pragma once

#include "geometry_config.hpp"
#include "types.hpp"

#include <array>

class LegIK {
public:
    explicit LegIK(HexapodGeometry geometry = defaultHexapodGeometry());

    JointTargets solve(const RobotState& est,
                       const LegTargets& targets, const SafetyState& safety);

private:
    bool solveOneLeg(const LegState& unwrap_reference,
                     LegState& out,
                     const FootTarget& foot,
                     const LegGeometry& leg);
    HexapodGeometry hexGeo;
    JointTargets last_unwrapped_command_{};
    std::array<bool, kNumLegs> unwrap_initialized_{};
};
