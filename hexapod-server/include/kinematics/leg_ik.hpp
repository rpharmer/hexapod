#pragma once

#include "types.hpp"
#include "geometry_config.hpp"

class LegIK {
public:
    explicit LegIK(HexapodGeometry geometry = defaultHexapodGeometry());

    JointTargets solve(const RobotState& est,
                       const LegTargets& targets, const SafetyState& safety);

    [[nodiscard]] std::array<bool, kNumLegs> lastReachClampHit() const {
        return last_reach_clamp_hit_;
    }

private:
    bool solveOneLeg(LegState& out,
                     const FootTarget& foot,
                     const LegGeometry& leg,
                     bool& reach_clamped);
    HexapodGeometry hexGeo;
    JointTargets last_commanded_targets_{};
    std::array<bool, kNumLegs> have_last_commanded_target_{};
    std::array<bool, kNumLegs> last_reach_clamp_hit_{};
};
