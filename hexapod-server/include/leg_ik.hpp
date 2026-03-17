#pragma once

#include "types.hpp"
#include "geometry_config.hpp"

class LegIK {
public:
    explicit LegIK(HexapodGeometry geometry = defaultHexapodGeometry());

    JointTargets solve(const EstimatedState& est,
                       const LegTargets& targets, const SafetyState& safety);

private:
    bool solveOneLeg(const LegRawState& est,
                        LegRawState& out,
                        const FootTarget& foot,
                        const LegGeometry& leg);
    uint64_t seq_tx_{0};

    HexapodGeometry hexGeo;
};
