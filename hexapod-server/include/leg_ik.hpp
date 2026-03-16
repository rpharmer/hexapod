#pragma once

#include "types.hpp"

class LegIK {
public:
    LegIK();

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
