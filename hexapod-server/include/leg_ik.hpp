#pragma once

#include "types.hpp"

class LegIK {
public:
    JointTargets solve(const EstimatedState& est,
                       const LegTargets& targets, const SafetyState& safety);

private:
    bool solveOneLeg(const LegRawState& est, LegRawState& out, const FootTarget& foot);
    uint64_t seq_tx_{0};

    HexapodGeometry hexGeo;
};