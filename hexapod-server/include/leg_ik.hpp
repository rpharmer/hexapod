#pragma once

#include "types.hpp"

class LegIK {
public:
    JointTargets solve(const EstimatedState& est,
                       const LegTargets& targets, const SafetyState& safety);

private:
    bool solveOneLeg(const LegRawState& est, LegRawState& out, const FootTarget& foot);
    uint64_t seq_tx_{0};

    // Toy geometry
    double l1_{0.10}; // hip offset / ignored in this simplified model
    double l2_{0.18}; // upper leg
    double l3_{0.18}; // lower leg
};