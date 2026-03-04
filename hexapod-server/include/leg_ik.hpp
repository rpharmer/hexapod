#pragma once

#include "types.hpp"

class LegIK {
public:
    JointTargets solve(const EstimatedState& est,
                       const LegTargets& targets,
                       const SafetyState& safety);

private:
    bool solveOneLeg(int leg_index, const FootTarget& foot, JointState& j0, JointState& j1, JointState& j2);
    uint64_t seq_tx_{0};

    // Toy geometry
    double l1_{0.10}; // hip offset / ignored in this simplified model
    double l2_{0.18}; // upper leg
    double l3_{0.18}; // lower leg
};