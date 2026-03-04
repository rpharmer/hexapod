#include "leg_ik.hpp"

JointTargets LegIK::solve(const EstimatedState& est,
                       const LegTargets& targets, const SafetyState& safety){
  JointTargets joints{};
  
  // for each leg; solve IK
  for(int legID = 0; legID < kNumLegs; legID++){
    // if right hand leg
    if (legID%2 == 0) {
      solveOneLeg(est.leg_states[legID], joints.leg_raw_states[legID], targets.feet[legID]);
    }
    else { //if left hand leg
      solveOneLeg(est.leg_states[legID], joints.leg_raw_states[legID], targets.feet[legID]);
    }
  }
  (void)safety;
  return joints;
}

bool LegIK::solveOneLeg(const LegRawState& est, LegRawState& out, const FootTarget& foot){
  out = est;
  (void)foot;
  return true;
}