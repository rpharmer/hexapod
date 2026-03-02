#include "estimator.hpp"

EstimatedState SimpleEstimator::update(const RawHardwareState& raw)
{
  EstimatedState est{};
  for(int leg = 0; leg < kNumLegs; leg++)
  {
    LegState& legState = est.leg_states[leg];
    LegRawState legRawState = raw.leg_states[leg];
    
    for(int joint = 0; joint < kJointsPerLeg; joint++)
    {
      JointRawState jointRawState = legRawState.joint_raw_state[joint];
      JointState& jointState = legState.joint_state[joint];
      
      jointState.pos_rad = jointRawState.pos_rad;
      
      // this needs to be calculated from the change in angle divided by the time difference
      jointState.vel_radps = 0;
    }
  }
  
  est.foot_contacts = raw.foot_contacts;
  est.timestamp_us = raw.timestamp_us;
  
  return est;
}