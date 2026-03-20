#include "estimator.hpp"

EstimatedState SimpleEstimator::update(const RawHardwareState& raw)
{
  EstimatedState est{};
 
  est.leg_states = raw.leg_states;
  est.foot_contacts = raw.foot_contacts;
  est.sample_id = raw.sample_id;
  est.timestamp_us = raw.timestamp_us;
  
  return est;
}
