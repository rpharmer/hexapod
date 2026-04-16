#pragma once

#include "physics_sim_protocol.hpp"
#include "types.hpp"

/** Fill `out` from a physics `StateResponse` (same frames as `PhysicsSimBridge`). */
void imuSampleFromPhysicsStateResponse(const physics_sim::StateResponse& rsp,
                                      TimePointUs timestamp_us,
                                      ImuSample& out);
