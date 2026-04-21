#pragma once

#include "physics_sim_protocol.hpp"
#include "types.hpp"

/// Copies matrix LiDAR tail from a physics `StateResponse` into `RobotState` (same path as IMU).
/// The wire capture timestamp is used when present so repeated frames can be rate-limited.
void matrixLidarFromPhysicsStateResponse(const physics_sim::StateResponse& rsp,
                                         TimePointUs timestamp_us,
                                         RobotState& out);
