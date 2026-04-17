#pragma once

#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "physics_sim_protocol.hpp"

namespace minphys3d::demo {

/// Simulates the built-in 64×8 matrix LiDAR on the chassis and writes the `StateResponse` tail fields.
void FillSimMatrixLidar64x8(const World& world,
                            const HexapodSceneObjects& scene,
                            const Body& chassis,
                            physics_sim::StateResponse& rsp);

} // namespace minphys3d::demo
