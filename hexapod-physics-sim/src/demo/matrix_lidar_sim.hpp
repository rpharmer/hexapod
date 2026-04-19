#pragma once

#include "demo/terrain_patch.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "physics_sim_protocol.hpp"

namespace minphys3d::demo {

/// Simulates the built-in 64×8 matrix LiDAR on the chassis and writes the `StateResponse` tail fields.
void FillSimMatrixLidar64x8(const World& world,
                            const HexapodSceneObjects& scene,
                            const TerrainPatch& terrain_patch,
                            const Body& chassis,
                            physics_sim::StateResponse& rsp);

/// Appends sparse `TerrainSample`s from matrix LiDAR beams whose first hit matches the terrain patch
/// (down-weighted via `TerrainPatchConfig::lidar_sample_weight` / stride).
void AppendMatrixLidarTerrainSamples(const World& world,
                                     const HexapodSceneObjects& scene,
                                     const TerrainPatch& terrain_patch,
                                     const Body& chassis,
                                     const physics_sim::StateResponse& rsp,
                                     const TerrainPatchConfig& terrain_config,
                                     std::vector<TerrainSample>& out_samples);

} // namespace minphys3d::demo
