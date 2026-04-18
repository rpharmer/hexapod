#pragma once

#include "control_config.hpp"
#include "local_map.hpp"
#include "types.hpp"

#include <array>

/** Sample `elevation_max_hit_z` at world XY (m); NaN if unknown / out of map. */
[[nodiscard]] double sampleMaxHitZWorldM(const LocalMapSnapshot& snap, double world_x_m, double world_y_m);

/** Sample mean ground Z from `elevation_ground_mean_z` at world XY (m); NaN if unknown / out of map. */
[[nodiscard]] double sampleGroundMeanZWorldM(const LocalMapSnapshot& snap, double world_x_m, double world_y_m);

/**
 * When walking in swing, nudge foot XY in body frame toward a neighboring cell with lower reported
 * obstacle top Z (late swing only, gated by `tau01`).
 */
void applyTerrainSwingXYNudge(const LocalMapSnapshot& snap,
                              const RobotState& est,
                              const control_config::FootTerrainConfig& cfg,
                              double tau01,
                              Vec3* foot_pos_body_m);

/**
 * When walking, bias nominal stance foot Z (body frame, before swing/stance split) from mean ground
 * height under the nominal foot XY positions in world.
 */
void applyTerrainStanceZBias(const LocalMapSnapshot& snap,
                             const RobotState& est,
                             const MotionIntent& intent,
                             const control_config::FootTerrainConfig& cfg,
                             std::array<Vec3, kNumLegs>* nominal_body_m);

/**
 * When walking in swing, nudge foot Z in body frame upward if the LiDAR elevation layer reports
 * obstacle tops above the nominal foot height in world (full body roll/pitch/yaw mapping).
 */
void applyTerrainSwingClearance(const LocalMapSnapshot& snap,
                                const RobotState& est,
                                const control_config::FootTerrainConfig& cfg,
                                Vec3* foot_pos_body_m);
