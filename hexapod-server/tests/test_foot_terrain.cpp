#include "control_config.hpp"
#include "foot_terrain.hpp"
#include "local_map.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool expect(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    LocalMapConfig cfg{};
    cfg.width_cells = 21;
    cfg.height_cells = 21;
    cfg.resolution_m = 0.1;
    cfg.observation_timeout_s = 1.0;
    cfg.observation_decay_s = 5.0;

    LocalMapBuilder builder(cfg);
    const TimePointUs t0{1'000'000};
    const NavPose2d pose{};
    builder.update(
        pose,
        t0,
        {LocalMapObservation{
            t0,
            {LocalMapObservationSample{0.2, 0.0, LocalMapCellState::Occupied, 0.28},
             LocalMapObservationSample{0.2, 0.0, LocalMapCellState::Free}}}});

    LocalMapSnapshot snap = builder.snapshot(t0);
    if (!expect(snap.fresh && snap.elevation_has_data, "snapshot should carry fresh elevation")) {
        return EXIT_FAILURE;
    }

    RobotState est{};
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.x = 0.0;
    est.body_twist_state.body_trans_m.y = 0.0;
    est.body_twist_state.body_trans_m.z = 0.05;
    est.body_twist_state.twist_pos_rad.z = 0.0;

    Vec3 foot{0.2, 0.0, -0.05};
    const double z_before = foot.z;
    applyTerrainSwingClearance(snap, est, control_config::FootTerrainConfig{}, &foot);

    if (!expect(foot.z > z_before + 1e-4, "swing foot should gain Z clearance from elevation map")) {
        return EXIT_FAILURE;
    }

    const double zh = sampleMaxHitZWorldM(snap, 0.2, 0.0);
    if (!expect(std::abs(zh - 0.28) < 1e-6, "world sample should read back obstacle top Z")) {
        return EXIT_FAILURE;
    }

    LocalMapSnapshot swing_snap{};
    swing_snap.raw.width_cells = 3;
    swing_snap.raw.height_cells = 3;
    swing_snap.raw.resolution_m = 0.1;
    swing_snap.raw.center_pose = NavPose2d{};
    swing_snap.raw.cells.assign(9, LocalMapCellState::Unknown);
    swing_snap.elevation_max_hit_z.width_cells = 3;
    swing_snap.elevation_max_hit_z.height_cells = 3;
    swing_snap.elevation_max_hit_z.resolution_m = 0.1;
    swing_snap.elevation_max_hit_z.center_pose = NavPose2d{};
    swing_snap.elevation_max_hit_z.max_hit_z_m.assign(9, std::numeric_limits<double>::quiet_NaN());
    swing_snap.elevation_ground_mean_z.width_cells = 3;
    swing_snap.elevation_ground_mean_z.height_cells = 3;
    swing_snap.elevation_ground_mean_z.resolution_m = 0.1;
    swing_snap.elevation_ground_mean_z.center_pose = NavPose2d{};
    swing_snap.elevation_ground_mean_z.max_hit_z_m.assign(9, std::numeric_limits<double>::quiet_NaN());
    swing_snap.elevation_has_data = true;
    swing_snap.ground_elevation_has_data = true;
    swing_snap.fresh = true;
    swing_snap.has_observations = true;
    swing_snap.has_primary_observations = true;
    swing_snap.raw.cells[4] = LocalMapCellState::Occupied;
    swing_snap.elevation_max_hit_z.max_hit_z_m[4] = 0.28;
    swing_snap.elevation_ground_mean_z.max_hit_z_m[4] = 0.10;

    control_config::FootTerrainConfig swing_cfg{};
    swing_cfg.swing_margin_m = 0.0;
    swing_cfg.swing_max_lift_m = 1.0;
    swing_cfg.swing_blend = 1.0;

    Vec3 swing_foot{0.0, 0.0, -0.05};
    applyTerrainSwingClearance(swing_snap, est, swing_cfg, &swing_foot);
    if (!expect(std::abs(swing_foot.z - 0.05) < 1e-6,
                "swing clearance should prefer ground mean over obstacle tops")) {
        return EXIT_FAILURE;
    }

    control_config::FootTerrainConfig disabled_cfg{};
    disabled_cfg.enable_stance_plane_bias = false;
    disabled_cfg.enable_swing_clearance = false;
    disabled_cfg.enable_swing_xy_nudge = false;

    std::array<Vec3, kNumLegs> nominal{};
    nominal.fill(Vec3{0.10, 0.0, -0.05});
    const std::array<Vec3, kNumLegs> nominal_before = nominal;
    applyTerrainStanceZBias(snap, est, MotionIntent{}, disabled_cfg, 1.0, &nominal);
    bool stance_unchanged = true;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        stance_unchanged = stance_unchanged && nominal[static_cast<std::size_t>(leg)].x == nominal_before[static_cast<std::size_t>(leg)].x &&
                           nominal[static_cast<std::size_t>(leg)].y == nominal_before[static_cast<std::size_t>(leg)].y &&
                           nominal[static_cast<std::size_t>(leg)].z == nominal_before[static_cast<std::size_t>(leg)].z;
    }
    if (!expect(stance_unchanged, "disabled stance terrain bias should not modify nominal stance")) {
        return EXIT_FAILURE;
    }

    Vec3 swing_xy{0.0, 0.0, -0.05};
    applyTerrainSwingXYNudge(swing_snap, est, disabled_cfg, 0.9, &swing_xy);
    if (!expect(swing_xy.x == 0.0 && swing_xy.y == 0.0,
                "disabled swing XY terrain nudge should not modify foot position")) {
        return EXIT_FAILURE;
    }

    Vec3 swing_z{0.0, 0.0, -0.05};
    applyTerrainSwingClearance(swing_snap, est, disabled_cfg, &swing_z);
    if (!expect(swing_z.z == -0.05, "disabled swing clearance should not modify foot position")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
