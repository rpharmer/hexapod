#include "control_config.hpp"
#include "foot_terrain.hpp"
#include "local_map.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

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

    return EXIT_SUCCESS;
}
