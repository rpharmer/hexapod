#include "local_map.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    LocalMapConfig cfg{};
    cfg.width_cells = 11;
    cfg.height_cells = 11;
    cfg.resolution_m = 0.1;
    cfg.obstacle_inflation_radius_m = 0.12;
    cfg.safety_margin_m = 0.0;
    cfg.observation_timeout_s = 1.0;
    cfg.observation_decay_s = 0.5;

    LocalMapBuilder builder(cfg);
    const TimePointUs t0{1'000'000};
    const NavPose2d pose{};

    builder.update(
        pose,
        t0,
        {LocalMapObservation{t0, {LocalMapObservationSample{0.0, 0.0, LocalMapCellState::Occupied},
                                  LocalMapObservationSample{0.2, 0.0, LocalMapCellState::Free}}}});

    const LocalMapSnapshot snap0 = builder.snapshot(t0);
    int cx = 0;
    int cy = 0;
    if (!expect(snap0.raw.worldToCell(0.0, 0.0, cx, cy), "center sample should map into grid")) {
        return EXIT_FAILURE;
    }
    if (!expect(snap0.raw.stateAtCell(cx, cy) == LocalMapCellState::Occupied,
                "occupied observation should mark raw grid")) {
        return EXIT_FAILURE;
    }
    int free_x = 0;
    int free_y = 0;
    if (!expect(snap0.raw.worldToCell(0.2, 0.0, free_x, free_y), "free sample should map into grid")) {
        return EXIT_FAILURE;
    }
    if (!expect(snap0.raw.stateAtCell(free_x, free_y) == LocalMapCellState::Free,
                "free observation should mark raw grid")) {
        return EXIT_FAILURE;
    }
    if (!expect(snap0.inflated.stateAtCell(cx + 1, cy) == LocalMapCellState::Occupied,
                "inflation should expand occupied cells")) {
        return EXIT_FAILURE;
    }
    if (!expect(snap0.fresh, "recent observation should yield fresh snapshot")) {
        return EXIT_FAILURE;
    }
    if (!expect(snap0.nearest_obstacle_distance_m >= 0.0, "nearest obstacle should be measurable")) {
        return EXIT_FAILURE;
    }

    const TimePointUs stale_time{2'200'000};
    const LocalMapSnapshot stale = builder.snapshot(stale_time);
    if (!expect(!stale.fresh, "old observation should become stale")) {
        return EXIT_FAILURE;
    }
    if (!expect(stale.raw.stateAtCell(cx, cy) == LocalMapCellState::Unknown,
                "old occupied cell should decay to unknown")) {
        return EXIT_FAILURE;
    }

    builder.reset();
    builder.update(
        pose,
        t0,
        {LocalMapObservation{t0, {LocalMapObservationSample{0.2, 0.0, LocalMapCellState::Occupied}}}});
    const NavPose2d shifted_pose{0.1, 0.0, 0.0};
    builder.update(shifted_pose, TimePointUs{1'100'000}, {LocalMapObservation{TimePointUs{1'100'000}, {}}});
    const LocalMapSnapshot shifted = builder.snapshot(TimePointUs{1'100'000});
    if (!expect(shifted.raw.worldToCell(0.2, 0.0, cx, cy), "shifted map should preserve world addressing")) {
        return EXIT_FAILURE;
    }
    if (!expect(shifted.raw.stateAtCell(cx, cy) == LocalMapCellState::Occupied,
                "recentering should retain nearby occupied cells")) {
        return EXIT_FAILURE;
    }

    builder.reset();
    LocalMapObservation primary{};
    primary.timestamp_us = TimePointUs{2'000'000};
    primary.samples.push_back(LocalMapObservationSample{0.0, 0.0, LocalMapCellState::Free});
    LocalMapObservation auxiliary{};
    auxiliary.timestamp_us = TimePointUs{2'800'000};
    auxiliary.samples.push_back(LocalMapObservationSample{0.1, 0.0, LocalMapCellState::Occupied});
    auxiliary.freshness_class = LocalMapObservationFreshnessClass::Auxiliary;
    builder.update(pose, auxiliary.timestamp_us, {primary, auxiliary});
    const LocalMapSnapshot masked = builder.snapshot(TimePointUs{3'100'000});
    if (!expect(!masked.fresh, "auxiliary observations should not mask stale primary map sources")) {
        return EXIT_FAILURE;
    }
    if (!expect(masked.has_primary_observations, "primary-source bookkeeping should survive auxiliary updates")) {
        return EXIT_FAILURE;
    }

    builder.reset();
    builder.update(
        pose,
        t0,
        {LocalMapObservation{
            t0,
            {LocalMapObservationSample{0.0, 0.0, LocalMapCellState::Occupied, 0.35},
             LocalMapObservationSample{0.2, 0.0, LocalMapCellState::Free}}}});
    const LocalMapSnapshot elev_snap = builder.snapshot(t0);
    int ox = 0;
    int oy = 0;
    if (!expect(elev_snap.raw.worldToCell(0.0, 0.0, ox, oy), "elevation test cell should map")) {
        return EXIT_FAILURE;
    }
    if (!expect(elev_snap.elevation_has_data, "occupied hit with z_m should populate elevation layer")) {
        return EXIT_FAILURE;
    }
    const double z0 = elev_snap.elevation_max_hit_z.maxHitZAtCell(ox, oy);
    if (!expect(std::abs(z0 - 0.35) < 1e-6, "max hit Z should match observation")) {
        return EXIT_FAILURE;
    }

    builder.reset();
    builder.update(
        pose,
        t0,
        {LocalMapObservation{
            t0,
            {LocalMapObservationSample{0.1,
                                        0.1,
                                        LocalMapCellState::Free,
                                        std::numeric_limits<double>::quiet_NaN(),
                                        -0.03}}}});
    const LocalMapSnapshot ground_snap = builder.snapshot(t0);
    int gx = 0;
    int gy = 0;
    if (!expect(ground_snap.raw.worldToCell(0.1, 0.1, gx, gy), "ground test cell should map")) {
        return EXIT_FAILURE;
    }
    if (!expect(ground_snap.ground_elevation_has_data, "free stamps with ground_z_m should fill ground layer")) {
        return EXIT_FAILURE;
    }
    const double zg = ground_snap.elevation_ground_mean_z.maxHitZAtCell(gx, gy);
    if (!expect(std::abs(zg + 0.03) < 1e-6, "ground mean Z should match observation")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
