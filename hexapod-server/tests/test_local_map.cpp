#include "local_map.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

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

    return EXIT_SUCCESS;
}
