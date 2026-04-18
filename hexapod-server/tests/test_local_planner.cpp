#include "local_planner.hpp"

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

LocalMapSnapshot makeSnapshot(const std::vector<LocalMapObservationSample>& samples) {
    LocalMapConfig cfg{};
    cfg.width_cells = 31;
    cfg.height_cells = 31;
    cfg.resolution_m = 0.05;
    cfg.obstacle_inflation_radius_m = 0.05;
    cfg.safety_margin_m = 0.0;
    cfg.observation_timeout_s = 1.0;
    cfg.observation_decay_s = 10.0;

    LocalMapBuilder builder(cfg);
    const TimePointUs now{1'000'000};
    builder.update(NavPose2d{}, now, {LocalMapObservation{now, samples}});
    return builder.snapshot(now);
}

} // namespace

int main() {
    LocalPlannerConfig cfg{};
    cfg.search_horizon_m = 1.0;
    cfg.segment_cell_horizon = 24;
    cfg.max_output_waypoints = 8;
    AStarLocalPlanner planner(cfg);

    const LocalMapSnapshot empty = makeSnapshot({});
    const LocalPlanResult straight = planner.plan(LocalPlanRequest{NavPose2d{}, NavPose2d{0.4, 0.0, 0.0}, empty});
    if (!expect(straight.status == LocalPlanStatus::Ready, "planner should find straight path on empty map")) {
        return EXIT_FAILURE;
    }
    if (!expect(!straight.waypoints.empty(), "straight path should emit waypoints")) {
        return EXIT_FAILURE;
    }
    if (!expect(std::abs(straight.waypoints.back().x_m - 0.4) < 0.08,
                "straight path should end near goal")) {
        return EXIT_FAILURE;
    }

    std::vector<LocalMapObservationSample> wall{};
    for (double y = -0.15; y <= 0.15; y += 0.05) {
        wall.push_back(LocalMapObservationSample{0.2, y, LocalMapCellState::Occupied});
        wall.push_back(LocalMapObservationSample{0.25, y, LocalMapCellState::Occupied});
    }
    const LocalMapSnapshot detour_snapshot = makeSnapshot(wall);
    const LocalPlanResult detour =
        planner.plan(LocalPlanRequest{NavPose2d{}, NavPose2d{0.4, 0.0, 0.0}, detour_snapshot});
    if (!expect(detour.status == LocalPlanStatus::Ready, "planner should find a detour around obstacle wall")) {
        return EXIT_FAILURE;
    }
    bool left_centerline = false;
    for (const NavPose2d& wp : detour.waypoints) {
        if (std::abs(wp.y_m) > 0.04) {
            left_centerline = true;
            break;
        }
    }
    if (!expect(left_centerline, "detour path should leave centerline")) {
        return EXIT_FAILURE;
    }

    std::vector<LocalMapObservationSample> blocked{};
    for (double y = -0.75; y <= 0.75; y += 0.05) {
        blocked.push_back(LocalMapObservationSample{0.1, y, LocalMapCellState::Occupied});
        blocked.push_back(LocalMapObservationSample{0.15, y, LocalMapCellState::Occupied});
    }
    const LocalMapSnapshot blocked_snapshot = makeSnapshot(blocked);
    const LocalPlanResult blocked_result =
        planner.plan(LocalPlanRequest{NavPose2d{}, NavPose2d{0.4, 0.0, 0.0}, blocked_snapshot});
    if (!expect(blocked_result.status == LocalPlanStatus::Blocked,
                "planner should report blocked when wall fully seals path")) {
        return EXIT_FAILURE;
    }
    if (!expect(blocked_result.block_reason == PlannerBlockReason::GoalOccupied ||
                    blocked_result.block_reason == PlannerBlockReason::NoPath,
                "blocked plan should surface a blocking reason")) {
        return EXIT_FAILURE;
    }

    std::vector<LocalMapObservationSample> narrow_free{};
    for (double x = 0.0; x <= 0.2; x += 0.05) {
        narrow_free.push_back(LocalMapObservationSample{x, 0.0, LocalMapCellState::Free});
    }
    narrow_free.push_back(LocalMapObservationSample{0.05, 0.20, LocalMapCellState::Occupied});
    const LocalMapSnapshot narrow_free_snapshot = makeSnapshot(narrow_free);
    const LocalPlanResult narrow_free_result =
        planner.plan(LocalPlanRequest{NavPose2d{}, NavPose2d{0.4, 0.0, 0.0}, narrow_free_snapshot});
    if (!expect(narrow_free_result.status == LocalPlanStatus::Blocked,
                "planner should not treat unknown cells as free once obstacles have been observed")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
