#include "matrix_lidar_local_map_source.hpp"
#include "motion_intent_utils.hpp"
#include "navigation_manager.hpp"
#include "nav_primitives.hpp"
#include "physics_sim_protocol.hpp"

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

void stepPose(NavPose2d& pose, const MotionIntent& intent, const double dt_s) {
    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    pose.x_m += (c * cmd.vx_mps - s * cmd.vy_mps) * dt_s;
    pose.y_m += (s * cmd.vx_mps + c * cmd.vy_mps) * dt_s;
    pose.yaw_rad = navWrapAngleRad(pose.yaw_rad + cmd.yaw_rate_radps * dt_s);
}

void fillNavRobotState(RobotState& est,
                       const NavPose2d& pose,
                       const TimePointUs now,
                       const uint64_t sample_id) {
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.x = pose.x_m;
    est.body_twist_state.body_trans_m.y = pose.y_m;
    est.body_twist_state.twist_pos_rad.z = pose.yaw_rad;
    est.sample_id = sample_id;
    est.timestamp_us = now;
}

void fillMatrixLidarBand(RobotState& est,
                         const TimePointUs lidar_time,
                         std::uint16_t range_mm,
                         int col0,
                         int col1,
                         int row0,
                         int row1) {
    est.has_matrix_lidar = true;
    est.matrix_lidar.valid = true;
    est.matrix_lidar.model = static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::Matrix64x8);
    est.matrix_lidar.cols = 64;
    est.matrix_lidar.rows = 8;
    est.matrix_lidar.timestamp_us = lidar_time;
    est.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
    for (int row = row0; row <= row1; ++row) {
        for (int col = col0; col <= col1; ++col) {
            est.matrix_lidar.ranges_mm[static_cast<std::size_t>(row * 64 + col)] = range_mm;
        }
    }
}

bool test_lidar_populates_nearest_obstacle() {
    auto lidar = std::make_shared<MatrixLidarLocalMapObservationSource>();
    NavigationManager nav(LocalMapConfig{}, LocalPlannerConfig{});
    nav.addObservationSource(lidar);
    nav.startNavigateToPose(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07),
                            NavPose2d{0.8, 0.0, 0.0});

    RobotState est{};
    const NavPose2d pose{0.0, 0.0, 0.0};
    const TimePointUs now{3'000'000};
    fillNavRobotState(est, pose, now, 1);
    // Mid-range wall straight ahead (~0.45 m) across a wide azimuth band.
    fillMatrixLidarBand(est, now, 450, 18, 46, 1, 6);

    (void)nav.mergeIntent(makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07), est, now);
    const LocalMapSnapshot snap = nav.latestMapSnapshot(now);

    if (!expect(snap.has_observations, "lidar observations should register on the map")) {
        return false;
    }
    if (!expect(snap.fresh, "map should be fresh immediately after mergeIntent")) {
        return false;
    }
    if (!expect(snap.nearest_obstacle_distance_m > 0.18 && snap.nearest_obstacle_distance_m < 0.75,
                "nearest obstacle distance should reflect a ~0.45 m forward wall")) {
        return false;
    }

    bool saw_occupied = false;
    for (int y = 0; y < snap.raw.height_cells; ++y) {
        for (int x = 0; x < snap.raw.width_cells; ++x) {
            if (snap.raw.stateAtCell(x, y) == LocalMapCellState::Occupied) {
                const NavPose2d cell = snap.raw.cellCenterPose(x, y);
                if (cell.x_m > 0.15 && cell.x_m < 0.85 && std::abs(cell.y_m) < 0.35) {
                    saw_occupied = true;
                    break;
                }
            }
        }
    }
    if (!expect(saw_occupied, "raw map should have occupied cells ahead of the robot from lidar hits")) {
        return false;
    }

    return true;
}

bool test_lidar_close_range_can_block_planner() {
    auto lidar = std::make_shared<MatrixLidarLocalMapObservationSource>();
    NavigationManager nav(LocalMapConfig{}, LocalPlannerConfig{});
    nav.addObservationSource(lidar);
    nav.startNavigateToPose(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07),
                            NavPose2d{0.5, 0.0, 0.0});

    RobotState est{};
    const NavPose2d pose{0.0, 0.0, 0.0};
    const TimePointUs now{5'000'000};
    fillNavRobotState(est, pose, now, 1);
    // Saturate the grid with minimum-range returns so inflated occupancy covers the planner start cell.
    fillMatrixLidarBand(est, now, 100, 0, 63, 0, 7);

    (void)nav.mergeIntent(makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07), est, now);
    const NavigationMonitorSnapshot mon = nav.monitor();

    if (!expect(mon.planner_status == LocalPlanStatus::Blocked, "planner should refuse a start inside inflated lidar hits")) {
        return false;
    }
    if (!expect(mon.block_reason == PlannerBlockReason::StartOccupied,
                "blocked plan should attribute start-in-occupied to lidar-inflated geometry")) {
        return false;
    }
    if (!expect(mon.lifecycle == NavigationLifecycleState::Blocked, "navigation should enter Blocked lifecycle")) {
        return false;
    }

    return true;
}

bool test_lidar_all_invalid_still_navigates_unknown_map() {
    auto lidar = std::make_shared<MatrixLidarLocalMapObservationSource>();
    NavigationManager nav(LocalMapConfig{}, LocalPlannerConfig{});
    nav.addObservationSource(lidar);
    nav.startNavigateToPose(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07),
                            NavPose2d{0.25, 0.0, 0.0});

    RobotState est{};
    NavPose2d sim{};
    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07);
    est.has_matrix_lidar = true;
    est.matrix_lidar.valid = true;
    est.matrix_lidar.model = static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::Matrix64x8);
    est.matrix_lidar.cols = 64;
    est.matrix_lidar.rows = 8;
    est.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);

    const double dt_s = 0.02;
    for (uint64_t i = 0; i < 8000; ++i) {
        const TimePointUs now{7'000'000 + i * 20'000};
        fillNavRobotState(est, sim, now, i + 1);
        est.matrix_lidar.timestamp_us = now;
        MotionIntent last = nav.mergeIntent(stand, est, now);
        if (!nav.active()) {
            break;
        }
        stepPose(sim, last, dt_s);
    }

    if (!expect(nav.monitor().lifecycle == NavigationLifecycleState::Completed,
                "with no occupied lidar cells the planner should still reach a nearby goal through unknown space")) {
        return false;
    }

    return true;
}

} // namespace

int main() {
    if (!test_lidar_populates_nearest_obstacle()) {
        return EXIT_FAILURE;
    }
    if (!test_lidar_close_range_can_block_planner()) {
        return EXIT_FAILURE;
    }
    if (!test_lidar_all_invalid_still_navigates_unknown_map()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
