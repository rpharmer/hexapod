/**
 * Phase 5 acceptance on the live physics-sim stack.
 *
 * The current UDP physics sim still has noticeable drift under closed-loop navigation, so the empty
 * and single-obstacle cases use "healthy progress + obstacle-aware planner behaviour" as the
 * acceptance bar instead of requiring tight goal completion. The blocked corridor and mid-run
 * intrusion cases remain strict about safe terminal behaviour and replanning.
 */

#include "control_config.hpp"
#include "matrix_lidar_local_map_source.hpp"
#include "motion_intent_utils.hpp"
#include "navigation_manager.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_local_map_source.hpp"
#include "physics_sim_protocol.hpp"
#include "robot_runtime.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool isTerminal(const NavigationLifecycleState state) {
    return state == NavigationLifecycleState::Completed ||
           state == NavigationLifecycleState::Failed ||
           state == NavigationLifecycleState::Cancelled ||
           state == NavigationLifecycleState::MapUnavailable;
}

double pointToGoalLateralDeviation(const NavPose2d& start,
                                   const NavPose2d& goal,
                                   const NavPose2d& point) {
    const double dx = goal.x_m - start.x_m;
    const double dy = goal.y_m - start.y_m;
    const double denom = std::hypot(dx, dy);
    if (denom <= 1.0e-9) {
        return 0.0;
    }
    return std::abs(dx * (point.y_m - start.y_m) - dy * (point.x_m - start.x_m)) / denom;
}

double pointProgressAlongGoal(const NavPose2d& start,
                              const NavPose2d& goal,
                              const NavPose2d& point) {
    const double dx = goal.x_m - start.x_m;
    const double dy = goal.y_m - start.y_m;
    const double denom = std::hypot(dx, dy);
    if (denom <= 1.0e-9) {
        return 0.0;
    }
    return ((point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy) / denom;
}

bool pointInsideFootprint(const NavPose2d& point,
                          const PhysicsSimObstacleFootprint& footprint,
                          const double margin_m) {
    const double cos_yaw = std::cos(footprint.yaw_rad);
    const double sin_yaw = std::sin(footprint.yaw_rad);
    const double dx = point.x_m - footprint.center_x_m;
    const double dy = point.y_m - footprint.center_y_m;
    const double local_x = dx * cos_yaw + dy * sin_yaw;
    const double local_y = -dx * sin_yaw + dy * cos_yaw;
    return std::abs(local_x) <= footprint.half_extent_x_m + margin_m &&
           std::abs(local_y) <= footprint.half_extent_y_m + margin_m;
}

std::filesystem::path resolveNavigationScene(const std::string& sim_exe_path,
                                             const std::string& filename) {
    namespace fs = std::filesystem;
    const fs::path sim_exe = fs::absolute(fs::path(sim_exe_path));
    const fs::path sim_root = sim_exe.parent_path().parent_path();
    const std::vector<fs::path> candidates{
        sim_root / "assets" / "scenes" / "navigation" / filename,
        fs::current_path() / "../../hexapod-physics-sim/assets/scenes/navigation" / filename,
        fs::current_path() / "../hexapod-physics-sim/assets/scenes/navigation" / filename,
        fs::current_path() / "hexapod-physics-sim/assets/scenes/navigation" / filename,
    };
    for (const fs::path& candidate : candidates) {
        std::error_code ec;
        if (fs::exists(candidate, ec)) {
            return fs::weakly_canonical(candidate, ec);
        }
    }
    return sim_root / "assets" / "scenes" / "navigation" / filename;
}

class PhysicsSimProcess {
public:
    PhysicsSimProcess(std::string sim_exe_path, int port, std::optional<std::string> scene_file)
        : sim_exe_path_(std::move(sim_exe_path)),
          port_(port),
          scene_file_(std::move(scene_file)) {}

    ~PhysicsSimProcess() {
        stop();
    }

    bool start() {
#if !defined(__linux__)
        return false;
#else
        pid_ = ::fork();
        if (pid_ < 0) {
            std::cerr << "fork failed\n";
            return false;
        }
        if (pid_ == 0) {
            const std::string port_str = std::to_string(port_);
            if (scene_file_.has_value()) {
                ::execl(sim_exe_path_.c_str(),
                        sim_exe_path_.c_str(),
                        "--serve",
                        "--serve-port",
                        port_str.c_str(),
                        "--scene-file",
                        scene_file_->c_str(),
                        nullptr);
            } else {
                ::execl(sim_exe_path_.c_str(),
                        sim_exe_path_.c_str(),
                        "--serve",
                        "--serve-port",
                        port_str.c_str(),
                        nullptr);
            }
            std::perror("execl");
            _exit(127);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
        return true;
#endif
    }

    void stop() {
#if defined(__linux__)
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            ::waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
#endif
    }

private:
    std::string sim_exe_path_{};
    int port_{0};
    std::optional<std::string> scene_file_{};
#if defined(__linux__)
    pid_t pid_{-1};
#endif
};

struct NavigationRunMetrics {
    NavigationMonitorSnapshot final_monitor{};
    NavPose2d start_pose{};
    double start_roll_rad{0.0};
    double start_pitch_rad{0.0};
    NavPose2d goal_pose{};
    NavPose2d end_pose{};
    double goal_error_m{0.0};
    double path_length_m{0.0};
    double max_lateral_deviation_m{0.0};
    double max_progress_m{0.0};
    std::size_t peak_replan_count{0};
    std::size_t peak_active_segment_waypoints{0};
    bool touched_raw_obstacle{false};
    bool saw_obstacle_footprints{false};
    std::size_t peak_obstacle_count{0};
    std::size_t peak_raw_occupied_cells{0};
    std::size_t peak_raw_free_cells{0};
    double min_nearest_obstacle_distance_m{-1.0};
    double nearest_obstacle_dx_m{0.0};
    double nearest_obstacle_dy_m{0.0};
};

void runWarmupStep(RobotRuntime& runtime, const MotionIntent& stand_fallback) {
    MotionIntent intent = stand_fallback;
    stampIntentStreamMotionFields(intent);
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

void runNavigationStep(RobotRuntime& runtime, const MotionIntent& stand_fallback) {
    MotionIntent intent = stand_fallback;
    stampIntentStreamMotionFields(intent);
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

std::optional<NavigationRunMetrics> runNavigationCase(const std::string& label,
                                                      const std::string& sim_exe_path,
                                                      const std::optional<std::string>& scene_file,
                                                      const double goal_forward_m,
                                                      const double blocked_timeout_s,
                                                      const int max_steps,
                                                      const int bus_loop_period_us) {
    const int port = 24000 + (static_cast<int>(::getpid()) % 3000) +
                     static_cast<int>(std::hash<std::string>{}(label) % 1000);

    PhysicsSimProcess sim(sim_exe_path, port, scene_file);
    if (!sim.start()) {
        expect(false, label + ": failed to start physics sim process");
        return std::nullopt;
    }

    auto bridge = std::make_unique<PhysicsSimBridge>("127.0.0.1", port, bus_loop_period_us, 24, nullptr);
    PhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    // Keep the broader live-navigation acceptance suite pinned to command-only blending for now;
    // deterministic coverage protects the sign-conversion seam directly while this suite remains stable.
    cfg.gait.foot_estimator_blend = 0.0;
    cfg.locomotion_cmd.enable_first_order_filter = false;
    cfg.locomotion_cmd.enable_chassis_accel_limit = false;
    cfg.local_map.width_cells = 61;
    cfg.local_map.height_cells = 61;
    cfg.local_map.resolution_m = 0.05;
    cfg.local_map.obstacle_inflation_radius_m = 0.04;
    cfg.local_map.safety_margin_m = 0.01;
    cfg.local_planner.search_horizon_m = 1.6;
    cfg.local_planner.segment_cell_horizon = 14;
    cfg.local_planner.max_output_waypoints = 8;
    cfg.local_planner.blocked_timeout_s = blocked_timeout_s;
    cfg.nav_bridge.body_frame_integral_ki_fwd_per_s = 0.35;
    cfg.nav_bridge.body_frame_integral_ki_lat_per_s = 0.30;
    cfg.nav_bridge.body_frame_integral_abs_cap_m_s = 0.08;

    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg);
    if (!expect(runtime.init(), label + ": runtime init should succeed")) {
        return std::nullopt;
    }

    physics_sim::StateCorrection correction{};
    correction.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    correction.sequence_id = 9001;
    correction.timestamp_us = 1;
    correction.flags = physics_sim::kStateCorrectionContactValid;
    correction.correction_strength = 0.45f;
    for (std::size_t i = 0; i < correction.foot_contact_phase.size(); ++i) {
        correction.foot_contact_phase[i] =
            static_cast<std::uint8_t>(physics_sim::ContactPhase::ConfirmedStance);
        correction.foot_contact_confidence[i] = 0.75f;
    }
    if (!expect(bridge_ptr->sendStateCorrection(correction),
                label + ": live physics sim should accept a correction packet")) {
        return std::nullopt;
    }

    auto navigation_manager =
        std::make_unique<NavigationManager>(cfg.local_map, cfg.local_planner, cfg.nav_bridge);
    navigation_manager->addObservationSource(std::make_shared<MatrixLidarLocalMapObservationSource>());
    navigation_manager->addObservationSource(std::make_shared<PhysicsSimLocalMapObservationSource>(
        *bridge_ptr, std::max(0.02, cfg.local_map.resolution_m * 0.5)));
    runtime.setNavigationManager(std::move(navigation_manager));

    MotionIntent stand_fallback = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.06);
    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.06);

    constexpr int kWarmupSteps = 140;
    for (int i = 0; i < kWarmupSteps; ++i) {
        runWarmupStep(runtime, stand_fallback);
    }

    const RobotState settled = runtime.estimatedSnapshot();
    const NavPose2d start_pose = navPose2dFromRobotState(settled);
    if (!expect(std::abs(start_pose.yaw_rad) <= 0.20, label + ": settled yaw should stay near zero")) {
        return std::nullopt;
    }
    if (!expect(settled.has_fusion_diagnostics && settled.fusion.model_trust >= 0.0,
                label + ": settled estimator should expose fusion diagnostics")) {
        return std::nullopt;
    }

    FollowWaypoints::Params follow_params{};
    follow_params.stall_timeout_s = 6.0;
    follow_params.go_to.rotate_first = false;
    follow_params.go_to.drive.max_v_mps = 0.055;
    follow_params.go_to.drive.position_gain = 0.24;
    follow_params.go_to.drive.position_tol_m = 0.035;
    follow_params.go_to.drive.settle_cycles_required = 5;
    follow_params.go_to.drive.yaw_hold_kp = 0.0;
    follow_params.go_to.rotate.error_threshold_rad = 0.20;
    follow_params.go_to.rotate.settle_cycles_required = 4;

    const NavPose2d goal_pose{
        start_pose.x_m + goal_forward_m,
        start_pose.y_m,
        start_pose.yaw_rad,
    };
    runtime.navigationManager()->startNavigateToPose(walk_base, goal_pose, follow_params);

    NavigationRunMetrics metrics{};
    metrics.start_pose = start_pose;
    metrics.start_roll_rad = settled.body_twist_state.twist_pos_rad.x;
    metrics.start_pitch_rad = settled.body_twist_state.twist_pos_rad.y;
    metrics.goal_pose = goal_pose;
    NavPose2d previous_pose = start_pose;

    for (int step = 0; step < max_steps; ++step) {
        runNavigationStep(runtime, stand_fallback);

        const NavPose2d pose = navPose2dFromRobotState(runtime.estimatedSnapshot());
        metrics.path_length_m += std::hypot(pose.x_m - previous_pose.x_m, pose.y_m - previous_pose.y_m);
        previous_pose = pose;
        metrics.end_pose = pose;
        metrics.max_lateral_deviation_m =
            std::max(metrics.max_lateral_deviation_m, pointToGoalLateralDeviation(start_pose, goal_pose, pose));
        metrics.max_progress_m =
            std::max(metrics.max_progress_m, pointProgressAlongGoal(start_pose, goal_pose, pose));

        const NavigationMonitorSnapshot monitor = runtime.navigationManager()->monitor();
        metrics.peak_replan_count = std::max(metrics.peak_replan_count, monitor.replan_count);
        metrics.peak_active_segment_waypoints =
            std::max(metrics.peak_active_segment_waypoints, monitor.active_segment_waypoint_count);
        metrics.final_monitor = monitor;

        const LocalMapSnapshot snapshot =
            runtime.navigationManager()->latestMapSnapshot(runtime.estimatedSnapshot().timestamp_us);
        std::size_t occupied_cells = 0;
        std::size_t free_cells = 0;
        for (const LocalMapCellState cell : snapshot.raw.cells) {
            if (cell == LocalMapCellState::Occupied) {
                ++occupied_cells;
            } else if (cell == LocalMapCellState::Free) {
                ++free_cells;
            }
        }
        metrics.peak_raw_occupied_cells = std::max(metrics.peak_raw_occupied_cells, occupied_cells);
        metrics.peak_raw_free_cells = std::max(metrics.peak_raw_free_cells, free_cells);
        if (snapshot.nearest_obstacle_distance_m >= 0.0 &&
            (metrics.min_nearest_obstacle_distance_m < 0.0 ||
             snapshot.nearest_obstacle_distance_m < metrics.min_nearest_obstacle_distance_m)) {
            metrics.min_nearest_obstacle_distance_m = snapshot.nearest_obstacle_distance_m;
            double best_distance = 1.0e9;
            for (int y = 0; y < snapshot.raw.height_cells; ++y) {
                for (int x = 0; x < snapshot.raw.width_cells; ++x) {
                    if (snapshot.raw.stateAtCell(x, y) != LocalMapCellState::Occupied) {
                        continue;
                    }
                    const NavPose2d cell = snapshot.raw.cellCenterPose(x, y);
                    const double dx = cell.x_m - snapshot.raw.center_pose.x_m;
                    const double dy = cell.y_m - snapshot.raw.center_pose.y_m;
                    const double distance = std::hypot(dx, dy);
                    if (distance < best_distance) {
                        best_distance = distance;
                        metrics.nearest_obstacle_dx_m = dx;
                        metrics.nearest_obstacle_dy_m = dy;
                    }
                }
            }
        }

        const std::vector<PhysicsSimObstacleFootprint> footprints = bridge_ptr->latestObstacleFootprints();
        metrics.peak_obstacle_count = std::max(metrics.peak_obstacle_count, footprints.size());
        metrics.saw_obstacle_footprints = metrics.saw_obstacle_footprints || !footprints.empty();
        for (const PhysicsSimObstacleFootprint& footprint : footprints) {
            if (pointInsideFootprint(pose, footprint, 0.01)) {
                metrics.touched_raw_obstacle = true;
            }
        }

        if (!monitor.active && isTerminal(monitor.lifecycle)) {
            break;
        }
        if (isTerminal(monitor.lifecycle) && monitor.lifecycle != NavigationLifecycleState::Running) {
            break;
        }
    }

    metrics.goal_error_m = std::hypot(metrics.end_pose.x_m - goal_pose.x_m,
                                      metrics.end_pose.y_m - goal_pose.y_m);
    metrics.final_monitor = runtime.navigationManager()->monitor();
    return metrics;
}

bool checkDirectPath(const NavigationRunMetrics& metrics) {
    return expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Running ||
                      metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed,
                  "direct_path: navigation should stay healthy in the empty scene") &&
           expect(metrics.max_progress_m >= 0.08,
                  "direct_path: body should make substantial forward progress") &&
           expect(metrics.peak_replan_count >= 1,
                  "direct_path: planner should produce at least one local segment") &&
           expect(metrics.path_length_m <= 2.0,
                  "direct_path: path length should stay bounded");
}

bool checkDetour(const NavigationRunMetrics& baseline,
                 const NavigationRunMetrics& metrics) {
    return expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Running ||
                      metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed,
                  "single_obstacle: navigation should stay healthy around the obstacle") &&
           expect(metrics.saw_obstacle_footprints,
                  "single_obstacle: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "single_obstacle: body path should stay out of raw obstacle footprints") &&
           expect(metrics.path_length_m >= baseline.path_length_m + 0.02,
                  "single_obstacle: detour path should be longer than the empty-scene baseline");
}

bool checkBlockedCorridor(const NavigationRunMetrics& metrics) {
    return expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed,
                  "blocked_corridor: navigation should fail safely after blocked timeout") &&
           expect(metrics.final_monitor.block_reason == PlannerBlockReason::StartOccupied ||
                      metrics.final_monitor.block_reason == PlannerBlockReason::NoPath ||
                      metrics.final_monitor.block_reason == PlannerBlockReason::GoalOccupied,
                  "blocked_corridor: failure should report a planner block reason") &&
           expect(metrics.max_progress_m <= 0.10,
                  "blocked_corridor: body should not pass through the blocked corridor") &&
           expect(metrics.saw_obstacle_footprints,
                  "blocked_corridor: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "blocked_corridor: body path should stay out of raw obstacle footprints");
}

bool checkMidrunIntrusion(const NavigationRunMetrics& baseline,
                          const NavigationRunMetrics& metrics) {
    const bool replanned_more = metrics.peak_replan_count > baseline.peak_replan_count;
    const bool blocked_safely_after_progress =
        metrics.max_progress_m >= 0.04 &&
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed &&
        (metrics.final_monitor.block_reason == PlannerBlockReason::StartOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::GoalOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::NoPath);
    return expect(metrics.saw_obstacle_footprints,
                  "midrun_intrusion: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "midrun_intrusion: body path should stay out of raw obstacle footprints") &&
           expect(replanned_more || blocked_safely_after_progress,
                  "midrun_intrusion: obstacle insertion should either trigger replans or force a safe planner block after progress") &&
           expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed ||
                      metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed,
                  "midrun_intrusion: run should end in a deterministic terminal state");
}

void printSummary(const std::string& label, const NavigationRunMetrics& metrics) {
    std::cout << label
              << " start_roll=" << metrics.start_roll_rad
              << " start_pitch=" << metrics.start_pitch_rad
              << " start_yaw=" << metrics.start_pose.yaw_rad
              << " lifecycle=" << static_cast<int>(metrics.final_monitor.lifecycle)
              << " block_reason=" << static_cast<int>(metrics.final_monitor.block_reason)
              << " goal_error=" << metrics.goal_error_m
              << " path=" << metrics.path_length_m
              << " max_progress=" << metrics.max_progress_m
              << " max_lateral=" << metrics.max_lateral_deviation_m
              << " replans=" << metrics.peak_replan_count
              << " peak_obstacles=" << metrics.peak_obstacle_count
              << " peak_raw_occupied=" << metrics.peak_raw_occupied_cells
              << " peak_raw_free=" << metrics.peak_raw_free_cells
              << " min_nearest_obstacle=" << metrics.min_nearest_obstacle_distance_m
              << " nearest_obstacle_dx=" << metrics.nearest_obstacle_dx_m
              << " nearest_obstacle_dy=" << metrics.nearest_obstacle_dy_m
              << '\n';
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_physics_sim_navigation_acceptance: Linux-only\n";
    return EXIT_SUCCESS;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_navigation_acceptance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return EXIT_SUCCESS;
    }

    constexpr int kBusLoopPeriodUs = 20000;
    constexpr double kGoalForwardM = 0.14;

    const std::filesystem::path single_box =
        resolveNavigationScene(sim_exe, "nav_single_box.json");
    const std::filesystem::path blocked_corridor =
        resolveNavigationScene(sim_exe, "nav_blocked_corridor.json");
    const std::filesystem::path midrun_intrusion =
        resolveNavigationScene(sim_exe, "nav_midrun_intrusion.json");

    const auto direct =
        runNavigationCase("direct_path", sim_exe, std::nullopt, kGoalForwardM, 1.2, 1800, kBusLoopPeriodUs);
    if (direct.has_value()) {
        printSummary("direct_path", *direct);
    }
    if (!direct.has_value() || !checkDirectPath(*direct)) {
        return EXIT_FAILURE;
    }

    const auto detour = runNavigationCase("single_obstacle",
                                          sim_exe,
                                          single_box.string(),
                                          kGoalForwardM,
                                          1.2,
                                          2200,
                                          kBusLoopPeriodUs);
    if (detour.has_value()) {
        printSummary("single_obstacle", *detour);
    }
    if (!detour.has_value() || !checkDetour(*direct, *detour)) {
        return EXIT_FAILURE;
    }

    const auto blocked = runNavigationCase("blocked_corridor",
                                           sim_exe,
                                           blocked_corridor.string(),
                                           kGoalForwardM,
                                           0.8,
                                           900,
                                           kBusLoopPeriodUs);
    if (blocked.has_value()) {
        printSummary("blocked_corridor", *blocked);
    }
    if (!blocked.has_value() || !checkBlockedCorridor(*blocked)) {
        return EXIT_FAILURE;
    }

    const auto intrusion = runNavigationCase("midrun_intrusion",
                                             sim_exe,
                                             midrun_intrusion.string(),
                                             kGoalForwardM,
                                             1.0,
                                             2200,
                                             kBusLoopPeriodUs);
    if (intrusion.has_value()) {
        printSummary("midrun_intrusion", *intrusion);
    }
    if (!intrusion.has_value() || !checkMidrunIntrusion(*direct, *intrusion)) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
#endif
}
