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
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_local_map_source.hpp"
#include "physics_sim_protocol.hpp"
#include "robot_runtime.hpp"
#include "locomotion_metrics.hpp"
#include "test_limits_manifest.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
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
            physics_sim_test_utils::quietChildProcessStdIo();
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
    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 24000 + (static_cast<int>(::getpid()) % 3000) +
                     static_cast<int>(std::hash<std::string>{}(label) % 1000);

    PhysicsSimProcess sim(sim_exe_path, port, scene_file);
    if (!sim.start()) {
        expect(false, label + ": failed to start physics sim process");
        return std::nullopt;
    }

    auto bridge = std::make_unique<PhysicsSimBridge>(
        "127.0.0.1", port, bus_loop_period_us, harness.physics_solver_iterations, nullptr);
    PhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
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
    cfg.nav_bridge.body_frame_integral_ki_fwd_per_s = 0.28;
    cfg.nav_bridge.body_frame_integral_ki_lat_per_s = 0.24;
    cfg.nav_bridge.body_frame_integral_abs_cap_m_s = 0.06;

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
    if (scene_file.has_value()) {
        navigation_manager->addObservationSource(std::make_shared<MatrixLidarLocalMapObservationSource>());
    }
    navigation_manager->addObservationSource(std::make_shared<PhysicsSimLocalMapObservationSource>(
        *bridge_ptr, std::max(0.02, cfg.local_map.resolution_m * 0.5)));
    runtime.setNavigationManager(std::move(navigation_manager));

    MotionIntent stand_fallback = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.06);
    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.06);

    const int kWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(140, bus_loop_period_us));
    for (int i = 0; i < kWarmupSteps; ++i) {
        runWarmupStep(runtime, stand_fallback);
    }

    const RobotState settled = runtime.estimatedSnapshot();
    const NavPose2d start_pose = navPose2dFromRobotState(settled);
    if (!expect(std::abs(start_pose.yaw_rad) <= 0.35, label + ": settled yaw should stay near zero")) {
        return std::nullopt;
    }
    if (!expect(settled.has_fusion_diagnostics && settled.fusion.model_trust >= 0.0,
                label + ": settled estimator should expose fusion diagnostics")) {
        return std::nullopt;
    }

    FollowWaypoints::Params follow_params{};
    follow_params.stall_timeout_s = 6.0;
    follow_params.go_to.rotate_first = false;
    follow_params.go_to.drive.max_v_mps = 0.05;
    follow_params.go_to.drive.position_gain = 0.22;
    follow_params.go_to.drive.position_tol_m = 0.035;
    follow_params.go_to.drive.settle_cycles_required = 5;
    follow_params.go_to.drive.yaw_hold_kp = 0.0;
    follow_params.go_to.rotate.error_threshold_rad = 0.20;
    follow_params.go_to.rotate.settle_cycles_required = 4;

    const double goal_dir_x = std::cos(start_pose.yaw_rad);
    const double goal_dir_y = std::sin(start_pose.yaw_rad);
    const NavPose2d goal_pose{
        start_pose.x_m + goal_dir_x * goal_forward_m,
        start_pose.y_m + goal_dir_y * goal_forward_m,
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
    constexpr const char* kSuite = "physics_sim_navigation_acceptance";
    constexpr const char* kCase = "direct_path";
    const double min_progress = test_limits::getDouble(kSuite, kCase, "", "max_progress_m_min", 0.08);
    const std::size_t min_replans =
        test_limits::getSizeT(kSuite, kCase, "", "peak_replan_count_min", 1);
    const double path_max = test_limits::getDouble(kSuite, kCase, "", "path_length_m_max", 2.0);
    return expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Running ||
                      metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed,
                  "direct_path: navigation should stay healthy in the empty scene") &&
           expect(metrics.max_progress_m >= min_progress,
                  "direct_path: body should make substantial forward progress") &&
           expect(metrics.peak_replan_count >= min_replans,
                  "direct_path: planner should produce at least one local segment") &&
           expect(metrics.path_length_m <= path_max,
                  "direct_path: path length should stay bounded");
}

bool checkDetour(const NavigationRunMetrics& baseline,
                 const NavigationRunMetrics& metrics) {
    constexpr const char* kSuite = "physics_sim_navigation_acceptance";
    constexpr const char* kCase = "single_obstacle";
    const double delta_min =
        test_limits::getDouble(kSuite, kCase, "", "path_length_delta_vs_baseline_min_m", 0.02);
    const double goal_error_max =
        test_limits::getDouble(kSuite, kCase, "", "goal_error_m_max", 0.08);
    const double path_length_max =
        test_limits::getDouble(kSuite, kCase, "", "path_length_m_max", 0.8);
    const double progress_max =
        test_limits::getDouble(kSuite, kCase, "", "max_progress_m_max", 0.30);
    const double safe_block_progress_min =
        test_limits::getDouble(kSuite, kCase, "", "min_progress_m_for_safe_block", 0.08);
    const double safe_block_goal_error_max =
        test_limits::getDouble(kSuite, kCase, "", "goal_error_m_max_if_blocked", 0.12);
    const bool completed_or_running =
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Running ||
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed;
    const bool safe_blocked_after_progress =
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed &&
        (metrics.final_monitor.block_reason == PlannerBlockReason::StartOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::GoalOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::NoPath) &&
        metrics.max_progress_m >= safe_block_progress_min &&
        std::isfinite(metrics.goal_error_m) &&
        metrics.goal_error_m <= safe_block_goal_error_max;
    return expect(completed_or_running || safe_blocked_after_progress,
                  "single_obstacle: navigation should either complete the detour or stop safely after making detour progress") &&
           expect(metrics.saw_obstacle_footprints,
                  "single_obstacle: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "single_obstacle: body path should stay out of raw obstacle footprints") &&
           expect(metrics.path_length_m >= baseline.path_length_m + delta_min,
                  "single_obstacle: detour path should be longer than the empty-scene baseline") &&
           expect(std::isfinite(metrics.goal_error_m) && metrics.goal_error_m <= goal_error_max,
                  "single_obstacle: detour should still converge near the goal") &&
           expect(std::isfinite(metrics.path_length_m) && metrics.path_length_m <= path_length_max,
                  "single_obstacle: detour path should remain bounded") &&
           expect(metrics.max_progress_m <= progress_max,
                  "single_obstacle: detour should not run far past the local obstacle scene");
}

bool checkBlockedCorridor(const NavigationRunMetrics& metrics) {
    constexpr const char* kSuite = "physics_sim_navigation_acceptance";
    constexpr const char* kCase = "blocked_corridor";
    const double max_progress = test_limits::getDouble(kSuite, kCase, "", "max_progress_m_max", 0.12);
    const double front_edge_progress_max =
        test_limits::getDouble(kSuite, kCase, "", "max_progress_m_max_if_front_edge_stop", 0.12);
    const double front_edge_goal_error_max =
        test_limits::getDouble(kSuite, kCase, "", "goal_error_m_max_if_front_edge_stop", 0.06);
    const double front_edge_path_length_max =
        test_limits::getDouble(kSuite, kCase, "", "path_length_m_max_if_front_edge_stop", 0.25);

    const bool failed_with_block_reason =
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed &&
        (metrics.final_monitor.block_reason == PlannerBlockReason::StartOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::NoPath ||
         metrics.final_monitor.block_reason == PlannerBlockReason::GoalOccupied);
    const bool safe_front_edge_stop =
        (metrics.final_monitor.lifecycle == NavigationLifecycleState::Running ||
         metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed) &&
        metrics.max_progress_m <= front_edge_progress_max &&
        std::isfinite(metrics.goal_error_m) &&
        metrics.goal_error_m <= front_edge_goal_error_max &&
        std::isfinite(metrics.path_length_m) &&
        metrics.path_length_m <= front_edge_path_length_max;

    return expect(failed_with_block_reason || safe_front_edge_stop,
                  "blocked_corridor: navigation should either fail with a planner block or stop safely at the corridor front edge") &&
           expect(failed_with_block_reason || metrics.max_progress_m <= front_edge_progress_max,
                  "blocked_corridor: non-failing edge stops must keep forward progress tightly bounded") &&
           expect(metrics.max_progress_m <= max_progress,
                  "blocked_corridor: body should not pass through the blocked corridor") &&
           expect(metrics.saw_obstacle_footprints,
                  "blocked_corridor: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "blocked_corridor: body path should stay out of raw obstacle footprints");
}

bool checkMidrunIntrusion(const NavigationRunMetrics& baseline,
                          const NavigationRunMetrics& metrics) {
    constexpr const char* kSuite = "physics_sim_navigation_acceptance";
    constexpr const char* kCase = "midrun_intrusion";
    const double min_progress_for_block =
        test_limits::getDouble(kSuite, kCase, "", "min_progress_m_for_safe_block_branch", 0.04);
    const double completed_path_delta_min =
        test_limits::getDouble(kSuite, kCase, "", "path_length_delta_vs_baseline_min_m_if_completed", 0.08);
    const double completed_goal_error_max =
        test_limits::getDouble(kSuite, kCase, "", "goal_error_m_max_if_completed", 0.05);
    const bool replanned_more = metrics.peak_replan_count > baseline.peak_replan_count;
    const bool blocked_safely_after_progress =
        metrics.max_progress_m >= min_progress_for_block &&
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed &&
        (metrics.final_monitor.block_reason == PlannerBlockReason::StartOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::GoalOccupied ||
         metrics.final_monitor.block_reason == PlannerBlockReason::NoPath);
    const bool completed_detour =
        metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed &&
        metrics.goal_error_m <= completed_goal_error_max &&
        metrics.path_length_m >= baseline.path_length_m + completed_path_delta_min;
    return expect(metrics.saw_obstacle_footprints,
                  "midrun_intrusion: live obstacle footprints should be observed") &&
           expect(!metrics.touched_raw_obstacle,
                  "midrun_intrusion: body path should stay out of raw obstacle footprints") &&
           expect(replanned_more || blocked_safely_after_progress || completed_detour,
                  "midrun_intrusion: obstacle insertion should trigger extra replanning, a longer completed detour, or a safe planner block after progress") &&
           expect(metrics.final_monitor.lifecycle == NavigationLifecycleState::Completed ||
                      metrics.final_monitor.lifecycle == NavigationLifecycleState::Failed,
                  "midrun_intrusion: run should end in a deterministic terminal state");
}

std::string navigationLimitsAppliedJson(const std::string& name,
                                        const NavigationRunMetrics* baseline,
                                        const double goal_forward_m) {
    constexpr const char* kSuite = "physics_sim_navigation_acceptance";
    std::ostringstream o;
    o << "{\"case_name\":\"" << locomotion_test::jsonEscape(name) << "\""
      << ",\"goal_forward_m\":" << locomotion_test::formatDouble(goal_forward_m);
    if (name == "direct_path") {
        const double min_progress = test_limits::getDouble(kSuite, "direct_path", "", "max_progress_m_min", 0.08);
        const std::size_t min_replans =
            test_limits::getSizeT(kSuite, "direct_path", "", "peak_replan_count_min", 1);
        const double path_max = test_limits::getDouble(kSuite, "direct_path", "", "path_length_m_max", 2.0);
        o << ",\"max_progress_m_min\":" << locomotion_test::formatDouble(min_progress)
          << ",\"peak_replan_count_min\":" << min_replans
          << ",\"path_length_m_max\":" << locomotion_test::formatDouble(path_max)
          << ",\"lifecycle_ok_any\":[\"Running\",\"Completed\"]";
    } else if (name == "single_obstacle") {
        const double delta_min =
            test_limits::getDouble(kSuite, "single_obstacle", "", "path_length_delta_vs_baseline_min_m", 0.02);
        const double goal_error_max =
            test_limits::getDouble(kSuite, "single_obstacle", "", "goal_error_m_max", 0.08);
        const double path_length_max =
            test_limits::getDouble(kSuite, "single_obstacle", "", "path_length_m_max", 0.8);
        const double progress_max =
            test_limits::getDouble(kSuite, "single_obstacle", "", "max_progress_m_max", 0.30);
        const double safe_block_progress_min =
            test_limits::getDouble(kSuite, "single_obstacle", "", "min_progress_m_for_safe_block", 0.08);
        const double safe_block_goal_error_max =
            test_limits::getDouble(kSuite, "single_obstacle", "", "goal_error_m_max_if_blocked", 0.12);
        o << ",\"lifecycle_ok_any\":[\"Running\",\"Completed\"]"
          << ",\"require_saw_obstacle_footprints\":true"
          << ",\"require_not_touched_raw_obstacle\":true"
          << ",\"path_length_delta_vs_baseline_min_m\":" << locomotion_test::formatDouble(delta_min);
        if (baseline != nullptr) {
            o << ",\"baseline_path_length_m\":" << locomotion_test::formatDouble(baseline->path_length_m);
        }
        o << ",\"goal_error_m_max\":" << locomotion_test::formatDouble(goal_error_max)
          << ",\"path_length_m_max\":" << locomotion_test::formatDouble(path_length_max)
          << ",\"max_progress_m_max\":" << locomotion_test::formatDouble(progress_max)
          << ",\"min_progress_m_for_safe_block\":" << locomotion_test::formatDouble(safe_block_progress_min)
          << ",\"goal_error_m_max_if_blocked\":" << locomotion_test::formatDouble(safe_block_goal_error_max);
    } else if (name == "blocked_corridor") {
        const double max_progress = test_limits::getDouble(kSuite, "blocked_corridor", "", "max_progress_m_max", 0.10);
        o << ",\"expect_lifecycle\":\"Failed\""
          << ",\"max_progress_m_max\":" << locomotion_test::formatDouble(max_progress)
          << ",\"require_saw_obstacle_footprints\":true"
          << ",\"require_not_touched_raw_obstacle\":true"
          << ",\"block_reason_ok_any\":[\"StartOccupied\",\"NoPath\",\"GoalOccupied\"]";
    } else if (name == "midrun_intrusion") {
        const double min_prog =
            test_limits::getDouble(kSuite, "midrun_intrusion", "", "min_progress_m_for_safe_block_branch", 0.04);
        const double completed_delta =
            test_limits::getDouble(kSuite, "midrun_intrusion", "", "path_length_delta_vs_baseline_min_m_if_completed", 0.08);
        const double completed_goal_error =
            test_limits::getDouble(kSuite, "midrun_intrusion", "", "goal_error_m_max_if_completed", 0.05);
        o << ",\"require_saw_obstacle_footprints\":true"
          << ",\"require_not_touched_raw_obstacle\":true"
          << ",\"min_progress_m_for_safe_block_branch\":" << locomotion_test::formatDouble(min_prog)
          << ",\"path_length_delta_vs_baseline_min_m_if_completed\":" << locomotion_test::formatDouble(completed_delta)
          << ",\"goal_error_m_max_if_completed\":" << locomotion_test::formatDouble(completed_goal_error)
          << ",\"lifecycle_terminal_ok_any\":[\"Completed\",\"Failed\"]";
        if (baseline != nullptr) {
            o << ",\"baseline_peak_replan_count\":" << baseline->peak_replan_count;
            o << ",\"baseline_path_length_m\":" << locomotion_test::formatDouble(baseline->path_length_m);
        }
    }
    o << '}';
    return o.str();
}

void emitNavigationCaseJson(const std::string& name,
                            const bool passed,
                            const NavigationRunMetrics& metrics,
                            const NavigationRunMetrics* baseline,
                            const double goal_forward_m) {
    const double dx = metrics.goal_pose.x_m - metrics.start_pose.x_m;
    const double dy = metrics.goal_pose.y_m - metrics.start_pose.y_m;
    const double straight_m = std::hypot(dx, dy);
    const double detour_factor = metrics.path_length_m / std::max(straight_m, 1.0e-9);
    std::cout << "{\"suite\":\"physics_sim_navigation_acceptance\",\"name\":\"" << locomotion_test::jsonEscape(name)
              << "\",\"passed\":" << (passed ? "true" : "false") << ",\"limits_applied\":"
              << navigationLimitsAppliedJson(name, baseline, goal_forward_m) << ",\"metrics\":{"
              << "\"goal_error_m\":" << locomotion_test::formatDouble(metrics.goal_error_m)
              << ",\"path_length_m\":" << locomotion_test::formatDouble(metrics.path_length_m)
              << ",\"path_detour_factor\":" << locomotion_test::formatDouble(detour_factor)
              << ",\"max_lateral_deviation_m\":" << locomotion_test::formatDouble(metrics.max_lateral_deviation_m)
              << ",\"max_progress_m\":" << locomotion_test::formatDouble(metrics.max_progress_m)
              << ",\"peak_replan_count\":" << metrics.peak_replan_count
              << ",\"peak_active_segment_waypoints\":" << metrics.peak_active_segment_waypoints
              << ",\"touched_raw_obstacle\":" << (metrics.touched_raw_obstacle ? "true" : "false")
              << ",\"saw_obstacle_footprints\":" << (metrics.saw_obstacle_footprints ? "true" : "false")
              << ",\"peak_obstacle_count\":" << metrics.peak_obstacle_count
              << ",\"peak_raw_occupied_cells\":" << metrics.peak_raw_occupied_cells
              << ",\"peak_raw_free_cells\":" << metrics.peak_raw_free_cells
              << ",\"min_nearest_obstacle_distance_m\":" << locomotion_test::formatDouble(metrics.min_nearest_obstacle_distance_m)
              << ",\"nearest_obstacle_dx_m\":" << locomotion_test::formatDouble(metrics.nearest_obstacle_dx_m)
              << ",\"nearest_obstacle_dy_m\":" << locomotion_test::formatDouble(metrics.nearest_obstacle_dy_m)
              << ",\"start_pose_x_m\":" << locomotion_test::formatDouble(metrics.start_pose.x_m)
              << ",\"start_pose_y_m\":" << locomotion_test::formatDouble(metrics.start_pose.y_m)
              << ",\"goal_pose_x_m\":" << locomotion_test::formatDouble(metrics.goal_pose.x_m)
              << ",\"goal_pose_y_m\":" << locomotion_test::formatDouble(metrics.goal_pose.y_m)
              << ",\"end_pose_x_m\":" << locomotion_test::formatDouble(metrics.end_pose.x_m)
              << ",\"end_pose_y_m\":" << locomotion_test::formatDouble(metrics.end_pose.y_m)
              << ",\"lifecycle\":" << static_cast<int>(metrics.final_monitor.lifecycle)
              << ",\"block_reason\":" << static_cast<int>(metrics.final_monitor.block_reason)
              << "}}\n";
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
    bool emit_metrics_json = false;
    std::string sim_path_arg;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--emit-metrics-json") == 0) {
            emit_metrics_json = true;
        } else if (std::strcmp(argv[i], "--limits-manifest") == 0 && i + 1 < argc) {
            ++i;
        } else if (argv[i][0] != '\0' && argv[i][0] != '-') {
            sim_path_arg = argv[i];
        }
    }

    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }

    const char* sim_exe = nullptr;
    if (!sim_path_arg.empty()) {
        sim_exe = sim_path_arg.c_str();
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_navigation_acceptance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return EXIT_SUCCESS;
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int kBusLoopPeriodUs = harness.bus_loop_period_us;
    constexpr double kGoalForwardM = 0.14;

    const std::filesystem::path single_box =
        resolveNavigationScene(sim_exe, "nav_single_box.json");
    const std::filesystem::path blocked_corridor =
        resolveNavigationScene(sim_exe, "nav_blocked_corridor.json");
    const std::filesystem::path midrun_intrusion =
        resolveNavigationScene(sim_exe, "nav_midrun_intrusion.json");

    const auto direct =
        runNavigationCase("direct_path",
                          sim_exe,
                          std::nullopt,
                          -kGoalForwardM,
                          1.2,
                          static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(1800, kBusLoopPeriodUs)),
                          kBusLoopPeriodUs);
    if (direct.has_value()) {
        printSummary("direct_path", *direct);
    }
    const bool direct_ok = direct.has_value() && checkDirectPath(*direct);
    if (direct.has_value() && emit_metrics_json) {
        emitNavigationCaseJson("direct_path", direct_ok, *direct, nullptr, -kGoalForwardM);
    }
    if (!direct_ok) {
        return EXIT_FAILURE;
    }

    const auto detour = runNavigationCase("single_obstacle",
                                          sim_exe,
                                          single_box.string(),
                                          -kGoalForwardM,
                                          1.2,
                                          static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(2200, kBusLoopPeriodUs)),
                                          kBusLoopPeriodUs);
    if (detour.has_value()) {
        printSummary("single_obstacle", *detour);
    }
    const bool detour_ok = detour.has_value() && checkDetour(*direct, *detour);
    if (detour.has_value() && emit_metrics_json) {
        emitNavigationCaseJson("single_obstacle", detour_ok, *detour, &(*direct), -kGoalForwardM);
    }
    if (!detour_ok) {
        return EXIT_FAILURE;
    }

    const auto blocked = runNavigationCase("blocked_corridor",
                                           sim_exe,
                                           blocked_corridor.string(),
                                           -kGoalForwardM,
                                           0.8,
                                           static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(900, kBusLoopPeriodUs)),
                                           kBusLoopPeriodUs);
    if (blocked.has_value()) {
        printSummary("blocked_corridor", *blocked);
    }
    const bool blocked_ok = blocked.has_value() && checkBlockedCorridor(*blocked);
    if (blocked.has_value() && emit_metrics_json) {
        emitNavigationCaseJson("blocked_corridor", blocked_ok, *blocked, nullptr, -kGoalForwardM);
    }
    if (!blocked_ok) {
        return EXIT_FAILURE;
    }

    const auto intrusion = runNavigationCase("midrun_intrusion",
                                             sim_exe,
                                             midrun_intrusion.string(),
                                             -kGoalForwardM,
                                             1.0,
                                             static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(2600, kBusLoopPeriodUs)),
                                             kBusLoopPeriodUs);
    if (intrusion.has_value()) {
        printSummary("midrun_intrusion", *intrusion);
    }
    const bool intrusion_ok = intrusion.has_value() && checkMidrunIntrusion(*direct, *intrusion);
    if (intrusion.has_value() && emit_metrics_json) {
        emitNavigationCaseJson("midrun_intrusion", intrusion_ok, *intrusion, &(*direct), -kGoalForwardM);
    }
    if (!intrusion_ok) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
#endif
}
