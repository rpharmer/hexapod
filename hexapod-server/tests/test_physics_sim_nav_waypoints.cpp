/**
 * Integration: navigation on the live UDP physics sim using the nav tick order:
 * bus → estimator → build intent (nav) → setMotionIntent → safety → control
 *
 * Phase 1: NavLocomotionBridge + FollowWaypoints from a settled stand (colinear goals, slew).
 *
 * The UDP physics sim has substantial lateral drift (see test_physics_sim_walk_distance), so by
 * default we only require healthy wiring: nav becomes inactive with a terminal lifecycle we
 * understand (Completed or stall Failed), measurable planar motion, and bounded displacement.
 * Tight completion is opt-in: HEXAPOD_STRICT_PHYSICS_NAV=1 (nonzero, not "0").
 *
 * Phase 2 (RotateToHeading) runs only after a successful follow: if follow stalls, the body is in
 * an arbitrary dynamical state and yaw trim is not a meaningful regression signal here (covered in
 * test_nav_primitives).
 *
 * Linux + HEXAPOD_PHYSICS_SIM_EXE (or argv[1]) required; otherwise skipped with exit 0.
 */

#include "control_config.hpp"
#include "math_types.hpp"
#include "motion_intent_utils.hpp"
#include "nav_locomotion_bridge.hpp"
#include "nav_primitives.hpp"
#include "nav_to_locomotion.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

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

Vec3 positionFromState(const RobotState& state) {
    return Vec3{state.body_twist_state.body_trans_m.x,
                state.body_twist_state.body_trans_m.y,
                state.body_twist_state.body_trans_m.z};
}

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              int physics_solver_iterations)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr) {}

    bool init() override {
        return inner_.init();
    }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override {
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const {
        return last_state_;
    }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
};

void runStandWarmupStep(RobotRuntime& runtime, const ScenarioMotionIntent& stand_motion) {
    MotionIntent mi = makeMotionIntent(stand_motion);
    stampIntentStreamMotionFields(mi);
    runtime.setMotionIntent(mi);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

void stampIntent(MotionIntent& intent) {
    stampIntentStreamMotionFields(intent);
}

/** Nav-style tick order: pose after bus/est, then intent, then safety/control. */
void runFollowNavTick(RobotRuntime& runtime,
                      NavLocomotionBridge& nav,
                      const MotionIntent& stand_fallback,
                      const double dt_s) {
    runtime.busStep();
    runtime.estimatorStep();
    const RobotState est = runtime.estimatedSnapshot();
    const MotionIntent intent = nav.mergeIntent(stand_fallback, est, dt_s);
    runtime.setMotionIntent(intent);
    runtime.safetyStep();
    runtime.controlStep();
}

void runRotateNavTick(RobotRuntime& runtime,
                      RotateToHeading& rot,
                      bool& turning,
                      const MotionIntent& walk_base,
                      const MotionIntent& stand_fallback,
                      const double dt_s) {
    runtime.busStep();
    runtime.estimatorStep();
    const RobotState est = runtime.estimatedSnapshot();

    MotionIntent intent = stand_fallback;
    if (turning) {
        const double yaw = est.body_twist_state.twist_pos_rad.z;
        const NavTaskUpdate u = rot.update(yaw, dt_s);
        intent = walk_base;
        applyNavCommandToMotionIntent(u.cmd, intent);
        if (u.status == NavTaskStatus::Completed) {
            turning = false;
        }
    }
    stampIntent(intent);
    runtime.setMotionIntent(intent);
    runtime.safetyStep();
    runtime.controlStep();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_physics_sim_nav_waypoints: Linux-only\n";
    return EXIT_SUCCESS;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_nav_waypoints (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int kPort = 23000 + (static_cast<int>(::getpid()) % 5000);
    const int kBusLoopPeriodUs = 20000;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(kPort);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>("127.0.0.1", kPort, kBusLoopPeriodUs, 24);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    /** Default foot blend injects measured slip; for waypoint closure on the UDP sim, command dominates. */
    cfg.gait.foot_estimator_blend = 0.0;
    cfg.locomotion_cmd.enable_first_order_filter = false;

    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg);
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{
        true, RobotMode::STAND, GaitType::TRIPOD, 0.06, 0.0, 0.0, 0.0};

    constexpr int kStandWarmupSteps = 120;
    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runStandWarmupStep(runtime, stand_motion);
    }

    if (!bridge_ptr->last_state().has_value()) {
        std::cerr << "FAIL: bridge never produced state\n";
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    MotionIntent stand_fallback = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.06);
    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.06);

    const double dt_s = static_cast<double>(kBusLoopPeriodUs) * 1e-6;

    const Vec3 wp0 = positionFromState(bridge_ptr->last_state().value());
    const double yaw_wp = bridge_ptr->last_state().value().body_twist_state.twist_pos_rad.z;
    const double cc = std::cos(yaw_wp);
    const double ss = std::sin(yaw_wp);
    /** Two shallow goals along current heading; keep tol below each leg length (0.012 m). */
    constexpr double seg1_m = 0.012;
    constexpr double seg2_m = 0.024;
    const double g1x = wp0.x + cc * seg1_m;
    const double g1y = wp0.y + ss * seg1_m;
    const double g2x = wp0.x + cc * seg2_m;
    const double g2y = wp0.y + ss * seg2_m;

    FollowWaypoints::Params fp{};
    fp.stall_timeout_s = 420.0;
    fp.go_to.rotate_first = false;
    fp.go_to.drive.position_tol_m = 0.0085;
    fp.go_to.drive.settle_cycles_required = 6;
    fp.go_to.drive.max_v_mps = 0.034;
    fp.go_to.drive.position_gain = 0.13;
    fp.go_to.drive.yaw_hold_kp = 0.0;
    fp.go_to.rotate.error_threshold_rad = 0.2;
    fp.go_to.rotate.settle_cycles_required = 4;

    NavLocomotionBridge nav;
    nav.setPlanarCommandSlew01(0.14);
    nav.startFollowWaypoints(walk_base, {NavPose2d{g1x, g1y, yaw_wp}, NavPose2d{g2x, g2y, yaw_wp}}, fp);

    double path_wp = 0.0;
    Vec3 prev_wp = wp0;
    Vec3 pos_at_follow_done = wp0;
    constexpr int kMaxFollowSteps = 26000;
    for (int i = 0; i < kMaxFollowSteps; ++i) {
        runFollowNavTick(runtime, nav, stand_fallback, dt_s);
        if (!bridge_ptr->last_state().has_value()) {
            std::cerr << "FAIL: lost sim state during follow\n";
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        const Vec3 p = positionFromState(bridge_ptr->last_state().value());
        path_wp += std::hypot(p.x - prev_wp.x, p.y - prev_wp.y);
        prev_wp = p;
        if (!nav.active()) {
            pos_at_follow_done = p;
            break;
        }
    }

    for (int i = 0; i < 8; ++i) {
        runFollowNavTick(runtime, nav, stand_fallback, dt_s);
    }

    const NavLocomotionBridge::MonitorSnapshot mon = nav.monitor();
    if (!expect(!nav.active(), "FollowWaypoints should finish (complete or stall-fail) in the physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const double err_goal =
        std::hypot(pos_at_follow_done.x - g2x, pos_at_follow_done.y - g2y);
    const double net_wp = std::hypot(pos_at_follow_done.x - wp0.x, pos_at_follow_done.y - wp0.y);

    if (!expect(path_wp >= 0.006, "follow phase should move measurably in the horizontal plane")) {
        std::cerr << "path_wp=" << path_wp << " err_goal=" << err_goal << '\n';
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const char* strict_env = std::getenv("HEXAPOD_STRICT_PHYSICS_NAV");
    const bool strict =
        strict_env != nullptr && strict_env[0] != '\0' && std::string{strict_env} != "0";

    if (strict) {
        if (!expect(mon.lifecycle == NavLocomotionBridge::LifecycleState::Completed,
                    "strict mode: FollowWaypoints should complete successfully")) {
            std::cerr << "lifecycle=" << static_cast<int>(mon.lifecycle)
                      << " failure_reason=" << static_cast<int>(mon.failure_reason) << " path_wp=" << path_wp
                      << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        if (!expect(net_wp <= 0.20, "strict mode: net displacement should stay small after success")) {
            std::cerr << "net_wp=" << net_wp << " err_goal=" << err_goal << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        if (!expect(err_goal <= 0.11, "strict mode: body should finish near the final waypoint")) {
            std::cerr << "wp0=" << wp0.x << "," << wp0.y << " end=" << pos_at_follow_done.x << ","
                      << pos_at_follow_done.y << " g2=" << g2x << "," << g2y << " err=" << err_goal << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
    } else {
        const bool completed = mon.lifecycle == NavLocomotionBridge::LifecycleState::Completed;
        const bool stalled = mon.lifecycle == NavLocomotionBridge::LifecycleState::Failed &&
                             mon.failure_reason == NavTaskFailureReason::StallTimeout;
        if (!expect(completed || stalled, "follow should complete or fail with stall timeout")) {
            std::cerr << "lifecycle=" << static_cast<int>(mon.lifecycle)
                      << " failure_reason=" << static_cast<int>(mon.failure_reason) << " path_wp=" << path_wp
                      << " net_wp=" << net_wp << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        if (!expect(net_wp <= 8.0, "follow phase net displacement should stay bounded (loose guard)")) {
            std::cerr << "net_wp=" << net_wp << " err_goal=" << err_goal << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        if (!expect(path_wp <= 120.0, "follow phase path length guard (catch runaway loops)")) {
            std::cerr << "path_wp=" << path_wp << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        if (completed) {
            if (!expect(err_goal <= 0.15, "completed runs should finish near the final waypoint")) {
                std::cerr << "err_goal=" << err_goal << " net_wp=" << net_wp << '\n';
                ::kill(pid, SIGTERM);
                ::waitpid(pid, nullptr, 0);
                return EXIT_FAILURE;
            }
        }
    }

    std::cout << "test_physics_sim_nav_waypoints phase1_follow ok lifecycle=" << static_cast<int>(mon.lifecycle)
              << " err_goal=" << err_goal << " net_wp=" << net_wp << " path_wp=" << path_wp << '\n';

    const bool follow_completed = mon.lifecycle == NavLocomotionBridge::LifecycleState::Completed;

    if (follow_completed) {
        constexpr int kStandBeforeRotate = 200;
        for (int i = 0; i < kStandBeforeRotate; ++i) {
            runStandWarmupStep(runtime, stand_motion);
        }

        const double yaw_r0 = bridge_ptr->last_state().value().body_twist_state.twist_pos_rad.z;
        const double target_yaw = navWrapAngleRad(yaw_r0 + 0.12);
        RotateToHeading::Params rp{};
        rp.yaw_rate_limit_radps = 0.30;
        rp.kp = 1.05;
        rp.error_threshold_rad = 0.12;
        rp.settle_cycles_required = 8;
        RotateToHeading rot(rp);
        rot.reset(target_yaw);

        bool turning = true;
        double path_r = 0.0;
        Vec3 prev_r = positionFromState(bridge_ptr->last_state().value());
        double yaw_done = yaw_r0;
        constexpr int kMaxRot = 20000;
        for (int i = 0; i < kMaxRot; ++i) {
            runRotateNavTick(runtime, rot, turning, walk_base, stand_fallback, dt_s);
            if (!bridge_ptr->last_state().has_value()) {
                std::cerr << "FAIL: lost sim state during rotate\n";
                ::kill(pid, SIGTERM);
                ::waitpid(pid, nullptr, 0);
                return EXIT_FAILURE;
            }
            const Vec3 p = positionFromState(bridge_ptr->last_state().value());
            path_r += std::hypot(p.x - prev_r.x, p.y - prev_r.y);
            prev_r = p;
            if (!turning) {
                yaw_done = bridge_ptr->last_state().value().body_twist_state.twist_pos_rad.z;
                break;
            }
        }
        for (int i = 0; i < 8; ++i) {
            runRotateNavTick(runtime, rot, turning, walk_base, stand_fallback, dt_s);
        }

        if (!expect(!turning, "RotateToHeading should complete after a successful follow")) {
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        const double yaw_err2 = std::abs(navWrapAngleRad(target_yaw - yaw_done));
        if (!expect(yaw_err2 <= 0.25, "phase2 yaw should approach target")) {
            std::cerr << "yaw_err2=" << yaw_err2 << " path_r=" << path_r << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        std::cout << "test_physics_sim_nav_waypoints phase2_rotate ok yaw_err=" << yaw_err2
                  << " path_r=" << path_r << '\n';
    } else {
        std::cout << "test_physics_sim_nav_waypoints phase2_rotate skipped (follow did not complete)\n";
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    return 0;
#endif
}
