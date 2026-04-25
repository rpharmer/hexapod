#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_ik.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

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

struct HoldMetrics {
    double mean_body_height_m{0.0};
    double min_body_height_m{1e9};
    double max_abs_roll_rad{0.0};
    double max_abs_pitch_rad{0.0};
    RobotState last_state{};
};

JointTargets buildStandTargets() {
    BodyController body{};
    LegIK ik(defaultHexapodGeometry());

    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);

    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    const LegTargets foot_targets = body.update(est, stand, gait, safety, cmd_twist, nullptr);
    return ik.solve(est, foot_targets, safety);
}

JointTargets buildTripodRaisedTargets() {
    BodyController body{};
    LegIK ik(defaultHexapodGeometry());
    const HexapodGeometry geometry = defaultHexapodGeometry();

    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);

    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    LegTargets foot_targets = body.update(est, stand, gait, safety, cmd_twist, nullptr);

    constexpr std::array<int, 3> kRaisedLegs{{1, 3, 5}};
    for (const int leg : kRaisedLegs) {
        const Vec3 coxa = geometry.legGeometry[leg].bodyCoxaOffset;
        const Vec3 rel = foot_targets.feet[leg].pos_body_m - coxa;
        foot_targets.feet[leg].pos_body_m = coxa + Vec3{
            rel.x * 0.72,
            rel.y * 0.72,
            rel.z + 0.055,
        };
        foot_targets.feet[leg].vel_body_mps = Vec3{};
    }

    return ik.solve(est, foot_targets, safety);
}

bool finiteJointTargets(const JointTargets& joints) {
    for (const auto& leg : joints.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

HoldMetrics holdPose(PhysicsSimBridge& bridge, const JointTargets& targets, int steps) {
    HoldMetrics metrics{};
    int samples = 0;
    for (int i = 0; i < steps; ++i) {
        if (!bridge.write(targets)) {
            break;
        }
        RobotState state{};
        if (!bridge.read(state)) {
            break;
        }
        const double z = state.body_twist_state.body_trans_m.z;
        metrics.mean_body_height_m += z;
        metrics.min_body_height_m = std::min(metrics.min_body_height_m, z);
        metrics.max_abs_roll_rad = std::max(metrics.max_abs_roll_rad, std::abs(state.body_twist_state.twist_pos_rad.x));
        metrics.max_abs_pitch_rad =
            std::max(metrics.max_abs_pitch_rad, std::abs(state.body_twist_state.twist_pos_rad.y));
        metrics.last_state = state;
        ++samples;
    }

    if (samples > 0) {
        metrics.mean_body_height_m /= static_cast<double>(samples);
    } else {
        metrics.min_body_height_m = 0.0;
    }
    return metrics;
}

void printMetrics(const std::string& label, const HoldMetrics& metrics) {
    std::cout << label
              << " mean_height_m=" << metrics.mean_body_height_m
              << " min_height_m=" << metrics.min_body_height_m
              << " max_roll_rad=" << metrics.max_abs_roll_rad
              << " max_pitch_rad=" << metrics.max_abs_pitch_rad
              << '\n';
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_tripod_support_baseline (Linux-only)\n";
    return 0;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_tripod_support_baseline (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int port = 24000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = 20000;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    PhysicsSimBridge bridge("127.0.0.1", port, bus_loop_period_us, 24, nullptr);
    if (!expect(bridge.init(), "physics sim bridge should initialize")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const JointTargets stand_targets = buildStandTargets();
    const JointTargets tripod_targets = buildTripodRaisedTargets();
    if (!expect(finiteJointTargets(stand_targets), "standing joint targets should be finite") ||
        !expect(finiteJointTargets(tripod_targets), "tripod-raised joint targets should be finite")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    constexpr int kStandWarmupSteps = 160;
    constexpr int kTripodWarmupSteps = 260;
    constexpr int kMetricsSteps = 180;

    (void)holdPose(bridge, stand_targets, kStandWarmupSteps);
    const HoldMetrics stand_metrics = holdPose(bridge, stand_targets, kMetricsSteps);

    (void)holdPose(bridge, tripod_targets, kTripodWarmupSteps);
    const HoldMetrics tripod_metrics = holdPose(bridge, tripod_targets, kMetricsSteps);

    printMetrics("six_leg_stand", stand_metrics);
    printMetrics("tripod_support", tripod_metrics);

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    const double sag_vs_stand_m = stand_metrics.mean_body_height_m - tripod_metrics.mean_body_height_m;

    return expect(stand_metrics.mean_body_height_m > 0.10,
                  "six-leg baseline should settle near nominal body height") &&
           expect(tripod_metrics.mean_body_height_m > 0.075,
                  "tripod support should keep the body well above a collapse height") &&
           expect(tripod_metrics.min_body_height_m > 0.06,
                  "tripod support should not let the body crash near the ground") &&
           expect(sag_vs_stand_m < 0.05,
                  "tripod support should sag noticeably less than 5 cm versus six-leg stand") &&
           expect(tripod_metrics.max_abs_roll_rad < 0.45,
                  "tripod support should keep roll within a moderate bound") &&
           expect(tripod_metrics.max_abs_pitch_rad < 0.45,
                  "tripod support should keep pitch within a moderate bound")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
#endif
}
