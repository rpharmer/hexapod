#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_ik.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
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

std::atomic<bool> g_exit{false};

struct HoldMetrics {
    double mean_body_height_m{0.0};
    double min_body_height_m{1e9};
    double max_abs_roll_rad{0.0};
    double max_abs_pitch_rad{0.0};
    int samples{0};
};

void signalHandler(int)
{
    g_exit.store(true);
}

JointTargets buildStandTargets()
{
    BodyController body{};
    LegIK ik(defaultHexapodGeometry());

    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);

    const MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    const GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    const LegTargets foot_targets = body.update(est, stand, gait, safety, cmd_twist, nullptr);
    return ik.solve(est, foot_targets, safety);
}

JointTargets buildTripodRaisedTargets()
{
    BodyController body{};
    LegIK ik(defaultHexapodGeometry());
    const HexapodGeometry geometry = defaultHexapodGeometry();

    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);

    const MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    const GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    LegTargets foot_targets = body.update(est, stand, gait, safety, cmd_twist, nullptr);

    constexpr std::array<int, 3> kRaisedLegs{{1, 3, 5}};
    for (const int leg : kRaisedLegs) {
        const Vec3 coxa = geometry.legGeometry[leg].bodyCoxaOffset;
        const Vec3 rel = foot_targets.feet[leg].pos_body_m - coxa;
        foot_targets.feet[leg].pos_body_m = coxa + Vec3{rel.x * 0.72, rel.y * 0.72, rel.z + 0.055};
        foot_targets.feet[leg].vel_body_mps = Vec3{};
    }

    return ik.solve(est, foot_targets, safety);
}

bool finiteJointTargets(const JointTargets& joints)
{
    for (const auto& leg : joints.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool stepPose(PhysicsSimBridge& bridge, const JointTargets& targets, HoldMetrics& metrics)
{
    if (!bridge.write(targets)) {
        return false;
    }
    RobotState state{};
    if (!bridge.read(state)) {
        return false;
    }

    const double z = state.body_twist_state.body_trans_m.z;
    metrics.mean_body_height_m += z;
    metrics.min_body_height_m = std::min(metrics.min_body_height_m, z);
    metrics.max_abs_roll_rad = std::max(metrics.max_abs_roll_rad, std::abs(state.body_twist_state.twist_pos_rad.x));
    metrics.max_abs_pitch_rad = std::max(metrics.max_abs_pitch_rad, std::abs(state.body_twist_state.twist_pos_rad.y));
    ++metrics.samples;
    return true;
}

void printMetrics(const std::string& label, const HoldMetrics& metrics)
{
    const double mean_height =
        metrics.samples > 0 ? metrics.mean_body_height_m / static_cast<double>(metrics.samples) : 0.0;
    const double min_height = metrics.samples > 0 ? metrics.min_body_height_m : 0.0;
    std::cout << label
              << " mean_height_m=" << mean_height
              << " min_height_m=" << min_height
              << " max_roll_rad=" << metrics.max_abs_roll_rad
              << " max_pitch_rad=" << metrics.max_abs_pitch_rad
              << '\n';
}

void usage()
{
    std::cout
        << "Usage: hexapod-tripod-support-baseline [--sim-exe <path>] [--stand-ms <ms>] [--tripod-ms <ms>] [--indefinite]\n"
        << "  Spawns hexapod-physics-sim in serve mode, holds a six-leg stand, then a tripod-raised pose.\n";
}

} // namespace

int main(int argc, char** argv)
{
#if !defined(__linux__)
    std::cerr << "hexapod-tripod-support-baseline is Linux-only\n";
    return EXIT_FAILURE;
#else
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const char* sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    int stand_ms = 3000;
    int tripod_ms = 15000;
    bool indefinite = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sim-exe") == 0 && i + 1 < argc) {
            sim_exe = argv[++i];
        } else if (std::strcmp(argv[i], "--stand-ms") == 0 && i + 1 < argc) {
            stand_ms = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--tripod-ms") == 0 && i + 1 < argc) {
            tripod_ms = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--indefinite") == 0) {
            indefinite = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            usage();
            return EXIT_SUCCESS;
        } else {
            std::cerr << "Unknown argument: " << argv[i] << '\n';
            usage();
            return EXIT_FAILURE;
        }
    }

    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cerr << "Missing physics sim executable. Pass --sim-exe or set HEXAPOD_PHYSICS_SIM_EXE.\n";
        return EXIT_FAILURE;
    }

    const JointTargets stand_targets = buildStandTargets();
    const JointTargets tripod_targets = buildTripodRaisedTargets();
    if (!finiteJointTargets(stand_targets) || !finiteJointTargets(tripod_targets)) {
        std::cerr << "Failed to build finite joint targets for baseline poses\n";
        return EXIT_FAILURE;
    }

    const int port = 26000 + (static_cast<int>(::getpid()) % 2000);
    pid_t sim_pid = ::fork();
    if (sim_pid < 0) {
        std::perror("fork");
        return EXIT_FAILURE;
    }
    if (sim_pid == 0) {
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    PhysicsSimBridge bridge("127.0.0.1", port, 20000, 24, nullptr);
    if (!bridge.init()) {
        std::cerr << "Failed to initialize PhysicsSimBridge\n";
        ::kill(sim_pid, SIGTERM);
        ::waitpid(sim_pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const int stand_steps = stand_ms / 20;
    const int tripod_steps = tripod_ms / 20;
    HoldMetrics stand_metrics{};
    for (int i = 0; i < stand_steps && !g_exit.load(); ++i) {
        if (!stepPose(bridge, stand_targets, stand_metrics)) {
            std::cerr << "Failed while holding six-leg stand\n";
            ::kill(sim_pid, SIGTERM);
            ::waitpid(sim_pid, nullptr, 0);
            return EXIT_FAILURE;
        }
    }
    printMetrics("six_leg_stand", stand_metrics);

    HoldMetrics tripod_metrics{};
    if (indefinite) {
        std::cout << "Holding tripod pose until Ctrl-C...\n";
        while (!g_exit.load()) {
            if (!stepPose(bridge, tripod_targets, tripod_metrics)) {
                std::cerr << "Failed while holding tripod support pose\n";
                ::kill(sim_pid, SIGTERM);
                ::waitpid(sim_pid, nullptr, 0);
                return EXIT_FAILURE;
            }
        }
    } else {
        for (int i = 0; i < tripod_steps && !g_exit.load(); ++i) {
            if (!stepPose(bridge, tripod_targets, tripod_metrics)) {
                std::cerr << "Failed while holding tripod support pose\n";
                ::kill(sim_pid, SIGTERM);
                ::waitpid(sim_pid, nullptr, 0);
                return EXIT_FAILURE;
            }
        }
    }
    printMetrics("tripod_support", tripod_metrics);

    ::kill(sim_pid, SIGTERM);
    ::waitpid(sim_pid, nullptr, 0);
    return EXIT_SUCCESS;
#endif
}
