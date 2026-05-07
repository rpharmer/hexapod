#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_ik.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_test_utils.hpp"
#include "test_limits_manifest.hpp"
#include "physics_sim_bridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
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

std::string tripodSupportBaselineLimitsJson(const double stand_mean_min,
                                            const double tripod_mean_min,
                                            const double tripod_min_min,
                                            const double sag_max,
                                            const double roll_max,
                                            const double pitch_max) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"stand_mean_body_height_m_min\":" << stand_mean_min
      << ",\"tripod_mean_body_height_m_min\":" << tripod_mean_min
      << ",\"tripod_min_body_height_m_min\":" << tripod_min_min << ",\"sag_vs_stand_m_max\":" << sag_max
      << ",\"max_abs_roll_rad\":" << roll_max << ",\"max_abs_pitch_rad\":" << pitch_max << '}';
    return o.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_tripod_support_baseline (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_tripod_support_baseline (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    constexpr const char* kSuite = "physics_sim_tripod_support_baseline";
    constexpr const char* kCase = "tripod_support_baseline";
    const double kStandMeanMin = test_limits::getDouble(kSuite, kCase, "", "stand_mean_body_height_m_min", 0.10);
    const double kTripodMeanMin = test_limits::getDouble(kSuite, kCase, "", "tripod_mean_body_height_m_min", 0.075);
    const double kTripodMinMin = test_limits::getDouble(kSuite, kCase, "", "tripod_min_body_height_m_min", 0.06);
    const double kSagVsStandMax = test_limits::getDouble(kSuite, kCase, "", "sag_vs_stand_m_max", 0.05);
    const double kMaxAbsRoll = test_limits::getDouble(kSuite, kCase, "", "max_abs_roll_rad", 0.45);
    const double kMaxAbsPitch = test_limits::getDouble(kSuite, kCase, "", "max_abs_pitch_rad", 0.45);
    const auto limitsJson = [&]() {
        return tripodSupportBaselineLimitsJson(
            kStandMeanMin, kTripodMeanMin, kTripodMinMin, kSagVsStandMax, kMaxAbsRoll, kMaxAbsPitch);
    };

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 24000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = harness.bus_loop_period_us;

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

    PhysicsSimBridge bridge(
        "127.0.0.1", port, bus_loop_period_us, harness.physics_solver_iterations, nullptr);
    if (!expect(bridge.init(), "physics sim bridge should initialize")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_tripod_support_baseline", "tripod_support_baseline", false,
                                          limitsJson(), "{\"stage\":\"bridge_init_failed\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const JointTargets stand_targets = buildStandTargets();
    const JointTargets tripod_targets = buildTripodRaisedTargets();
    if (!expect(finiteJointTargets(stand_targets), "standing joint targets should be finite") ||
        !expect(finiteJointTargets(tripod_targets), "tripod-raised joint targets should be finite")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_tripod_support_baseline", "tripod_support_baseline", false,
                                          limitsJson(), "{\"stage\":\"invalid_joint_targets\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(160, bus_loop_period_us));
    const int kTripodWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(260, bus_loop_period_us));
    const int kMetricsSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(180, bus_loop_period_us));

    (void)holdPose(bridge, stand_targets, kStandWarmupSteps);
    const HoldMetrics stand_metrics = holdPose(bridge, stand_targets, kMetricsSteps);

    (void)holdPose(bridge, tripod_targets, kTripodWarmupSteps);
    const HoldMetrics tripod_metrics = holdPose(bridge, tripod_targets, kMetricsSteps);

    printMetrics("six_leg_stand", stand_metrics);
    printMetrics("tripod_support", tripod_metrics);

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    const double sag_vs_stand_m = stand_metrics.mean_body_height_m - tripod_metrics.mean_body_height_m;

    const bool ok = expect(stand_metrics.mean_body_height_m > kStandMeanMin,
                           "six-leg baseline should settle near nominal body height") &&
                    expect(tripod_metrics.mean_body_height_m > kTripodMeanMin,
                           "tripod support should keep the body well above a collapse height") &&
                    expect(tripod_metrics.min_body_height_m > kTripodMinMin,
                           "tripod support should not let the body crash near the ground") &&
                    expect(sag_vs_stand_m < kSagVsStandMax,
                           "tripod support should sag noticeably less than 5 cm versus six-leg stand") &&
                    expect(tripod_metrics.max_abs_roll_rad < kMaxAbsRoll,
                           "tripod support should keep roll within a moderate bound") &&
                    expect(tripod_metrics.max_abs_pitch_rad < kMaxAbsPitch,
                           "tripod support should keep pitch within a moderate bound");
    if (emit_metrics_json) {
        std::ostringstream metrics;
        metrics << std::setprecision(17)
                << "{\"six_leg_stand_mean_body_height_m\":" << stand_metrics.mean_body_height_m
                << ",\"six_leg_stand_min_body_height_m\":" << stand_metrics.min_body_height_m
                << ",\"six_leg_stand_max_abs_roll_rad\":" << stand_metrics.max_abs_roll_rad
                << ",\"six_leg_stand_max_abs_pitch_rad\":" << stand_metrics.max_abs_pitch_rad
                << ",\"tripod_mean_body_height_m\":" << tripod_metrics.mean_body_height_m
                << ",\"tripod_min_body_height_m\":" << tripod_metrics.min_body_height_m
                << ",\"tripod_max_abs_roll_rad\":" << tripod_metrics.max_abs_roll_rad
                << ",\"tripod_max_abs_pitch_rad\":" << tripod_metrics.max_abs_pitch_rad
                << ",\"sag_vs_stand_m\":" << sag_vs_stand_m << '}';
        physics_sim_metrics::emitLine("physics_sim_tripod_support_baseline", "tripod_support_baseline", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
