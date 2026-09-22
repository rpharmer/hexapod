#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
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
#include <limits>
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
    double max_body_height_m{-1e9};
    double max_abs_roll_rad{0.0};
    double max_abs_pitch_rad{0.0};
    double max_support_foot_world_drift_m{0.0};
    double max_support_joint_drift_rad{0.0};
    double max_support_commanded_tracking_error_m{0.0};
    double max_support_commanded_joint_error_rad{0.0};
    double max_support_commanded_tracking_error_first_120ms_m{0.0};
    double max_support_commanded_tracking_error_after_120ms_m{0.0};
    int max_tracking_leg{-1};
    std::array<double, kNumLegs> max_tracking_error_m{};
    std::array<double, kNumLegs> max_abs_dx_m{};
    std::array<double, kNumLegs> max_abs_dy_m{};
    std::array<double, kNumLegs> max_abs_dz_m{};
    std::array<double, kNumLegs * 3> max_abs_commanded_joint_error_rad{};
    std::array<double, kNumLegs * 3> mean_abs_commanded_joint_error_rad{};
    std::array<double, kNumLegs * 3> terminal_commanded_joint_error_rad{};
    double max_peak_servo_torque_utilization{0.0};
    double mean_peak_servo_torque_utilization{0.0};
    double peak_actuator_impulse_ns{0.0};
    double max_base_speed_norm{0.0};
    double max_joint_speed_radps{0.0};
    double mean_joint_speed_radps{0.0};
    std::array<std::uint64_t, 7> contact_count_histogram{};
    RobotState last_state{};
};

struct TripodCommand {
    JointTargets joints{};
    LegTargets pre_ik_feet{};
};

constexpr double kTwoPi = 6.28318530717958647692;

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

TripodCommand buildTripodRaisedCommand() {
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

    // Internal order is R3, L3, R2, L2, R1, L1. Raise one valid alternating
    // tripod rather than the odd indices, which are all left-side legs.
    constexpr std::array<int, 3> kRaisedLegs{{0, 3, 4}};
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

    TripodCommand command{};
    command.pre_ik_feet = foot_targets;
    command.joints = ik.solve(est, foot_targets, safety);
    return command;
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

BodyPose makeBodyPose(const RobotState& state) {
    BodyPose pose{};
    pose.position = state.body_twist_state.body_trans_m;
    pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};
    return pose;
}

HoldMetrics holdPose(PhysicsSimBridge& bridge,
                     const JointTargets& targets,
                     const std::array<bool, kNumLegs>& support_legs,
                     int steps,
                     int transient_steps) {
    HoldMetrics metrics{};
    int samples = 0;
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    std::array<bool, kNumLegs> have_reference{};
    std::array<Vec3, kNumLegs> reference_world{};
    std::array<LegState, kNumLegs> reference_joints{};
    std::array<std::uint64_t, kNumLegs * 3> joint_error_samples{};
    std::uint64_t solver_telemetry_samples = 0;
    std::uint64_t joint_speed_samples = 0;
    const int census_transient_steps = std::max(1, transient_steps);
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
        metrics.max_body_height_m = std::max(metrics.max_body_height_m, z);
        metrics.max_abs_roll_rad = std::max(metrics.max_abs_roll_rad, std::abs(state.body_twist_state.twist_pos_rad.x));
        metrics.max_abs_pitch_rad =
            std::max(metrics.max_abs_pitch_rad, std::abs(state.body_twist_state.twist_pos_rad.y));
        const std::size_t contact_count = static_cast<std::size_t>(
            std::count(state.foot_contacts.begin(), state.foot_contacts.end(), true));
        ++metrics.contact_count_histogram[std::min<std::size_t>(6, contact_count)];
        metrics.max_base_speed_norm = std::max(
            metrics.max_base_speed_norm,
            std::hypot(
                vecNorm(state.body_twist_state.body_trans_mps.raw()),
                vecNorm(state.body_twist_state.twist_vel_radps.raw())));
        const BodyPose body_pose = makeBodyPose(state);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t leg_index = static_cast<std::size_t>(leg);
            if (!support_legs[leg_index]) {
                continue;
            }
            const FootTarget measured_body = fk.footInBodyFrame(state.leg_states[leg_index], geometry.legGeometry[leg_index]);
            const FootTarget measured_world = fk.footInWorldFrame(state.leg_states[leg_index], body_pose, geometry.legGeometry[leg_index]);
            const FootTarget commanded_body = fk.footInBodyFrame(targets.leg_states[leg_index], geometry.legGeometry[leg_index]);
            if (!have_reference[leg_index]) {
                have_reference[leg_index] = true;
                reference_world[leg_index] = measured_world.pos_body_m.raw();
                reference_joints[leg_index] = state.leg_states[leg_index];
            }
            metrics.max_support_foot_world_drift_m = std::max(
                metrics.max_support_foot_world_drift_m,
                vecNorm(measured_world.pos_body_m.raw() - reference_world[leg_index]));
            const Vec3 tracking_delta = commanded_body.pos_body_m.raw() - measured_body.pos_body_m.raw();
            const double tracking_error_m = vecNorm(tracking_delta);
            if (tracking_error_m > metrics.max_support_commanded_tracking_error_m) {
                metrics.max_tracking_leg = leg;
            }
            metrics.max_support_commanded_tracking_error_m =
                std::max(metrics.max_support_commanded_tracking_error_m, tracking_error_m);
            metrics.max_tracking_error_m[leg_index] =
                std::max(metrics.max_tracking_error_m[leg_index], tracking_error_m);
            metrics.max_abs_dx_m[leg_index] = std::max(metrics.max_abs_dx_m[leg_index], std::abs(tracking_delta.x));
            metrics.max_abs_dy_m[leg_index] = std::max(metrics.max_abs_dy_m[leg_index], std::abs(tracking_delta.y));
            metrics.max_abs_dz_m[leg_index] = std::max(metrics.max_abs_dz_m[leg_index], std::abs(tracking_delta.z));
            if (i < census_transient_steps) {
                metrics.max_support_commanded_tracking_error_first_120ms_m =
                    std::max(metrics.max_support_commanded_tracking_error_first_120ms_m, tracking_error_m);
            } else {
                metrics.max_support_commanded_tracking_error_after_120ms_m =
                    std::max(metrics.max_support_commanded_tracking_error_after_120ms_m, tracking_error_m);
            }
            for (int joint = 0; joint < 3; ++joint) {
                const std::size_t wire_index = leg_index * 3 + static_cast<std::size_t>(joint);
                const double joint_speed = std::abs(
                    state.leg_states[leg_index].joint_state[joint].vel_radps.value);
                metrics.max_joint_speed_radps = std::max(metrics.max_joint_speed_radps, joint_speed);
                metrics.mean_joint_speed_radps += joint_speed;
                ++joint_speed_samples;
                metrics.max_support_joint_drift_rad = std::max(
                    metrics.max_support_joint_drift_rad,
                    std::abs(state.leg_states[leg_index].joint_state[joint].pos_rad.value -
                             reference_joints[leg_index].joint_state[joint].pos_rad.value));
                const double joint_error_rad = std::remainder(
                    targets.leg_states[leg_index].joint_state[joint].pos_rad.value -
                        state.leg_states[leg_index].joint_state[joint].pos_rad.value,
                    kTwoPi);
                metrics.max_support_commanded_joint_error_rad =
                    std::max(metrics.max_support_commanded_joint_error_rad, std::abs(joint_error_rad));
                metrics.max_abs_commanded_joint_error_rad[wire_index] = std::max(
                    metrics.max_abs_commanded_joint_error_rad[wire_index], std::abs(joint_error_rad));
                metrics.mean_abs_commanded_joint_error_rad[wire_index] += std::abs(joint_error_rad);
                metrics.terminal_commanded_joint_error_rad[wire_index] = joint_error_rad;
                ++joint_error_samples[wire_index];
            }
        }
        if (const auto telemetry = bridge.latestSolverTelemetry(); telemetry.has_value()) {
            metrics.max_peak_servo_torque_utilization = std::max(
                metrics.max_peak_servo_torque_utilization,
                static_cast<double>(telemetry->peak_servo_torque_utilization));
            metrics.mean_peak_servo_torque_utilization += telemetry->peak_servo_torque_utilization;
            metrics.peak_actuator_impulse_ns = std::max(
                metrics.peak_actuator_impulse_ns,
                static_cast<double>(telemetry->peak_actuator_impulse));
            ++solver_telemetry_samples;
        }
        metrics.last_state = state;
        ++samples;
    }

    if (samples > 0) {
        metrics.mean_body_height_m /= static_cast<double>(samples);
    } else {
        metrics.min_body_height_m = 0.0;
        metrics.max_body_height_m = 0.0;
    }
    for (std::size_t i = 0; i < joint_error_samples.size(); ++i) {
        if (joint_error_samples[i] > 0) {
            metrics.mean_abs_commanded_joint_error_rad[i] /=
                static_cast<double>(joint_error_samples[i]);
        }
    }
    if (solver_telemetry_samples > 0) {
        metrics.mean_peak_servo_torque_utilization /=
            static_cast<double>(solver_telemetry_samples);
    }
    if (joint_speed_samples > 0) {
        metrics.mean_joint_speed_radps /= static_cast<double>(joint_speed_samples);
    }
    return metrics;
}

template <typename T, std::size_t N>
void appendJsonArray(std::ostringstream& o, const std::array<T, N>& values) {
    o << '[';
    for (std::size_t i = 0; i < N; ++i) {
        if (i > 0) {
            o << ',';
        }
        o << values[i];
    }
    o << ']';
}

void printMetrics(const std::string& label, const HoldMetrics& metrics) {
    std::cout << label
              << " mean_height_m=" << metrics.mean_body_height_m
              << " min_height_m=" << metrics.min_body_height_m
              << " max_height_m=" << metrics.max_body_height_m
              << " max_roll_rad=" << metrics.max_abs_roll_rad
              << " max_pitch_rad=" << metrics.max_abs_pitch_rad
              << " max_support_foot_world_drift_m=" << metrics.max_support_foot_world_drift_m
              << " max_support_joint_drift_rad=" << metrics.max_support_joint_drift_rad
              << " max_support_commanded_tracking_error_m=" << metrics.max_support_commanded_tracking_error_m
              << " max_support_commanded_joint_error_rad=" << metrics.max_support_commanded_joint_error_rad
              << " max_tracking_leg=" << metrics.max_tracking_leg
              << " tracking_first_120ms_m=" << metrics.max_support_commanded_tracking_error_first_120ms_m
              << " tracking_after_120ms_m=" << metrics.max_support_commanded_tracking_error_after_120ms_m
              << " max_peak_servo_torque_utilization=" << metrics.max_peak_servo_torque_utilization
              << " mean_peak_servo_torque_utilization=" << metrics.mean_peak_servo_torque_utilization
              << " peak_actuator_impulse_ns=" << metrics.peak_actuator_impulse_ns
              << " max_base_speed_norm=" << metrics.max_base_speed_norm
              << " max_joint_speed_radps=" << metrics.max_joint_speed_radps
              << " mean_joint_speed_radps=" << metrics.mean_joint_speed_radps
              << '\n';
    std::cout << label << "_per_leg";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t leg_index = static_cast<std::size_t>(leg);
        std::cout << " L" << leg
                  << "{err=" << metrics.max_tracking_error_m[leg_index]
                  << ",dx=" << metrics.max_abs_dx_m[leg_index]
                  << ",dy=" << metrics.max_abs_dy_m[leg_index]
                  << ",dz=" << metrics.max_abs_dz_m[leg_index] << '}';
    }
    std::cout << '\n';
    std::cout << label << "_contact_histogram";
    for (std::size_t n = 0; n < metrics.contact_count_histogram.size(); ++n) {
        std::cout << " n" << n << '=' << metrics.contact_count_histogram[n];
    }
    std::cout << '\n';
    std::cout << label << "_joint_errors";
    constexpr std::array<const char*, 3> kJointNames{{"coxa", "femur", "tibia"}};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            const std::size_t wire_index = static_cast<std::size_t>(leg * 3 + joint);
            std::cout << " L" << leg << '.' << kJointNames[static_cast<std::size_t>(joint)]
                      << "{max=" << metrics.max_abs_commanded_joint_error_rad[wire_index]
                      << ",mean=" << metrics.mean_abs_commanded_joint_error_rad[wire_index]
                      << ",terminal_signed=" << metrics.terminal_commanded_joint_error_rad[wire_index]
                      << '}';
        }
    }
    std::cout << '\n';
}

std::string tripodSupportBaselineLimitsJson(const double stand_mean_min,
                                            const double tripod_mean_min,
                                            const double tripod_min_min,
                                            const double sag_max,
                                            const double roll_max,
                                            const double pitch_max,
                                            const double stand_body_height_creep_max,
                                            const double tripod_body_height_creep_max,
                                            const double stand_support_foot_drift_max,
                                            const double tripod_support_foot_drift_max,
                                            const double tripod_joint_drift_max,
                                            const double tripod_tracking_error_max) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"stand_mean_body_height_m_min\":" << stand_mean_min
      << ",\"tripod_mean_body_height_m_min\":" << tripod_mean_min
      << ",\"tripod_min_body_height_m_min\":" << tripod_min_min << ",\"sag_vs_stand_m_max\":" << sag_max
      << ",\"max_abs_roll_rad\":" << roll_max << ",\"max_abs_pitch_rad\":" << pitch_max
      << ",\"stand_body_height_creep_m_max\":" << stand_body_height_creep_max
      << ",\"tripod_body_height_creep_m_max\":" << tripod_body_height_creep_max
      << ",\"stand_support_foot_world_drift_m_max\":" << stand_support_foot_drift_max
      << ",\"tripod_support_foot_world_drift_m_max\":" << tripod_support_foot_drift_max
      << ",\"tripod_support_joint_drift_rad_max\":" << tripod_joint_drift_max
      << ",\"tripod_support_commanded_tracking_error_m_max\":" << tripod_tracking_error_max << '}';
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
    const double kStandBodyHeightCreepMax =
        test_limits::getDouble(kSuite, kCase, "", "stand_body_height_creep_m_max", 0.005);
    const double kTripodBodyHeightCreepMax =
        test_limits::getDouble(kSuite, kCase, "", "tripod_body_height_creep_m_max", 0.01);
    const double kStandSupportFootWorldDriftMax =
        test_limits::getDouble(kSuite, kCase, "", "stand_support_foot_world_drift_m_max", 0.01);
    const double kTripodSupportFootWorldDriftMax =
        test_limits::getDouble(kSuite, kCase, "", "tripod_support_foot_world_drift_m_max", 0.13);
    const double kTripodSupportJointDriftMax =
        test_limits::getDouble(kSuite, kCase, "", "tripod_support_joint_drift_rad_max", 0.14);
    const double kTripodSupportTrackingErrorMax =
        test_limits::getDouble(kSuite, kCase, "", "tripod_support_commanded_tracking_error_m_max", 0.045);
    const auto limitsJson = [&]() {
        return tripodSupportBaselineLimitsJson(
            kStandMeanMin,
            kTripodMeanMin,
            kTripodMinMin,
            kSagVsStandMax,
            kMaxAbsRoll,
            kMaxAbsPitch,
            kStandBodyHeightCreepMax,
            kTripodBodyHeightCreepMax,
            kStandSupportFootWorldDriftMax,
            kTripodSupportFootWorldDriftMax,
            kTripodSupportJointDriftMax,
            kTripodSupportTrackingErrorMax);
    };

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 24000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = harness.bus_loop_period_us;
    PhysicsSimSolverSettings solver_settings =
        physics_sim_test_utils::productionProximalSolverSettings();
    if (const char* mode = std::getenv("HEXAPOD_WALK_TEST_SOLVER_MODE")) {
        if (std::string(mode) == "legacy-pgs") {
            solver_settings.mode = physics_sim::PhysicsSolverMode::LegacyPgs;
            solver_settings.iterations = harness.physics_solver_iterations;
        } else if (std::string(mode) == "pinocchio-compliant") {
            solver_settings.mode = physics_sim::PhysicsSolverMode::PinocchioProximalCompliant;
        } else if (std::string(mode) != "pinocchio-proximal") {
            std::cerr << "invalid HEXAPOD_WALK_TEST_SOLVER_MODE=" << mode << '\n';
            return 2;
        }
    }

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    PhysicsSimBridge bridge("127.0.0.1", port, bus_loop_period_us, solver_settings, nullptr);
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
    const TripodCommand tripod_command = buildTripodRaisedCommand();
    const JointTargets& tripod_targets = tripod_command.joints;
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
    constexpr std::array<bool, kNumLegs> kStandSupportLegs{{true, true, true, true, true, true}};
    constexpr std::array<bool, kNumLegs> kTripodSupportLegs{{false, true, true, false, false, true}};
    const int kTransientSteps = std::max(
        1, static_cast<int>(std::llround(0.120 / std::max(1.0e-6, static_cast<double>(bus_loop_period_us) * 1.0e-6))));

    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    double max_support_command_residual_m = 0.0;
    int max_command_residual_leg = -1;
    std::array<double, kNumLegs> support_command_residual_m{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t leg_index = static_cast<std::size_t>(leg);
        if (!kTripodSupportLegs[leg_index]) {
            continue;
        }
        const FootTarget post_ik = fk.footInBodyFrame(
            tripod_targets.leg_states[leg_index], geometry.legGeometry[leg_index]);
        const double residual_m = vecNorm(
            tripod_command.pre_ik_feet.feet[leg_index].pos_body_m.raw() - post_ik.pos_body_m.raw());
        support_command_residual_m[leg_index] = residual_m;
        if (residual_m > max_support_command_residual_m) {
            max_support_command_residual_m = residual_m;
            max_command_residual_leg = leg;
        }
    }

    double max_support_joint_command_delta_rad = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t leg_index = static_cast<std::size_t>(leg);
        if (!kTripodSupportLegs[leg_index]) {
            continue;
        }
        for (int joint = 0; joint < 3; ++joint) {
            const double delta = std::remainder(
                tripod_targets.leg_states[leg_index].joint_state[joint].pos_rad.value -
                    stand_targets.leg_states[leg_index].joint_state[joint].pos_rad.value,
                kTwoPi);
            max_support_joint_command_delta_rad =
                std::max(max_support_joint_command_delta_rad, std::abs(delta));
        }
    }

    (void)holdPose(bridge, stand_targets, kStandSupportLegs, kStandWarmupSteps, kTransientSteps);
    const HoldMetrics stand_metrics =
        holdPose(bridge, stand_targets, kStandSupportLegs, kMetricsSteps, kTransientSteps);

    (void)holdPose(bridge, tripod_targets, kTripodSupportLegs, kTripodWarmupSteps, kTransientSteps);
    const HoldMetrics tripod_metrics =
        holdPose(bridge, tripod_targets, kTripodSupportLegs, kMetricsSteps, kTransientSteps);

    std::cout << "solver_mode="
              << (solver_settings.mode == physics_sim::PhysicsSolverMode::LegacyPgs
                      ? "legacy-pgs"
                      : "pinocchio-proximal")
              << " solver_iterations=" << solver_settings.iterations << '\n';
    printMetrics("six_leg_stand", stand_metrics);
    printMetrics("tripod_support", tripod_metrics);
    std::cout << "tripod_command_residual_m=" << max_support_command_residual_m
              << " max_command_residual_leg=" << max_command_residual_leg
              << " max_support_joint_command_delta_rad=" << max_support_joint_command_delta_rad
              << " per_leg";
    for (int leg = 0; leg < kNumLegs; ++leg) {
        std::cout << " L" << leg << '=' << support_command_residual_m[static_cast<std::size_t>(leg)];
    }
    std::cout << '\n';

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    const double sag_vs_stand_m = stand_metrics.mean_body_height_m - tripod_metrics.mean_body_height_m;
    const double stand_body_height_creep_m = stand_metrics.max_body_height_m - stand_metrics.min_body_height_m;
    const double tripod_body_height_creep_m = tripod_metrics.max_body_height_m - tripod_metrics.min_body_height_m;

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
                           "tripod support should keep pitch within a moderate bound") &&
                    expect(stand_body_height_creep_m < kStandBodyHeightCreepMax,
                           "six-leg stand should not creep noticeably in body height") &&
                    expect(tripod_body_height_creep_m < kTripodBodyHeightCreepMax,
                           "tripod support should hold body height without large creep") &&
                    expect(stand_metrics.max_support_foot_world_drift_m < kStandSupportFootWorldDriftMax,
                           "six-leg stand should not drift its support feet") &&
                    expect(tripod_metrics.max_support_foot_world_drift_m < kTripodSupportFootWorldDriftMax,
                           "tripod support should keep support feet near their initial world anchors") &&
                    expect(tripod_metrics.max_support_joint_drift_rad < kTripodSupportJointDriftMax,
                           "tripod support should not accumulate large loaded joint drift") &&
                    expect(tripod_metrics.max_support_commanded_tracking_error_m < kTripodSupportTrackingErrorMax,
                           "tripod support should keep commanded-vs-measured support foot error bounded");
    if (emit_metrics_json) {
        std::ostringstream metrics;
        metrics << std::setprecision(17)
                << "{\"six_leg_stand_mean_body_height_m\":" << stand_metrics.mean_body_height_m
                << ",\"six_leg_stand_min_body_height_m\":" << stand_metrics.min_body_height_m
                << ",\"six_leg_stand_max_body_height_m\":" << stand_metrics.max_body_height_m
                << ",\"six_leg_stand_max_abs_roll_rad\":" << stand_metrics.max_abs_roll_rad
                << ",\"six_leg_stand_max_abs_pitch_rad\":" << stand_metrics.max_abs_pitch_rad
                << ",\"six_leg_stand_body_height_creep_m\":" << stand_body_height_creep_m
                << ",\"six_leg_stand_max_support_foot_world_drift_m\":" << stand_metrics.max_support_foot_world_drift_m
                << ",\"tripod_mean_body_height_m\":" << tripod_metrics.mean_body_height_m
                << ",\"tripod_min_body_height_m\":" << tripod_metrics.min_body_height_m
                << ",\"tripod_max_body_height_m\":" << tripod_metrics.max_body_height_m
                << ",\"tripod_max_abs_roll_rad\":" << tripod_metrics.max_abs_roll_rad
                << ",\"tripod_max_abs_pitch_rad\":" << tripod_metrics.max_abs_pitch_rad
                << ",\"tripod_body_height_creep_m\":" << tripod_body_height_creep_m
                << ",\"tripod_max_support_foot_world_drift_m\":" << tripod_metrics.max_support_foot_world_drift_m
                << ",\"tripod_max_support_joint_drift_rad\":" << tripod_metrics.max_support_joint_drift_rad
                << ",\"tripod_max_support_commanded_tracking_error_m\":" << tripod_metrics.max_support_commanded_tracking_error_m
                << ",\"six_leg_stand_max_support_commanded_tracking_error_m\":"
                << stand_metrics.max_support_commanded_tracking_error_m
                << ",\"tripod_max_support_commanded_joint_error_rad\":"
                << tripod_metrics.max_support_commanded_joint_error_rad
                << ",\"six_leg_stand_max_peak_servo_torque_utilization\":"
                << stand_metrics.max_peak_servo_torque_utilization
                << ",\"six_leg_stand_mean_peak_servo_torque_utilization\":"
                << stand_metrics.mean_peak_servo_torque_utilization
                << ",\"tripod_max_peak_servo_torque_utilization\":"
                << tripod_metrics.max_peak_servo_torque_utilization
                << ",\"tripod_mean_peak_servo_torque_utilization\":"
                << tripod_metrics.mean_peak_servo_torque_utilization
                << ",\"tripod_peak_actuator_impulse_ns\":"
                << tripod_metrics.peak_actuator_impulse_ns
                << ",\"tripod_max_base_speed_norm\":"
                << tripod_metrics.max_base_speed_norm
                << ",\"tripod_max_joint_speed_radps\":"
                << tripod_metrics.max_joint_speed_radps
                << ",\"tripod_mean_joint_speed_radps\":"
                << tripod_metrics.mean_joint_speed_radps
                << ",\"tripod_max_tracking_leg\":" << tripod_metrics.max_tracking_leg
                << ",\"tripod_tracking_first_120ms_m\":"
                << tripod_metrics.max_support_commanded_tracking_error_first_120ms_m
                << ",\"tripod_tracking_after_120ms_m\":"
                << tripod_metrics.max_support_commanded_tracking_error_after_120ms_m
                << ",\"tripod_max_support_command_residual_m\":" << max_support_command_residual_m
                << ",\"tripod_max_command_residual_leg\":" << max_command_residual_leg
                << ",\"tripod_max_support_joint_command_delta_rad\":" << max_support_joint_command_delta_rad
                << ",\"sag_vs_stand_m\":" << sag_vs_stand_m
                << ",\"tripod_per_leg_tracking_error_m\":";
        appendJsonArray(metrics, tripod_metrics.max_tracking_error_m);
        metrics << ",\"tripod_per_leg_abs_dx_m\":";
        appendJsonArray(metrics, tripod_metrics.max_abs_dx_m);
        metrics << ",\"tripod_per_leg_abs_dy_m\":";
        appendJsonArray(metrics, tripod_metrics.max_abs_dy_m);
        metrics << ",\"tripod_per_leg_abs_dz_m\":";
        appendJsonArray(metrics, tripod_metrics.max_abs_dz_m);
        metrics << ",\"tripod_per_leg_command_residual_m\":";
        appendJsonArray(metrics, support_command_residual_m);
        metrics << ",\"six_leg_stand_per_joint_max_abs_error_rad\":";
        appendJsonArray(metrics, stand_metrics.max_abs_commanded_joint_error_rad);
        metrics << ",\"six_leg_stand_per_joint_mean_abs_error_rad\":";
        appendJsonArray(metrics, stand_metrics.mean_abs_commanded_joint_error_rad);
        metrics << ",\"tripod_per_joint_max_abs_error_rad\":";
        appendJsonArray(metrics, tripod_metrics.max_abs_commanded_joint_error_rad);
        metrics << ",\"tripod_per_joint_mean_abs_error_rad\":";
        appendJsonArray(metrics, tripod_metrics.mean_abs_commanded_joint_error_rad);
        metrics << ",\"tripod_per_joint_terminal_signed_error_rad\":";
        appendJsonArray(metrics, tripod_metrics.terminal_commanded_joint_error_rad);
        metrics << ",\"six_leg_stand_contact_count_histogram\":";
        appendJsonArray(metrics, stand_metrics.contact_count_histogram);
        metrics << ",\"tripod_contact_count_histogram\":";
        appendJsonArray(metrics, tripod_metrics.contact_count_histogram);
        metrics << '}';
        physics_sim_metrics::emitLine("physics_sim_tripod_support_baseline", "tripod_support_baseline", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
