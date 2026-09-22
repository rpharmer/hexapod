#include "body_controller.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "test_limits_manifest.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
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

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              PhysicsSimSolverSettings solver_settings)
        : inner_(std::move(host), port, bus_loop_period_us, solver_settings, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        applied_targets_ = last_targets_.value_or(JointTargets{});
        const bool ok = inner_.read(out);
        last_solver_telemetry_ = inner_.latestSolverTelemetry();
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override {
        last_targets_ = in;
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const { return last_state_; }
    const std::optional<JointTargets>& last_targets() const { return last_targets_; }
    const JointTargets& applied_targets() const { return applied_targets_; }
    const std::optional<PhysicsSimSolverTelemetry>& last_solver_telemetry() const {
        return last_solver_telemetry_;
    }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
    std::optional<JointTargets> last_targets_{};
    JointTargets applied_targets_{};
    std::optional<PhysicsSimSolverTelemetry> last_solver_telemetry_{};
};

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

double minFootTipWorldZ(const RobotState& state) {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    BodyPose body_pose{};
    body_pose.position = state.body_twist_state.body_trans_m;
    body_pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    body_pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    body_pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};

    double min_z = std::numeric_limits<double>::infinity();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const FootTarget foot_world =
            fk.footInWorldFrame(state.leg_states[static_cast<std::size_t>(leg)], body_pose, geometry.legGeometry[leg]);
        min_z = std::min(min_z, foot_world.pos_body_m.z);
    }
    return min_z;
}

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kStationaryJointTargetDeltaRad = 1.0e-5;
constexpr double kStaticReducedSupportLegTargetDeltaRad = 0.10;
constexpr double kStaticReducedSupportDwellS = 0.250;

double maxJointTargetDeltaRad(const JointTargets& a, const JointTargets& b) {
    double max_delta = 0.0;
    for (std::size_t leg = 0; leg < a.leg_states.size(); ++leg) {
        for (std::size_t joint = 0; joint < a.leg_states[leg].joint_state.size(); ++joint) {
            const double delta = std::remainder(
                a.leg_states[leg].joint_state[joint].pos_rad.value -
                    b.leg_states[leg].joint_state[joint].pos_rad.value,
                kTwoPi);
            max_delta = std::max(max_delta, std::abs(delta));
        }
    }
    return max_delta;
}

int changedLegCountVsReference(const JointTargets& current, const JointTargets& reference) {
    int changed = 0;
    for (std::size_t leg = 0; leg < current.leg_states.size(); ++leg) {
        double max_leg_delta = 0.0;
        for (std::size_t joint = 0; joint < current.leg_states[leg].joint_state.size(); ++joint) {
            const double delta = std::remainder(
                current.leg_states[leg].joint_state[joint].pos_rad.value -
                    reference.leg_states[leg].joint_state[joint].pos_rad.value,
                kTwoPi);
            max_leg_delta = std::max(max_leg_delta, std::abs(delta));
        }
        if (max_leg_delta > kStaticReducedSupportLegTargetDeltaRad) {
            ++changed;
        }
    }
    return changed;
}

int rawFootContactCount(const RobotState& state) {
    int count = 0;
    for (const bool contact : state.foot_contacts) {
        if (contact) {
            ++count;
        }
    }
    return count;
}

struct MinHeightSnapshot {
    int step{-1};
    double body_z_m{std::numeric_limits<double>::quiet_NaN()};
    double body_vz_mps{std::numeric_limits<double>::quiet_NaN()};
    double roll_rad{std::numeric_limits<double>::quiet_NaN()};
    double pitch_rad{std::numeric_limits<double>::quiet_NaN()};
    int raw_contacts{0};
    int planned_supports{0};
    int fused_supports{0};
    int gait_stance_legs{0};
    double gait_phase{std::numeric_limits<double>::quiet_NaN()};
    double duty_factor{std::numeric_limits<double>::quiet_NaN()};
    double peak_servo_torque_utilization{std::numeric_limits<double>::quiet_NaN()};
    double peak_actuator_impulse{std::numeric_limits<double>::quiet_NaN()};
    double peak_normal_impulse{std::numeric_limits<double>::quiet_NaN()};
    double max_joint_error_rad{0.0};
    int max_joint_error_wire{-1};
    std::array<double, kNumJoints> joint_error_rad{};
    std::array<double, kNumJoints> joint_velocity_radps{};
    std::array<double, kNumJoints> joint_target_velocity_radps{};
    std::array<double, kNumLegs> tracking_error_m{};
    std::array<double, kNumLegs> commanded_body_z_m{};
    std::array<double, kNumLegs> measured_world_z_m{};
    std::array<bool, kNumLegs> planned_stance{};
    std::array<bool, kNumLegs> fused_support{};
    std::array<bool, kNumLegs> raw_contact{};
};

std::string slowFwdWalkFootClearanceLimitsJson(const double min_foot_tip_world_z_m,
                                                const bool require_saw_raw_contact_loss,
                                                const double max_body_undershoot_m) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
      << ",\"require_saw_raw_contact_loss\":" << (require_saw_raw_contact_loss ? "true" : "false")
      << ",\"max_body_undershoot_m\":" << max_body_undershoot_m << '}';
    return o.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_slow_fwd_walk_foot_clearance (Linux-only)\n";
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
        std::cout << "skip test_physics_sim_slow_fwd_walk_foot_clearance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    constexpr const char* kSuite = "physics_sim_slow_fwd_walk_foot_clearance";
    constexpr const char* kCase = "slow_fwd_walk_foot_clearance";
    const double kMinFootTipWorldZ =
        test_limits::getDouble(kSuite, kCase, "", "min_foot_tip_world_z_m", -1.0e-4);
    const bool kRequireSawRawContactLoss =
        test_limits::getBool(kSuite, kCase, "", "require_saw_raw_contact_loss", true);
    const double kMaxBodyUndershootM =
        test_limits::getDouble(kSuite, kCase, "", "max_body_undershoot_m", 0.010);
    const double kCommandedBodyHeightM =
        test_limits::getDouble(kSuite, kCase, "", "commanded_body_height_m", 0.14);
    const auto limitsJson = [&]() {
        return slowFwdWalkFootClearanceLimitsJson(kMinFootTipWorldZ, kRequireSawRawContactLoss, kMaxBodyUndershootM);
    };

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 26000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = harness.bus_loop_period_us;

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

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1", port, bus_loop_period_us,
        physics_sim_test_utils::productionProximalSolverSettings());
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher());
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                          limitsJson(), "{\"stage\":\"runtime_init_failed\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    // Scenario 05, t=3-12 s: TRIPOD 0.06 m/s straight forward — smallest non-fast swing height
    // in the forward-walk phases.  This exercises the swing floor at low speed without yaw.
    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.06, 0.0, 0.0};

    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(140, bus_loop_period_us));
    const int kWalkObserveSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(1400, bus_loop_period_us));
    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce an initial state before walking")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                          limitsJson(), "{\"stage\":\"no_initial_state\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const double stand_end_body_z_m = bridge_ptr->last_state().value().body_twist_state.body_trans_m.z;
    const double stand_end_governed_body_height_m = runtime.commandGovernorSnapshot().governed_body_height_m;
    const int kTransientSteps = std::max(
        1, static_cast<int>(std::llround(0.120 / std::max(1.0e-6, static_cast<double>(bus_loop_period_us) * 1.0e-6))));

    double min_foot_tip_world_z_m = std::numeric_limits<double>::infinity();
    double min_body_height_m = std::numeric_limits<double>::infinity();
    double min_body_height_first_120ms_m = std::numeric_limits<double>::infinity();
    double min_body_height_after_120ms_m = std::numeric_limits<double>::infinity();
    double min_governed_body_height_m = std::numeric_limits<double>::infinity();
    double max_governed_body_height_m = -std::numeric_limits<double>::infinity();
    double min_commanded_stance_body_z_m = std::numeric_limits<double>::infinity();
    double min_planned_stance_body_z_m = std::numeric_limits<double>::infinity();
    double min_measured_stance_foot_world_z_m = std::numeric_limits<double>::infinity();
    double max_stance_commanded_tracking_error_m = 0.0;
    int max_unchanged_target_steps = 0;
    int unchanged_target_steps = 0;
    int max_latch_candidate_steps = 0;
    int latch_candidate_steps = 0;
    std::optional<JointTargets> stand_end_targets = bridge_ptr->last_targets();
    std::optional<JointTargets> previous_targets{};
    std::vector<double> walk_body_heights_m;
    walk_body_heights_m.reserve(static_cast<std::size_t>(kWalkObserveSteps));
    std::vector<double> commanded_stance_body_z_m;
    commanded_stance_body_z_m.reserve(static_cast<std::size_t>(kWalkObserveSteps) * 3U);
    MinHeightSnapshot min_height_snapshot{};
    std::array<std::uint64_t, kNumLegs + 1> raw_contact_histogram{};
    std::optional<JointTargets> previous_applied_targets{};
    double max_joint_target_velocity_radps = 0.0;
    bool saw_any_raw_contact_loss = false;

    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion);

        const ControlStatus status = runtime.getStatus();
        if (!expect(bridge_ptr->last_state().has_value(), "bridge should keep producing live state during walk")) {
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            if (emit_metrics_json) {
                std::ostringstream metrics;
                metrics << std::setprecision(17) << "{\"stage\":\"lost_state_during_walk\",\"walk_step\":" << i
                        << ",\"min_body_height_m\":" << min_body_height_m
                        << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                        << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false") << '}';
                physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                              limitsJson(), metrics.str());
            }
            return EXIT_FAILURE;
        }
        const RobotState& state = bridge_ptr->last_state().value();
        std::array<double, kNumJoints> joint_target_velocity_radps{};
        if (previous_applied_targets.has_value()) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                    const std::size_t wire = static_cast<std::size_t>(leg * kJointsPerLeg + joint);
                    const double current = bridge_ptr->applied_targets()
                        .leg_states[static_cast<std::size_t>(leg)]
                        .joint_state[static_cast<std::size_t>(joint)].pos_rad.value;
                    const double previous = previous_applied_targets.value()
                        .leg_states[static_cast<std::size_t>(leg)]
                        .joint_state[static_cast<std::size_t>(joint)].pos_rad.value;
                    joint_target_velocity_radps[wire] =
                        std::remainder(current - previous, kTwoPi)
                        / std::max(1.0e-6, static_cast<double>(bus_loop_period_us) * 1.0e-6);
                    max_joint_target_velocity_radps = std::max(
                        max_joint_target_velocity_radps,
                        std::abs(joint_target_velocity_radps[wire]));
                }
            }
        }
        previous_applied_targets = bridge_ptr->applied_targets();
        const double body_z_m = state.body_twist_state.body_trans_m.z;
        const double governed_body_height_m = runtime.commandGovernorSnapshot().governed_body_height_m;
        const bool is_new_minimum = body_z_m < min_body_height_m;
        walk_body_heights_m.push_back(body_z_m);
        min_body_height_m = std::min(min_body_height_m, body_z_m);
        if (i < kTransientSteps) {
            min_body_height_first_120ms_m = std::min(min_body_height_first_120ms_m, body_z_m);
        } else {
            min_body_height_after_120ms_m = std::min(min_body_height_after_120ms_m, body_z_m);
        }
        min_governed_body_height_m = std::min(min_governed_body_height_m, governed_body_height_m);
        max_governed_body_height_m = std::max(max_governed_body_height_m, governed_body_height_m);
        min_foot_tip_world_z_m = std::min(min_foot_tip_world_z_m, minFootTipWorldZ(state));
        const int raw_contact_count = rawFootContactCount(state);
        if (raw_contact_count >= 0 && raw_contact_count <= kNumLegs) {
            ++raw_contact_histogram[static_cast<std::size_t>(raw_contact_count)];
        }
        saw_any_raw_contact_loss = saw_any_raw_contact_loss || (raw_contact_count < kNumLegs);

        const telemetry::LocomotionDebugSnapshot debug = runtime.locomotionDebugSnapshot();
        if (debug.valid) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t leg_index = static_cast<std::size_t>(leg);
                const bool stance = debug.fused_support[leg_index] || debug.planned_stance[leg_index];
                if (!stance) {
                    continue;
                }
                commanded_stance_body_z_m.push_back(debug.commanded_foot_body_m[leg_index].z);
                min_commanded_stance_body_z_m =
                    std::min(min_commanded_stance_body_z_m, debug.commanded_foot_body_m[leg_index].z);
                min_planned_stance_body_z_m =
                    std::min(min_planned_stance_body_z_m, debug.planned_leg_target_body_m[leg_index].z);
                min_measured_stance_foot_world_z_m =
                    std::min(min_measured_stance_foot_world_z_m, debug.measured_foot_world_m[leg_index].z);
                max_stance_commanded_tracking_error_m =
                    std::max(max_stance_commanded_tracking_error_m, debug.commanded_tracking_error_m[leg_index]);
            }
        }
        if (is_new_minimum) {
            const GaitState gait = runtime.gaitSnapshot();
            min_height_snapshot = MinHeightSnapshot{};
            min_height_snapshot.step = i;
            min_height_snapshot.body_z_m = body_z_m;
            min_height_snapshot.body_vz_mps = state.body_twist_state.body_trans_mps.z;
            min_height_snapshot.roll_rad = state.body_twist_state.twist_pos_rad.x;
            min_height_snapshot.pitch_rad = state.body_twist_state.twist_pos_rad.y;
            min_height_snapshot.raw_contacts = raw_contact_count;
            min_height_snapshot.gait_phase = gait.phase[0];
            min_height_snapshot.duty_factor = gait.duty_factor;
            min_height_snapshot.planned_supports = 0;
            min_height_snapshot.fused_supports = 0;
            min_height_snapshot.gait_stance_legs = 0;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t leg_index = static_cast<std::size_t>(leg);
                min_height_snapshot.gait_stance_legs += gait.in_stance[leg_index] ? 1 : 0;
                if (debug.valid) {
                    min_height_snapshot.tracking_error_m[leg_index] =
                        debug.commanded_tracking_error_m[leg_index];
                    min_height_snapshot.commanded_body_z_m[leg_index] =
                        debug.commanded_foot_body_m[leg_index].z;
                    min_height_snapshot.measured_world_z_m[leg_index] =
                        debug.measured_foot_world_m[leg_index].z;
                    min_height_snapshot.planned_stance[leg_index] = debug.planned_stance[leg_index];
                    min_height_snapshot.fused_support[leg_index] = debug.fused_support[leg_index];
                    min_height_snapshot.raw_contact[leg_index] = debug.raw_contact[leg_index];
                    min_height_snapshot.planned_supports += debug.planned_stance[leg_index] ? 1 : 0;
                    min_height_snapshot.fused_supports += debug.fused_support[leg_index] ? 1 : 0;
                }
                for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                    const std::size_t joint_index = static_cast<std::size_t>(joint);
                    const std::size_t wire = leg_index * kJointsPerLeg + joint_index;
                    const double error = std::remainder(
                        bridge_ptr->applied_targets().leg_states[leg_index].joint_state[joint_index].pos_rad.value
                            - state.leg_states[leg_index].joint_state[joint_index].pos_rad.value,
                        kTwoPi);
                    min_height_snapshot.joint_error_rad[wire] = error;
                    min_height_snapshot.joint_velocity_radps[wire] =
                        state.leg_states[leg_index].joint_state[joint_index].vel_radps.value;
                    min_height_snapshot.joint_target_velocity_radps[wire] =
                        joint_target_velocity_radps[wire];
                    if (std::abs(error) > min_height_snapshot.max_joint_error_rad) {
                        min_height_snapshot.max_joint_error_rad = std::abs(error);
                        min_height_snapshot.max_joint_error_wire = static_cast<int>(wire);
                    }
                }
            }
            if (bridge_ptr->last_solver_telemetry().has_value()) {
                const PhysicsSimSolverTelemetry& telemetry = bridge_ptr->last_solver_telemetry().value();
                min_height_snapshot.peak_servo_torque_utilization = telemetry.peak_servo_torque_utilization;
                min_height_snapshot.peak_actuator_impulse = telemetry.peak_actuator_impulse;
                min_height_snapshot.peak_normal_impulse = telemetry.peak_normal_impulse;
            }
        }
        if (bridge_ptr->last_targets().has_value()) {
            const JointTargets& targets = bridge_ptr->last_targets().value();
            const bool stationary_targets =
                previous_targets.has_value() &&
                maxJointTargetDeltaRad(previous_targets.value(), targets) < kStationaryJointTargetDeltaRad;
            if (stationary_targets) {
                ++unchanged_target_steps;
            } else {
                unchanged_target_steps = 0;
            }
            max_unchanged_target_steps = std::max(max_unchanged_target_steps, unchanged_target_steps);
            const int support_leg_count = rawFootContactCount(state);
            const int changed_leg_count =
                stand_end_targets.has_value() ? changedLegCountVsReference(targets, stand_end_targets.value()) : 0;
            const bool latch_candidate =
                stationary_targets && stand_end_targets.has_value() && changed_leg_count == 3 &&
                support_leg_count > 0 && support_leg_count < kNumLegs;
            if (latch_candidate) {
                ++latch_candidate_steps;
            } else {
                latch_candidate_steps = 0;
            }
            max_latch_candidate_steps = std::max(max_latch_candidate_steps, latch_candidate_steps);
            previous_targets = targets;
        }

        if (status.active_fault != FaultCode::NONE) {
            std::cerr << "walk step=" << i
                      << " fault=" << static_cast<int>(status.active_fault)
                      << " body_height=" << state.body_twist_state.body_trans_m.z
                      << " min_foot_tip_z=" << min_foot_tip_world_z_m
                      << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            if (emit_metrics_json) {
                const double undershoot = kCommandedBodyHeightM - min_body_height_m;
                std::ostringstream metrics;
                metrics << std::setprecision(17) << "{\"stage\":\"fault_during_walk\",\"walk_step\":" << i
                        << ",\"min_body_height_m\":" << min_body_height_m
                        << ",\"max_body_undershoot_m\":" << undershoot
                        << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                        << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false") << '}';
                physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                              limitsJson(), metrics.str());
            }
            return EXIT_FAILURE;
        }
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    const double max_body_undershoot_m = kCommandedBodyHeightM - min_body_height_m;
    auto median_of = [](std::vector<double> values) -> double {
        if (values.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        std::sort(values.begin(), values.end());
        const std::size_t mid = values.size() / 2;
        if (values.size() % 2 == 0) {
            return 0.5 * (values[mid - 1] + values[mid]);
        }
        return values[mid];
    };
    const double median_body_height_m = median_of(walk_body_heights_m);
    const double median_commanded_stance_body_z_m = median_of(commanded_stance_body_z_m);
    const bool has_post_transient = std::isfinite(min_body_height_after_120ms_m);
    const double step_s = std::max(1.0e-6, static_cast<double>(bus_loop_period_us) * 1.0e-6);
    const double max_unchanged_target_s = static_cast<double>(max_unchanged_target_steps) * step_s;
    const double max_latch_candidate_s = static_cast<double>(max_latch_candidate_steps) * step_s;
    const bool reduced_support_latch_could_arm = max_latch_candidate_s >= kStaticReducedSupportDwellS;

    std::cout << "slow_fwd_walk_min_body_height_m=" << min_body_height_m
              << " max_body_undershoot_m=" << max_body_undershoot_m
              << " min_foot_tip_world_z_m=" << min_foot_tip_world_z_m
              << " saw_any_raw_contact_loss=" << (saw_any_raw_contact_loss ? 1 : 0)
              << " stand_end_body_z_m=" << stand_end_body_z_m
              << " stand_end_governed_body_height_m=" << stand_end_governed_body_height_m
              << " median_body_height_m=" << median_body_height_m
              << " min_body_height_first_120ms_m=" << min_body_height_first_120ms_m
              << " min_body_height_after_120ms_m=" << min_body_height_after_120ms_m
              << " min_governed_body_height_m=" << min_governed_body_height_m
              << " max_governed_body_height_m=" << max_governed_body_height_m
              << " min_commanded_stance_body_z_m=" << min_commanded_stance_body_z_m
              << " median_commanded_stance_body_z_m=" << median_commanded_stance_body_z_m
              << " min_planned_stance_body_z_m=" << min_planned_stance_body_z_m
              << " min_measured_stance_foot_world_z_m=" << min_measured_stance_foot_world_z_m
              << " max_stance_commanded_tracking_error_m=" << max_stance_commanded_tracking_error_m
              << " max_unchanged_target_s=" << max_unchanged_target_s
              << " max_latch_candidate_s=" << max_latch_candidate_s
              << " reduced_support_latch_could_arm=" << (reduced_support_latch_could_arm ? 1 : 0)
              << " min_step=" << min_height_snapshot.step
              << " min_body_vz_mps=" << min_height_snapshot.body_vz_mps
              << " min_roll_rad=" << min_height_snapshot.roll_rad
              << " min_pitch_rad=" << min_height_snapshot.pitch_rad
              << " min_raw_contacts=" << min_height_snapshot.raw_contacts
              << " min_planned_supports=" << min_height_snapshot.planned_supports
              << " min_fused_supports=" << min_height_snapshot.fused_supports
              << " min_gait_stance_legs=" << min_height_snapshot.gait_stance_legs
              << " min_gait_phase=" << min_height_snapshot.gait_phase
              << " min_peak_servo_torque_utilization="
              << min_height_snapshot.peak_servo_torque_utilization
              << " min_peak_actuator_impulse=" << min_height_snapshot.peak_actuator_impulse
              << " min_peak_normal_impulse=" << min_height_snapshot.peak_normal_impulse
              << " min_max_joint_error_rad=" << min_height_snapshot.max_joint_error_rad
              << " min_max_joint_error_wire=" << min_height_snapshot.max_joint_error_wire
              << " max_joint_target_velocity_radps=" << max_joint_target_velocity_radps
              << '\n';
    std::cout << "slow_fwd_walk_raw_contact_histogram=";
    for (std::size_t count = 0; count < raw_contact_histogram.size(); ++count) {
        std::cout << (count == 0 ? "" : ",") << count << ':' << raw_contact_histogram[count];
    }
    std::cout << '\n';
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t leg_index = static_cast<std::size_t>(leg);
        std::cout << "slow_fwd_walk_min_leg=" << leg
                  << " planned=" << (min_height_snapshot.planned_stance[leg_index] ? 1 : 0)
                  << " fused=" << (min_height_snapshot.fused_support[leg_index] ? 1 : 0)
                  << " raw=" << (min_height_snapshot.raw_contact[leg_index] ? 1 : 0)
                  << " tracking_m=" << min_height_snapshot.tracking_error_m[leg_index]
                  << " commanded_body_z_m=" << min_height_snapshot.commanded_body_z_m[leg_index]
                  << " measured_world_z_m=" << min_height_snapshot.measured_world_z_m[leg_index];
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const std::size_t wire = leg_index * kJointsPerLeg + static_cast<std::size_t>(joint);
            std::cout << " j" << joint << "_error_rad=" << min_height_snapshot.joint_error_rad[wire]
                      << " j" << joint << "_vel_radps=" << min_height_snapshot.joint_velocity_radps[wire]
                      << " j" << joint << "_target_vel_radps="
                      << min_height_snapshot.joint_target_velocity_radps[wire];
        }
        std::cout << '\n';
    }

    bool ok = true;
    ok = expect(min_foot_tip_world_z_m >= kMinFootTipWorldZ,
                "slow forward TRIPOD walk should keep the reported server foot tip at or above the ground plane") &&
         ok;
    ok = expect(!kRequireSawRawContactLoss || saw_any_raw_contact_loss,
                "slow forward TRIPOD walk should see at least one swing-leg contact loss confirming feet lift") &&
         ok;
    ok = expect(max_body_undershoot_m < kMaxBodyUndershootM,
                "body height should track within 10 mm of commanded 0.14 m during slow forward walk") &&
         ok;
    if (emit_metrics_json) {
        std::ostringstream metrics;
        metrics << std::setprecision(17) << "{\"min_body_height_m\":" << min_body_height_m
                << ",\"max_body_undershoot_m\":" << max_body_undershoot_m
                << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false")
                << ",\"stand_end_body_z_m\":" << stand_end_body_z_m
                << ",\"stand_end_governed_body_height_m\":" << stand_end_governed_body_height_m
                << ",\"median_body_height_m\":" << median_body_height_m
                << ",\"min_body_height_first_120ms_m\":" << min_body_height_first_120ms_m
                << ",\"min_body_height_after_120ms_m\":" << min_body_height_after_120ms_m
                << ",\"has_post_transient\":" << (has_post_transient ? "true" : "false")
                << ",\"min_governed_body_height_m\":" << min_governed_body_height_m
                << ",\"max_governed_body_height_m\":" << max_governed_body_height_m
                << ",\"min_commanded_stance_body_z_m\":" << min_commanded_stance_body_z_m
                << ",\"median_commanded_stance_body_z_m\":" << median_commanded_stance_body_z_m
                << ",\"min_planned_stance_body_z_m\":" << min_planned_stance_body_z_m
                << ",\"min_measured_stance_foot_world_z_m\":" << min_measured_stance_foot_world_z_m
                << ",\"max_stance_commanded_tracking_error_m\":" << max_stance_commanded_tracking_error_m
                << ",\"max_unchanged_target_s\":" << max_unchanged_target_s
                << ",\"max_latch_candidate_s\":" << max_latch_candidate_s
                << ",\"reduced_support_latch_could_arm\":"
                << (reduced_support_latch_could_arm ? "true" : "false") << '}';
        physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
