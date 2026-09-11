#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_test_utils.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

enum class ReplayPhase : std::uint8_t {
    Stand,
    Transition,
    Forward,
    Reverse,
    Strafe,
    Diagonal,
    TurnInPlace,
    Count,
};

constexpr std::size_t kPhaseCount = static_cast<std::size_t>(ReplayPhase::Count);

constexpr std::array<const char*, kPhaseCount> kPhaseNames{
    "stand", "transition", "forward", "reverse", "strafe", "diagonal", "turn_in_place"};

struct CapturedFrame {
    JointTargets targets{};
    ReplayPhase phase{ReplayPhase::Stand};
};

struct PhaseResult {
    std::uint64_t frames{0};
    std::uint64_t healthy{0};
    std::uint64_t recovered{0};
    std::uint64_t held{0};
    std::uint64_t unsupported{0};
    std::uint64_t read_failures{0};
    std::uint64_t solver_not_converged{0};
    std::uint16_t max_iterations{0};
    double max_ncp_dual_residual{0.0};
    double max_ncp_complementarity_residual{0.0};
    double max_contact_penetration{0.0};
    std::optional<Vec3> first_valid_position{};
    std::optional<Vec3> last_valid_position{};
    double completed_delta_x_sum{0.0};
    double completed_delta_y_sum{0.0};
    std::uint64_t completed_trajectories{0};
};

struct ReplayResult {
    std::array<PhaseResult, kPhaseCount> phases{};
    std::uint64_t frames{0};
    std::uint64_t telemetry_frames{0};
    std::uint64_t healthy{0};
    std::uint64_t recovered{0};
    std::uint64_t held{0};
    std::uint64_t unsupported{0};
    std::uint64_t read_failures{0};
    std::uint64_t solver_not_converged{0};
    std::uint16_t max_iterations{0};
    std::uint32_t max_contact_constraints{0};
    std::uint64_t max_warm_start_resets{0};
    double p99_step_time_ms{0.0};
    double max_step_time_ms{0.0};
    double healthy_p99_step_time_ms{0.0};
    double healthy_max_step_time_ms{0.0};
    double p99_solver_dynamics_time_ms{0.0};
    double p99_solver_contact_setup_time_ms{0.0};
    double p99_solver_collision_time_ms{0.0};
    double p99_solver_constraint_assembly_time_ms{0.0};
    double p99_solver_delassus_time_ms{0.0};
    double p99_solver_admm_time_ms{0.0};
    double p99_solver_integration_time_ms{0.0};
    double p99_solver_total_step_time_ms{0.0};
};

class CommandCapturingBridge final : public IHardwareBridge {
public:
    CommandCapturingBridge(std::string host,
                           int port,
                           int bus_loop_period_us,
                           PhysicsSimSolverSettings settings)
        : inner_(std::move(host), port, bus_loop_period_us, settings, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        // Capture the target that this read is about to advance.  The initial
        // all-zero value is intentional: PhysicsSimBridge maps it to the same
        // standing seed used during normal runtime start-up.
        frames_.push_back(CapturedFrame{pending_targets_, phase_});
        return inner_.read(out);
    }

    bool write(const JointTargets& in) override {
        pending_targets_ = in;
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    bool supportsAutomaticBusTimeoutRecovery() const override {
        return inner_.supportsAutomaticBusTimeoutRecovery();
    }

    bool latestSampleHealthyForAutomaticRecovery() const override {
        return inner_.latestSampleHealthyForAutomaticRecovery();
    }

    void setPhase(const ReplayPhase phase) { phase_ = phase; }

    const std::vector<CapturedFrame>& frames() const { return frames_; }

private:
    PhysicsSimBridge inner_;
    JointTargets pending_targets_{};
    ReplayPhase phase_{ReplayPhase::Stand};
    std::vector<CapturedFrame> frames_{};
};

bool targetsAreFinite(const JointTargets& targets) {
    for (const LegState& leg : targets.leg_states) {
        for (const JointState& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value) || !std::isfinite(joint.vel_radps.value)) {
                return false;
            }
        }
    }
    return true;
}

void hashBytes(std::uint64_t& hash, const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= 1099511628211ULL;
    }
}

std::uint64_t commandStreamHash(const std::vector<CapturedFrame>& frames) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const CapturedFrame& frame : frames) {
        const auto phase = static_cast<std::uint8_t>(frame.phase);
        hashBytes(hash, &phase, sizeof(phase));
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                hashBytes(hash, &joint.pos_rad.value, sizeof(joint.pos_rad.value));
                hashBytes(hash, &joint.vel_radps.value, sizeof(joint.vel_radps.value));
            }
        }
    }
    return hash;
}

MotionIntent makeReplayIntent(const ReplayPhase phase, const double body_height_m) {
    ScenarioMotionIntent scenario{};
    scenario.enabled = true;
    scenario.mode = phase == ReplayPhase::Stand || phase == ReplayPhase::Transition
        ? RobotMode::STAND
        : RobotMode::WALK;
    scenario.gait = GaitType::TRIPOD;
    scenario.body_height_m = body_height_m;
    scenario.has_direct_velocity = true;
    switch (phase) {
    case ReplayPhase::Forward:
        scenario.vx_mps = 0.12;
        break;
    case ReplayPhase::Reverse:
        scenario.vx_mps = -0.12;
        break;
    case ReplayPhase::Strafe:
        scenario.vy_mps = 0.10;
        break;
    case ReplayPhase::Diagonal:
        scenario.vx_mps = 0.085;
        scenario.vy_mps = 0.085;
        break;
    case ReplayPhase::TurnInPlace:
        scenario.yaw_rate_radps = 0.45;
        break;
    case ReplayPhase::Stand:
    case ReplayPhase::Transition:
    case ReplayPhase::Count:
        break;
    }
    return makeMotionIntent(scenario);
}

int positiveEnvOrDefault(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed <= 0 || parsed > 1000000) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return static_cast<int>(parsed);
}

int nonnegativeEnvOrDefault(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < 0 || parsed > 1000000) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return static_cast<int>(parsed);
}

double positiveDoubleEnvOrDefault(const char* name, const double fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return parsed;
}

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

std::uint64_t splitMix64(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

double seededUnitInterval(const std::uint64_t seed, const std::uint64_t channel) {
    const std::uint64_t bits = splitMix64(seed ^ (channel * 0x9e3779b97f4a7c15ULL));
    return static_cast<double>(bits >> 11U) * (1.0 / 9007199254740992.0);
}

std::array<double, 4> multiplyQuaternion(const std::array<double, 4>& lhs,
                                         const std::array<double, 4>& rhs) {
    return {
        lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0],
    };
}

physics_sim::StateCorrection makePerturbedStandingCorrection(
    const std::uint64_t seed,
    const double body_height_m,
    const double scale) {
    const auto symmetric = [seed](const std::uint64_t channel) {
        return 2.0 * seededUnitInterval(seed, channel) - 1.0;
    };
    const double horizontal_x = seed == 0 ? 0.0 : scale * 0.003 * symmetric(1);
    const double vertical = seed == 0 ? 0.0 : scale * 0.0015 * symmetric(2);
    const double horizontal_z = seed == 0 ? 0.0 : scale * 0.003 * symmetric(3);
    constexpr double kDegreesToRadians = 0.01745329251994329577;
    const double roll = seed == 0 ? 0.0 : scale * 0.75 * kDegreesToRadians * symmetric(4);
    const double yaw = seed == 0 ? 0.0 : scale * 1.0 * kDegreesToRadians * symmetric(5);
    const double pitch = seed == 0 ? 0.0 : scale * 0.75 * kDegreesToRadians * symmetric(6);
    const std::array<double, 4> qx{
        std::cos(0.5 * roll), std::sin(0.5 * roll), 0.0, 0.0};
    const std::array<double, 4> qy{
        std::cos(0.5 * yaw), 0.0, std::sin(0.5 * yaw), 0.0};
    const std::array<double, 4> qz{
        std::cos(0.5 * pitch), 0.0, 0.0, std::sin(0.5 * pitch)};
    const std::array<double, 4> orientation =
        multiplyQuaternion(qy, multiplyQuaternion(qz, qx));

    physics_sim::StateCorrection correction{};
    correction.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    correction.sequence_id = static_cast<std::uint32_t>(seed);
    correction.timestamp_us = now_us().value;
    correction.flags = physics_sim::kStateCorrectionPoseValid
        | physics_sim::kStateCorrectionTwistValid
        | physics_sim::kStateCorrectionHardReset;
    correction.correction_strength = 1.0f;
    correction.body_position = {
        static_cast<float>(horizontal_x),
        static_cast<float>(body_height_m + vertical),
        static_cast<float>(horizontal_z)};
    correction.body_orientation = {
        static_cast<float>(orientation[0]),
        static_cast<float>(orientation[1]),
        static_cast<float>(orientation[2]),
        static_cast<float>(orientation[3])};
    correction.body_linear_velocity = {0.0f, 0.0f, 0.0f};
    correction.body_angular_velocity = {0.0f, 0.0f, 0.0f};
    return correction;
}

std::optional<ReplayPhase> selectedMotionPhase() {
    const char* value = std::getenv("HEXAPOD_EXACT_REPLAY_MOTION_CASE");
    if (value == nullptr || value[0] == '\0' || std::string(value) == "all") {
        return std::nullopt;
    }
    constexpr std::array<ReplayPhase, 5> motion_phases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
        ReplayPhase::TurnInPlace,
    };
    for (const ReplayPhase phase : motion_phases) {
        if (std::string(value) == kPhaseNames[static_cast<std::size_t>(phase)]) {
            return phase;
        }
    }
    throw std::runtime_error(
        std::string("invalid HEXAPOD_EXACT_REPLAY_MOTION_CASE=") + value);
}

#if defined(__linux__)
pid_t launchSimulator(const char* sim_exe,
                      const int port,
                      const std::uint64_t contact_order_seed = 0) {
    const pid_t pid = ::fork();
    if (pid != 0) {
        return pid;
    }
    if (!envEnabled("HEXAPOD_EXACT_REPLAY_CHILD_STDIO")) {
        physics_sim_test_utils::quietChildProcessStdIo();
    }
    const std::string contact_order_seed_text = std::to_string(contact_order_seed);
    (void)::setenv("HEXAPOD_PINOCCHIO_CONTACT_ORDER_SEED",
                   contact_order_seed_text.c_str(),
                   1);
    const std::string port_text = std::to_string(port);
    ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_text.c_str(), nullptr);
    std::perror("execl");
    _exit(127);
}

void stopSimulator(const pid_t pid) {
    if (pid > 0) {
        (void)::kill(pid, SIGTERM);
        (void)::waitpid(pid, nullptr, 0);
    }
}
#endif

void runRuntimeFrame(RobotRuntime& runtime,
                     CommandCapturingBridge& bridge,
                     const ReplayPhase phase,
                     MotionIntent intent,
                     TimePointUs& command_time,
                     const int bus_loop_period_us) {
    bridge.setPhase(phase);
    command_time.value += static_cast<std::uint64_t>(bus_loop_period_us);
    intent.timestamp_us = command_time;
    intent.sample_id = 0;
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

std::vector<CapturedFrame> captureCommands(const physics_sim_test_utils::HarnessSettings& harness,
                                           const int port,
                                           const int stand_frames,
                                           const int motion_frames,
                                           const int transition_frames,
                                           const double body_height_m,
                                           const std::optional<ReplayPhase> selected_phase) {
    PhysicsSimSolverSettings legacy{};
    legacy.mode = physics_sim::PhysicsSolverMode::LegacyPgs;
    legacy.iterations = harness.physics_solver_iterations;
    auto bridge = std::make_unique<CommandCapturingBridge>(
        "127.0.0.1", port, harness.bus_loop_period_us, legacy);
    CommandCapturingBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig config = harness.control_cfg;
    config.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    config.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, config);
    if (!runtime.init()) {
        throw std::runtime_error("legacy command-capture runtime failed to initialise");
    }

    TimePointUs command_time{now_us().value + 3'600'000'000ULL};
    const auto run_phase = [&](const ReplayPhase phase, const int frames) {
        const MotionIntent intent = makeReplayIntent(phase, body_height_m);
        for (int i = 0; i < frames; ++i) {
            runRuntimeFrame(runtime,
                            *bridge_ptr,
                            phase,
                            intent,
                            command_time,
                            harness.bus_loop_period_us);
        }
    };

    run_phase(ReplayPhase::Stand, stand_frames);
    constexpr std::array<ReplayPhase, 5> motion_phases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
        ReplayPhase::TurnInPlace,
    };
    for (const ReplayPhase phase : motion_phases) {
        if (selected_phase.has_value() && phase != *selected_phase) {
            continue;
        }
        run_phase(phase, motion_frames);
        run_phase(ReplayPhase::Transition, transition_frames);
    }
    return bridge_ptr->frames();
}

ReplayResult replayCommands(const std::vector<CapturedFrame>& frames,
                            const int port,
                            const int replay_period_us,
                            const int solver_iterations,
                            const double absolute_tolerance,
                            const double relative_tolerance,
                            const std::uint64_t perturbation_seed,
                            const double perturbation_scale) {
    PhysicsSimSolverSettings proximal{};
    proximal.mode = physics_sim::PhysicsSolverMode::PinocchioProximal;
    proximal.iterations = solver_iterations;
    proximal.absolute_tolerance = static_cast<float>(absolute_tolerance);
    proximal.relative_tolerance = static_cast<float>(relative_tolerance);
    PhysicsSimBridge bridge(
        "127.0.0.1", port, replay_period_us, proximal, nullptr);
    if (!bridge.init()) {
        throw std::runtime_error("proximal replay bridge failed to initialise");
    }
    if (!bridge.sendStateCorrection(
            makePerturbedStandingCorrection(
                perturbation_seed,
                physicsSimStandingBodyHeightM(),
                perturbation_scale))) {
        throw std::runtime_error("proximal replay failed to send initial-pose perturbation");
    }

    ReplayResult result{};
    std::vector<double> step_times_ms{};
    std::vector<double> healthy_step_times_ms{};
    std::vector<double> solver_dynamics_times_ms{};
    std::vector<double> solver_contact_setup_times_ms{};
    std::vector<double> solver_collision_times_ms{};
    std::vector<double> solver_constraint_assembly_times_ms{};
    std::vector<double> solver_delassus_times_ms{};
    std::vector<double> solver_admm_times_ms{};
    std::vector<double> solver_integration_times_ms{};
    std::vector<double> solver_total_step_times_ms{};
    step_times_ms.reserve(frames.size());
    healthy_step_times_ms.reserve(frames.size());
    solver_dynamics_times_ms.reserve(frames.size());
    solver_contact_setup_times_ms.reserve(frames.size());
    solver_collision_times_ms.reserve(frames.size());
    solver_constraint_assembly_times_ms.reserve(frames.size());
    solver_delassus_times_ms.reserve(frames.size());
    solver_admm_times_ms.reserve(frames.size());
    solver_integration_times_ms.reserve(frames.size());
    solver_total_step_times_ms.reserve(frames.size());
    for (const CapturedFrame& frame : frames) {
        PhaseResult& phase = result.phases[static_cast<std::size_t>(frame.phase)];
        ++result.frames;
        ++phase.frames;
        if (!bridge.write(frame.targets)) {
            ++result.read_failures;
            ++phase.read_failures;
            continue;
        }

        RobotState state{};
        const auto step_start = std::chrono::steady_clock::now();
        const bool read_ok = bridge.read(state);
        const auto step_end = std::chrono::steady_clock::now();
        const double step_time_ms =
            std::chrono::duration<double, std::milli>(step_end - step_start).count();
        step_times_ms.push_back(step_time_ms);
        if (!read_ok) {
            ++result.read_failures;
            ++phase.read_failures;
        } else {
            const Vec3 position{state.body_twist_state.body_trans_m.x,
                                state.body_twist_state.body_trans_m.y,
                                state.body_twist_state.body_trans_m.z};
            if (!phase.first_valid_position.has_value()) {
                phase.first_valid_position = position;
            }
            phase.last_valid_position = position;
        }

        const auto telemetry = bridge.latestSolverTelemetry();
        if (!telemetry.has_value()) {
            continue;
        }
        ++result.telemetry_frames;
        solver_dynamics_times_ms.push_back(telemetry->dynamics_time_ms);
        solver_contact_setup_times_ms.push_back(telemetry->contact_setup_time_ms);
        solver_collision_times_ms.push_back(telemetry->collision_time_ms);
        solver_constraint_assembly_times_ms.push_back(
            telemetry->constraint_assembly_time_ms);
        solver_delassus_times_ms.push_back(telemetry->delassus_time_ms);
        solver_admm_times_ms.push_back(telemetry->admm_time_ms);
        solver_integration_times_ms.push_back(telemetry->integration_time_ms);
        solver_total_step_times_ms.push_back(telemetry->total_step_time_ms);
        result.max_iterations = std::max(result.max_iterations, telemetry->iterations);
        result.max_contact_constraints = std::max(
            result.max_contact_constraints, telemetry->contact_constraint_count);
        result.max_warm_start_resets = std::max(
            result.max_warm_start_resets, telemetry->warm_start_reset_count);
        phase.max_iterations = std::max(phase.max_iterations, telemetry->iterations);
        phase.max_ncp_dual_residual =
            std::max(phase.max_ncp_dual_residual, static_cast<double>(telemetry->ncp_dual_residual));
        phase.max_ncp_complementarity_residual = std::max(
            phase.max_ncp_complementarity_residual,
            static_cast<double>(telemetry->ncp_complementarity_residual));
        phase.max_contact_penetration = std::max(
            phase.max_contact_penetration,
            static_cast<double>(telemetry->max_contact_penetration));
        if (telemetry->failure_reason == physics_sim::SolverFailureReason::SolverNotConverged) {
            ++result.solver_not_converged;
            ++phase.solver_not_converged;
        }
        switch (telemetry->status) {
        case physics_sim::SolverStatus::Healthy:
            healthy_step_times_ms.push_back(step_time_ms);
            ++result.healthy;
            ++phase.healthy;
            break;
        case physics_sim::SolverStatus::RecoveredRetry:
            ++result.recovered;
            ++phase.recovered;
            break;
        case physics_sim::SolverStatus::HeldLastGood:
            ++result.held;
            ++phase.held;
            break;
        case physics_sim::SolverStatus::UnsupportedIsland:
            ++result.unsupported;
            ++phase.unsupported;
            break;
        }
    }
    const auto assignTimingSummary = [](std::vector<double>& samples,
                                        double& p99,
                                        double& maximum) {
        if (samples.empty()) {
            return;
        }
        std::sort(samples.begin(), samples.end());
        const std::size_t p99_index = std::min(
            samples.size() - 1,
            static_cast<std::size_t>(
                std::ceil(0.99 * static_cast<double>(samples.size()))) - 1);
        p99 = samples[p99_index];
        maximum = samples.back();
    };
    assignTimingSummary(
        step_times_ms, result.p99_step_time_ms, result.max_step_time_ms);
    assignTimingSummary(healthy_step_times_ms,
                        result.healthy_p99_step_time_ms,
                        result.healthy_max_step_time_ms);
    double ignoredMaximum = 0.0;
    assignTimingSummary(solver_dynamics_times_ms,
                        result.p99_solver_dynamics_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_contact_setup_times_ms,
                        result.p99_solver_contact_setup_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_collision_times_ms,
                        result.p99_solver_collision_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_constraint_assembly_times_ms,
                        result.p99_solver_constraint_assembly_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_delassus_times_ms,
                        result.p99_solver_delassus_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_admm_times_ms,
                        result.p99_solver_admm_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_integration_times_ms,
                        result.p99_solver_integration_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_total_step_times_ms,
                        result.p99_solver_total_step_time_ms,
                        ignoredMaximum);
    for (PhaseResult& phase : result.phases) {
        if (phase.first_valid_position.has_value()
            && phase.last_valid_position.has_value()) {
            phase.completed_delta_x_sum =
                phase.last_valid_position->x - phase.first_valid_position->x;
            phase.completed_delta_y_sum =
                phase.last_valid_position->y - phase.first_valid_position->y;
            phase.completed_trajectories = 1;
        }
    }
    return result;
}

void accumulateReplayResult(ReplayResult& total, const ReplayResult& sample) {
    total.frames += sample.frames;
    total.telemetry_frames += sample.telemetry_frames;
    total.healthy += sample.healthy;
    total.recovered += sample.recovered;
    total.held += sample.held;
    total.unsupported += sample.unsupported;
    total.read_failures += sample.read_failures;
    total.solver_not_converged += sample.solver_not_converged;
    total.max_iterations = std::max(total.max_iterations, sample.max_iterations);
    total.max_contact_constraints = std::max(
        total.max_contact_constraints, sample.max_contact_constraints);
    total.max_warm_start_resets = std::max(
        total.max_warm_start_resets, sample.max_warm_start_resets);
    const auto accumulateMaximum = [](double& aggregate, const double value) {
        aggregate = std::max(aggregate, value);
    };
    accumulateMaximum(total.p99_step_time_ms, sample.p99_step_time_ms);
    accumulateMaximum(total.max_step_time_ms, sample.max_step_time_ms);
    accumulateMaximum(total.healthy_p99_step_time_ms, sample.healthy_p99_step_time_ms);
    accumulateMaximum(total.healthy_max_step_time_ms, sample.healthy_max_step_time_ms);
    accumulateMaximum(total.p99_solver_dynamics_time_ms, sample.p99_solver_dynamics_time_ms);
    accumulateMaximum(total.p99_solver_contact_setup_time_ms, sample.p99_solver_contact_setup_time_ms);
    accumulateMaximum(total.p99_solver_collision_time_ms, sample.p99_solver_collision_time_ms);
    accumulateMaximum(
        total.p99_solver_constraint_assembly_time_ms,
        sample.p99_solver_constraint_assembly_time_ms);
    accumulateMaximum(total.p99_solver_delassus_time_ms, sample.p99_solver_delassus_time_ms);
    accumulateMaximum(total.p99_solver_admm_time_ms, sample.p99_solver_admm_time_ms);
    accumulateMaximum(
        total.p99_solver_integration_time_ms,
        sample.p99_solver_integration_time_ms);
    accumulateMaximum(
        total.p99_solver_total_step_time_ms,
        sample.p99_solver_total_step_time_ms);
    for (std::size_t i = 0; i < total.phases.size(); ++i) {
        PhaseResult& out = total.phases[i];
        const PhaseResult& in = sample.phases[i];
        out.frames += in.frames;
        out.healthy += in.healthy;
        out.recovered += in.recovered;
        out.held += in.held;
        out.unsupported += in.unsupported;
        out.read_failures += in.read_failures;
        out.solver_not_converged += in.solver_not_converged;
        out.max_iterations = std::max(out.max_iterations, in.max_iterations);
        out.max_ncp_dual_residual = std::max(
            out.max_ncp_dual_residual, in.max_ncp_dual_residual);
        out.max_ncp_complementarity_residual = std::max(
            out.max_ncp_complementarity_residual,
            in.max_ncp_complementarity_residual);
        out.max_contact_penetration = std::max(
            out.max_contact_penetration, in.max_contact_penetration);
        out.completed_delta_x_sum += in.completed_delta_x_sum;
        out.completed_delta_y_sum += in.completed_delta_y_sum;
        out.completed_trajectories += in.completed_trajectories;
    }
}

std::string metricsJson(const ReplayResult& result,
                        const std::uint64_t command_hash,
                        const std::size_t captured_frames,
                        const int perturbation_seed_count,
                        const int perturbation_seed_offset,
                        const double perturbation_scale,
                        const bool fixed_initial_pose,
                        const bool fixed_contact_order,
                        const int replay_period_us,
                        const int solver_iterations,
                        const double absolute_tolerance,
                        const double relative_tolerance) {
    std::ostringstream out;
    out << std::setprecision(9)
        << "{\"captured_frames\":" << captured_frames
        << ",\"replayed_frames\":" << result.frames
        << ",\"telemetry_frames\":" << result.telemetry_frames
        << ",\"command_hash\":\"" << std::hex << command_hash << std::dec << "\""
        << ",\"perturbation_seed_count\":" << perturbation_seed_count
        << ",\"perturbation_seed_offset\":" << perturbation_seed_offset
        << ",\"perturbation_scale\":" << perturbation_scale
        << ",\"fixed_initial_pose\":" << (fixed_initial_pose ? "true" : "false")
        << ",\"fixed_contact_order\":" << (fixed_contact_order ? "true" : "false")
        << ",\"replay_period_us\":" << replay_period_us
        << ",\"solver_iteration_limit\":" << solver_iterations
        << ",\"absolute_tolerance\":" << absolute_tolerance
        << ",\"relative_tolerance\":" << relative_tolerance
        << ",\"healthy\":" << result.healthy
        << ",\"recovered\":" << result.recovered
        << ",\"held\":" << result.held
        << ",\"unsupported\":" << result.unsupported
        << ",\"read_failures\":" << result.read_failures
        << ",\"solver_not_converged\":" << result.solver_not_converged
        << ",\"max_iterations\":" << result.max_iterations
        << ",\"max_contact_constraints\":" << result.max_contact_constraints
        << ",\"max_warm_start_resets\":" << result.max_warm_start_resets
        << ",\"p99_step_time_ms\":" << result.p99_step_time_ms
        << ",\"max_step_time_ms\":" << result.max_step_time_ms
        << ",\"healthy_p99_step_time_ms\":" << result.healthy_p99_step_time_ms
        << ",\"healthy_max_step_time_ms\":" << result.healthy_max_step_time_ms
        << ",\"p99_solver_dynamics_time_ms\":" << result.p99_solver_dynamics_time_ms
        << ",\"p99_solver_contact_setup_time_ms\":"
        << result.p99_solver_contact_setup_time_ms
        << ",\"p99_solver_collision_time_ms\":" << result.p99_solver_collision_time_ms
        << ",\"p99_solver_constraint_assembly_time_ms\":"
        << result.p99_solver_constraint_assembly_time_ms
        << ",\"p99_solver_delassus_time_ms\":" << result.p99_solver_delassus_time_ms
        << ",\"p99_solver_admm_time_ms\":" << result.p99_solver_admm_time_ms
        << ",\"p99_solver_integration_time_ms\":"
        << result.p99_solver_integration_time_ms
        << ",\"p99_solver_total_step_time_ms\":"
        << result.p99_solver_total_step_time_ms
        << ",\"phases\":[";
    for (std::size_t i = 0; i < result.phases.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        const PhaseResult& phase = result.phases[i];
        const double dx = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_x_sum
                / static_cast<double>(phase.completed_trajectories);
        const double dy = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_y_sum
                / static_cast<double>(phase.completed_trajectories);
        out << "{\"name\":\"" << kPhaseNames[i]
            << "\",\"frames\":" << phase.frames
            << ",\"healthy\":" << phase.healthy
            << ",\"recovered\":" << phase.recovered
            << ",\"held\":" << phase.held
            << ",\"unsupported\":" << phase.unsupported
            << ",\"read_failures\":" << phase.read_failures
            << ",\"solver_not_converged\":" << phase.solver_not_converged
            << ",\"max_iterations\":" << phase.max_iterations
            << ",\"max_ncp_dual_residual\":" << phase.max_ncp_dual_residual
            << ",\"max_ncp_complementarity_residual\":"
            << phase.max_ncp_complementarity_residual
            << ",\"max_contact_penetration_m\":" << phase.max_contact_penetration
            << ",\"valid_delta_x_m\":" << dx
            << ",\"valid_delta_y_m\":" << dy << '}';
    }
    out << "]}";
    return out.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_exact_command_replay (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_exact_command_replay "
                     "(pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    try {
        const auto harness = physics_sim_test_utils::loadHarnessSettings(true);
        const int stand_frames = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_STAND_FRAMES", 240);
        const int motion_frames = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_MOTION_FRAMES", 72);
        const int transition_frames =
            positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_TRANSITION_FRAMES", 24);
        const int solver_iterations =
            positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS", 50);
        const int replay_period_us = positiveEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERIOD_US", harness.bus_loop_period_us);
        const double absolute_tolerance = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_ABSOLUTE_TOLERANCE", 1.0e-8);
        const double relative_tolerance = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_RELATIVE_TOLERANCE", 1.0e-6);
        const int perturbation_seed_count = positiveEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS", 1);
        const int perturbation_seed_offset = nonnegativeEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEED_OFFSET", 0);
        const double perturbation_scale = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SCALE", 0.25);
        const bool fixed_initial_pose = envEnabled(
            "HEXAPOD_EXACT_REPLAY_FIXED_INITIAL_POSE");
        const bool fixed_contact_order = envEnabled(
            "HEXAPOD_EXACT_REPLAY_FIXED_CONTACT_ORDER");
        if (perturbation_seed_count > 1000) {
            throw std::runtime_error(
                "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS must be at most 1000");
        }
        const std::optional<ReplayPhase> selected_phase = selectedMotionPhase();
        const int base_port = 23500 + (static_cast<int>(::getpid()) % 4000);

        pid_t capture_pid = launchSimulator(sim_exe, base_port);
        if (capture_pid < 0) {
            throw std::runtime_error("failed to fork legacy capture simulator");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
        std::vector<CapturedFrame> frames{};
        try {
            frames = captureCommands(
                harness,
                base_port,
                stand_frames,
                motion_frames,
                transition_frames,
                0.14,
                selected_phase);
        } catch (...) {
            stopSimulator(capture_pid);
            throw;
        }
        stopSimulator(capture_pid);

        const int motion_case_count = selected_phase.has_value() ? 1 : 5;
        const std::size_t expected_frames = static_cast<std::size_t>(
            stand_frames + motion_case_count * (motion_frames + transition_frames));
        if (frames.size() != expected_frames
            || !std::all_of(frames.begin(), frames.end(), [](const CapturedFrame& frame) {
                   return targetsAreFinite(frame.targets);
               })) {
            throw std::runtime_error("captured command stream is incomplete or non-finite");
        }
        const std::uint64_t command_hash = commandStreamHash(frames);

        ReplayResult result{};
        for (int seed = 0; seed < perturbation_seed_count; ++seed) {
            const std::uint64_t absolute_seed = static_cast<std::uint64_t>(
                perturbation_seed_offset + seed);
            const std::uint64_t pose_seed = fixed_initial_pose ? 0 : absolute_seed;
            const std::uint64_t contact_order_seed = fixed_contact_order ? 0 : absolute_seed;
            const int replay_port = base_port + 1 + seed;
            pid_t replay_pid = launchSimulator(
                sim_exe, replay_port, contact_order_seed);
            if (replay_pid < 0) {
                throw std::runtime_error("failed to fork proximal replay simulator");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{250});
            ReplayResult seed_result{};
            try {
                seed_result = replayCommands(
                    frames,
                    replay_port,
                    replay_period_us,
                    solver_iterations,
                    absolute_tolerance,
                    relative_tolerance,
                    pose_seed,
                    perturbation_scale);
            } catch (...) {
                stopSimulator(replay_pid);
                throw;
            }
            stopSimulator(replay_pid);
            if (seed_result.recovered != 0 || seed_result.held != 0
                || seed_result.unsupported != 0 || seed_result.read_failures != 0) {
                std::cerr << "replay seed " << absolute_seed
                          << " healthy=" << seed_result.healthy
                          << " recovered=" << seed_result.recovered
                          << " held=" << seed_result.held
                          << " unsupported=" << seed_result.unsupported
                          << " read_failures=" << seed_result.read_failures
                          << " max_iterations=" << seed_result.max_iterations << '\n';
            }
            accumulateReplayResult(result, seed_result);
        }

        const std::size_t expected_replayed_frames =
            frames.size() * static_cast<std::size_t>(perturbation_seed_count);
        const bool accounting_ok = result.frames == expected_replayed_frames
            && result.telemetry_frames == expected_replayed_frames
            && result.healthy + result.recovered + result.held + result.unsupported
                == result.telemetry_frames;
        const bool gates_requested = envEnabled("HEXAPOD_EXACT_REPLAY_ENFORCE_GATES");
        const bool safety_gates_requested = envEnabled(
            "HEXAPOD_EXACT_REPLAY_ENFORCE_SAFETY_GATES");
        const bool gates_ok = !gates_requested
            || (result.recovered == 0 && result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool safety_gates_ok = !safety_gates_requested
            || (result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool passed = accounting_ok && gates_ok && safety_gates_ok;
        const std::string metrics = metricsJson(result,
                                                command_hash,
                                                frames.size(),
                                                perturbation_seed_count,
                                                perturbation_seed_offset,
                                                perturbation_scale,
                                                fixed_initial_pose,
                                                fixed_contact_order,
                                                replay_period_us,
                                                solver_iterations,
                                                absolute_tolerance,
                                                relative_tolerance);
        if (emit_metrics_json) {
            std::cout << "{\"suite\":\"physics_sim_exact_command_replay\","
                         "\"case\":\"legacy_capture_to_proximal\",\"passed\":"
                      << (passed ? "true" : "false") << ",\"metrics\":" << metrics << "}\n";
        } else {
            std::cout << "exact command replay: " << (passed ? "PASS" : "FAIL")
                      << " captured=" << frames.size() << " hash=" << std::hex << command_hash
                      << std::dec << " seeds=" << perturbation_seed_count
                      << " healthy=" << result.healthy
                      << " recovered=" << result.recovered << " held=" << result.held
                      << " unsupported=" << result.unsupported
                      << " max_iterations=" << result.max_iterations << '\n';
        }
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
#endif
}
