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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <map>
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

using IterationHistogram = std::map<std::uint16_t, std::uint64_t>;

constexpr std::array<const char*, kPhaseCount> kPhaseNames{
    "stand", "transition", "forward", "reverse", "strafe", "diagonal", "turn_in_place"};

constexpr std::array<const char*, 15> kFailureReasonNames{
    "none",
    "invalid_dt",
    "read_state",
    "non_finite_state",
    "non_finite_mass",
    "non_finite_acceleration",
    "unsupported_island",
    "solver_not_converged",
    "non_finite_impulse",
    "non_finite_velocity",
    "speed_limit",
    "non_finite_configuration",
    "write_state",
    "non_finite_energy",
    "extreme_penetration",
};

struct CapturedFrame {
    JointTargets targets{};
    ReplayPhase phase{ReplayPhase::Stand};
    bool inhibit_motion{false};
    bool walk_mode{false};
};

struct PhaseResult {
    std::uint64_t frames{0};
    std::uint64_t healthy{0};
    std::uint64_t recovered{0};
    std::uint64_t held{0};
    std::uint64_t unsupported{0};
    std::uint64_t read_failures{0};
    std::uint64_t solver_not_converged{0};
    IterationHistogram iteration_histogram{};
    std::uint16_t max_iterations{0};
    double max_ncp_dual_residual{0.0};
    double max_ncp_complementarity_residual{0.0};
    double max_contact_penetration{0.0};
    double max_body_height_error{0.0};
    double completed_delta_x_sum{0.0};
    double completed_delta_y_sum{0.0};
    double completed_body_forward_sum{0.0};
    double completed_body_lateral_sum{0.0};
    double completed_yaw_delta_sum{0.0};
    double completed_horizontal_path_sum{0.0};
    double completed_horizontal_displacement_sum{0.0};
    std::uint64_t completed_trajectories{0};
};

struct PhaseCommand {
    double vx_mps{0.0};
    double vy_mps{0.0};
    double yaw_rate_radps{0.0};
};

PhaseCommand phaseCommand(const ReplayPhase phase) {
    switch (phase) {
    case ReplayPhase::Forward:
        return {0.12, 0.0, 0.0};
    case ReplayPhase::Reverse:
        return {-0.12, 0.0, 0.0};
    case ReplayPhase::Strafe:
        return {0.0, 0.10, 0.0};
    case ReplayPhase::Diagonal:
        return {0.085, 0.085, 0.0};
    case ReplayPhase::TurnInPlace:
        return {0.0, 0.0, 0.45};
    case ReplayPhase::Stand:
    case ReplayPhase::Transition:
    case ReplayPhase::Count:
        return {};
    }
    return {};
}

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
    IterationHistogram iteration_histogram{};
    std::map<std::uint32_t, IterationHistogram> contact_count_iteration_histograms{};
    std::array<std::uint64_t, kFailureReasonNames.size()> failure_reason_histogram{};
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

bool passesBehaviorGates(const ReplayResult& result, const int replay_period_us) {
    constexpr double kMinimumCommandProgressRatio = 0.70;
    constexpr double kMaximumLateralPathFraction = 0.10;
    constexpr double kLateralAllowanceM = 0.010;
    constexpr double kMaximumTurnTranslationM = 0.050;
    constexpr std::array<ReplayPhase, 4> kTranslationPhases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
    };
    for (const ReplayPhase replay_phase : kTranslationPhases) {
        const PhaseResult& phase = result.phases[static_cast<std::size_t>(replay_phase)];
        if (phase.completed_trajectories == 0) {
            return false;
        }
        const double trajectory_count = static_cast<double>(phase.completed_trajectories);
        const PhaseCommand command = phaseCommand(replay_phase);
        const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
        const double frames_per_trajectory =
            static_cast<double>(phase.frames) / trajectory_count;
        const double commanded_translation = command_speed * frames_per_trajectory
            * static_cast<double>(replay_period_us) * 1.0e-6;
        const double body_forward = phase.completed_body_forward_sum / trajectory_count;
        const double body_lateral = phase.completed_body_lateral_sum / trajectory_count;
        const double ux = command.vx_mps / command_speed;
        const double uy = command.vy_mps / command_speed;
        const double command_progress = ux * body_forward + uy * body_lateral;
        const double command_lateral = -uy * body_forward + ux * body_lateral;
        const double horizontal_path =
            phase.completed_horizontal_path_sum / trajectory_count;
        if (command_progress < kMinimumCommandProgressRatio * commanded_translation
            || std::abs(command_lateral)
                > kMaximumLateralPathFraction * horizontal_path + kLateralAllowanceM) {
            return false;
        }
    }

    const PhaseResult& turn =
        result.phases[static_cast<std::size_t>(ReplayPhase::TurnInPlace)];
    if (turn.completed_trajectories == 0) {
        return false;
    }
    const double trajectory_count = static_cast<double>(turn.completed_trajectories);
    const double frames_per_trajectory = static_cast<double>(turn.frames) / trajectory_count;
    const double commanded_yaw = phaseCommand(ReplayPhase::TurnInPlace).yaw_rate_radps
        * frames_per_trajectory * static_cast<double>(replay_period_us) * 1.0e-6;
    const double yaw_delta = turn.completed_yaw_delta_sum / trajectory_count;
    const double horizontal_displacement =
        turn.completed_horizontal_displacement_sum / trajectory_count;
    return yaw_delta >= kMinimumCommandProgressRatio * commanded_yaw
        && horizontal_displacement <= kMaximumTurnTranslationM;
}

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

    void annotateLastFrame(const SafetyState& safety, const ControlStatus& status) {
        if (!frames_.empty()) {
            frames_.back().inhibit_motion = safety.inhibit_motion;
            frames_.back().walk_mode = status.active_mode == RobotMode::WALK;
        }
    }

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

std::uint16_t iterationPercentile(const IterationHistogram& histogram,
                                  const double percentile) {
    std::uint64_t sample_count = 0;
    for (const auto& [iterations, frames] : histogram) {
        (void)iterations;
        sample_count += frames;
    }
    if (sample_count == 0) {
        return 0;
    }
    const std::uint64_t rank = std::max<std::uint64_t>(
        1U,
        static_cast<std::uint64_t>(
            std::ceil(percentile * static_cast<double>(sample_count))));
    std::uint64_t cumulative = 0;
    for (const auto& [iterations, frames] : histogram) {
        cumulative += frames;
        if (cumulative >= rank) {
            return iterations;
        }
    }
    return histogram.rbegin()->first;
}

constexpr const char* kCommandFixtureHeader = "hexapod-exact-command-replay-v1";

void saveCommandFixture(const std::string& path,
                        const std::vector<CapturedFrame>& frames) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("failed to open command fixture for writing: " + path);
    }
    output << kCommandFixtureHeader << '\n'
           << frames.size() << '\n'
           << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const CapturedFrame& frame : frames) {
        output << static_cast<unsigned>(frame.phase) << ' '
               << (frame.inhibit_motion ? 1 : 0) << ' '
               << (frame.walk_mode ? 1 : 0);
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                output << ' ' << static_cast<double>(joint.pos_rad.value)
                       << ' ' << static_cast<double>(joint.vel_radps.value);
            }
        }
        output << '\n';
    }
    if (!output) {
        throw std::runtime_error("failed while writing command fixture: " + path);
    }
}

std::vector<CapturedFrame> loadCommandFixture(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open command fixture for reading: " + path);
    }
    std::string header;
    std::getline(input, header);
    if (header != kCommandFixtureHeader) {
        throw std::runtime_error("unsupported command fixture header: " + path);
    }
    std::size_t frame_count = 0;
    if (!(input >> frame_count) || frame_count > 1'000'000U) {
        throw std::runtime_error("invalid command fixture frame count: " + path);
    }
    std::vector<CapturedFrame> frames;
    frames.reserve(frame_count);
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        unsigned phase = 0;
        int inhibit_motion = 0;
        int walk_mode = 0;
        if (!(input >> phase >> inhibit_motion >> walk_mode)
            || phase >= static_cast<unsigned>(ReplayPhase::Count)
            || (inhibit_motion != 0 && inhibit_motion != 1)
            || (walk_mode != 0 && walk_mode != 1)) {
            throw std::runtime_error(
                "invalid command fixture metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        CapturedFrame frame{};
        frame.phase = static_cast<ReplayPhase>(phase);
        frame.inhibit_motion = inhibit_motion != 0;
        frame.walk_mode = walk_mode != 0;
        for (LegState& leg : frame.targets.leg_states) {
            for (JointState& joint : leg.joint_state) {
                double position = 0.0;
                double velocity = 0.0;
                if (!(input >> position >> velocity)
                    || !std::isfinite(position) || !std::isfinite(velocity)) {
                    throw std::runtime_error(
                        "invalid command fixture target at frame "
                        + std::to_string(frame_index) + ": " + path);
                }
                joint.pos_rad.value = position;
                joint.vel_radps.value = velocity;
            }
        }
        frames.push_back(frame);
    }
    input >> std::ws;
    if (!input.eof()) {
        throw std::runtime_error("unexpected trailing command fixture data: " + path);
    }
    return frames;
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
    bridge.annotateLastFrame(runtime.getSafetyState(), runtime.getStatus());
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
                            const physics_sim::PhysicsSolverMode solver_mode,
                            const int solver_iterations,
                            const double body_height_m,
                            const double proximal_mu,
                            const double contact_regularization,
                            const double absolute_tolerance,
                            const double relative_tolerance,
                            const std::uint64_t perturbation_seed,
                            const double perturbation_scale) {
    PhysicsSimSolverSettings solver{};
    solver.mode = solver_mode;
    solver.iterations = solver_iterations;
    solver.proximal_mu = static_cast<float>(proximal_mu);
    solver.contact_regularization = static_cast<float>(contact_regularization);
    solver.absolute_tolerance = static_cast<float>(absolute_tolerance);
    solver.relative_tolerance = static_cast<float>(relative_tolerance);
    PhysicsSimBridge bridge(
        "127.0.0.1", port, replay_period_us, solver, nullptr);
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

    struct ActivePhaseSegment {
        ReplayPhase phase{ReplayPhase::Stand};
        std::optional<Vec3> start_position{};
        std::optional<Vec3> last_position{};
        double start_yaw{0.0};
        double last_raw_yaw{0.0};
        double accumulated_yaw{0.0};
        double horizontal_path{0.0};
    };
    std::optional<ActivePhaseSegment> active_segment{};
    const auto finishSegment = [&]() {
        if (!active_segment.has_value()
            || !active_segment->start_position.has_value()
            || !active_segment->last_position.has_value()) {
            active_segment.reset();
            return;
        }
        PhaseResult& phase = result.phases[static_cast<std::size_t>(active_segment->phase)];
        const double dx = active_segment->last_position->x
            - active_segment->start_position->x;
        const double dy = active_segment->last_position->y
            - active_segment->start_position->y;
        const double c = std::cos(active_segment->start_yaw);
        const double s = std::sin(active_segment->start_yaw);
        phase.completed_delta_x_sum += dx;
        phase.completed_delta_y_sum += dy;
        phase.completed_body_forward_sum += c * dx + s * dy;
        phase.completed_body_lateral_sum += -s * dx + c * dy;
        phase.completed_yaw_delta_sum += active_segment->accumulated_yaw;
        phase.completed_horizontal_path_sum += active_segment->horizontal_path;
        phase.completed_horizontal_displacement_sum += std::hypot(dx, dy);
        ++phase.completed_trajectories;
        active_segment.reset();
    };

    for (const CapturedFrame& frame : frames) {
        if (!active_segment.has_value() || active_segment->phase != frame.phase) {
            finishSegment();
            active_segment = ActivePhaseSegment{frame.phase};
        }
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
            const double yaw = state.body_twist_state.twist_pos_rad.z;
            phase.max_body_height_error = std::max(
                phase.max_body_height_error,
                std::abs(position.z - body_height_m));
            if (!active_segment->start_position.has_value()) {
                active_segment->start_position = position;
                active_segment->start_yaw = yaw;
                active_segment->last_raw_yaw = yaw;
            } else if (active_segment->last_position.has_value()) {
                active_segment->horizontal_path += std::hypot(
                    position.x - active_segment->last_position->x,
                    position.y - active_segment->last_position->y);
                active_segment->accumulated_yaw += std::remainder(
                    yaw - active_segment->last_raw_yaw,
                    6.28318530717958647692);
                active_segment->last_raw_yaw = yaw;
            }
            active_segment->last_position = position;
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
        ++result.iteration_histogram[telemetry->iterations];
        ++result.contact_count_iteration_histograms[telemetry->contact_constraint_count]
                                                   [telemetry->iterations];
        const std::size_t failure_reason = static_cast<std::size_t>(
            telemetry->failure_reason);
        if (failure_reason < result.failure_reason_histogram.size()) {
            ++result.failure_reason_histogram[failure_reason];
        }
        result.max_contact_constraints = std::max(
            result.max_contact_constraints, telemetry->contact_constraint_count);
        result.max_warm_start_resets = std::max(
            result.max_warm_start_resets, telemetry->warm_start_reset_count);
        phase.max_iterations = std::max(phase.max_iterations, telemetry->iterations);
        ++phase.iteration_histogram[telemetry->iterations];
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
    finishSegment();
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
    for (const auto& [iterations, frames] : sample.iteration_histogram) {
        total.iteration_histogram[iterations] += frames;
    }
    for (const auto& [contact_count, histogram] :
         sample.contact_count_iteration_histograms) {
        for (const auto& [iterations, frames] : histogram) {
            total.contact_count_iteration_histograms[contact_count][iterations] += frames;
        }
    }
    for (std::size_t i = 0; i < total.failure_reason_histogram.size(); ++i) {
        total.failure_reason_histogram[i] += sample.failure_reason_histogram[i];
    }
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
        for (const auto& [iterations, frames] : in.iteration_histogram) {
            out.iteration_histogram[iterations] += frames;
        }
        out.max_iterations = std::max(out.max_iterations, in.max_iterations);
        out.max_ncp_dual_residual = std::max(
            out.max_ncp_dual_residual, in.max_ncp_dual_residual);
        out.max_ncp_complementarity_residual = std::max(
            out.max_ncp_complementarity_residual,
            in.max_ncp_complementarity_residual);
        out.max_contact_penetration = std::max(
            out.max_contact_penetration, in.max_contact_penetration);
        out.max_body_height_error = std::max(
            out.max_body_height_error, in.max_body_height_error);
        out.completed_delta_x_sum += in.completed_delta_x_sum;
        out.completed_delta_y_sum += in.completed_delta_y_sum;
        out.completed_body_forward_sum += in.completed_body_forward_sum;
        out.completed_body_lateral_sum += in.completed_body_lateral_sum;
        out.completed_yaw_delta_sum += in.completed_yaw_delta_sum;
        out.completed_horizontal_path_sum += in.completed_horizontal_path_sum;
        out.completed_horizontal_displacement_sum +=
            in.completed_horizontal_displacement_sum;
        out.completed_trajectories += in.completed_trajectories;
    }
}

std::string metricsJson(const ReplayResult& result,
                        const std::uint64_t command_hash,
                        const std::vector<CapturedFrame>& captured_frames,
                        const bool command_fixture_loaded,
                        const bool command_fixture_written,
                        const int perturbation_seed_count,
                        const int perturbation_seed_offset,
                        const double perturbation_scale,
                        const bool fixed_initial_pose,
                        const bool fixed_contact_order,
                        const bool dense_admm,
                        const bool behavior_gates_requested,
                        const std::uint64_t behavior_gate_failures,
                        const int replay_period_us,
                        const int solver_iterations,
                        const double body_height_m,
                        const double proximal_mu,
                        const double contact_regularization,
                        const double absolute_tolerance,
                        const double relative_tolerance) {
    std::ostringstream out;
    struct CapturedPhaseMetrics {
        std::array<double, 18> minimum{};
        std::array<double, 18> maximum{};
        std::array<double, 18> previous{};
        bool initialized{false};
        std::uint64_t inhibited_frames{0};
        std::uint64_t walk_mode_frames{0};
        std::uint64_t moving_target_frames{0};
        double max_target_step{0.0};
        double max_target_span{0.0};
    };
    std::array<CapturedPhaseMetrics, kPhaseCount> captured_phase_metrics{};
    for (const CapturedFrame& frame : captured_frames) {
        CapturedPhaseMetrics& metrics =
            captured_phase_metrics[static_cast<std::size_t>(frame.phase)];
        metrics.inhibited_frames += frame.inhibit_motion ? 1U : 0U;
        metrics.walk_mode_frames += frame.walk_mode ? 1U : 0U;
        std::array<double, 18> current{};
        std::size_t joint_index = 0;
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                current[joint_index++] = joint.pos_rad.value;
            }
        }
        if (!metrics.initialized) {
            metrics.minimum = current;
            metrics.maximum = current;
            metrics.previous = current;
            metrics.initialized = true;
            continue;
        }
        double frame_max_step = 0.0;
        for (std::size_t i = 0; i < current.size(); ++i) {
            metrics.minimum[i] = std::min(metrics.minimum[i], current[i]);
            metrics.maximum[i] = std::max(metrics.maximum[i], current[i]);
            frame_max_step = std::max(
                frame_max_step, std::abs(current[i] - metrics.previous[i]));
            metrics.previous[i] = current[i];
        }
        metrics.max_target_step = std::max(metrics.max_target_step, frame_max_step);
        metrics.moving_target_frames += frame_max_step > 1.0e-6 ? 1U : 0U;
    }
    for (CapturedPhaseMetrics& metrics : captured_phase_metrics) {
        if (!metrics.initialized) {
            continue;
        }
        for (std::size_t i = 0; i < metrics.minimum.size(); ++i) {
            metrics.max_target_span = std::max(
                metrics.max_target_span, metrics.maximum[i] - metrics.minimum[i]);
        }
    }
    out << std::setprecision(9)
        << "{\"captured_frames\":" << captured_frames.size()
        << ",\"replayed_frames\":" << result.frames
        << ",\"telemetry_frames\":" << result.telemetry_frames
        << ",\"command_hash\":\"" << std::hex << command_hash << std::dec << "\""
        << ",\"command_fixture_loaded\":"
        << (command_fixture_loaded ? "true" : "false")
        << ",\"command_fixture_written\":"
        << (command_fixture_written ? "true" : "false")
        << ",\"perturbation_seed_count\":" << perturbation_seed_count
        << ",\"perturbation_seed_offset\":" << perturbation_seed_offset
        << ",\"perturbation_scale\":" << perturbation_scale
        << ",\"fixed_initial_pose\":" << (fixed_initial_pose ? "true" : "false")
        << ",\"fixed_contact_order\":" << (fixed_contact_order ? "true" : "false")
        << ",\"dense_admm\":" << (dense_admm ? "true" : "false")
        << ",\"behavior_gates_requested\":"
        << (behavior_gates_requested ? "true" : "false")
        << ",\"behavior_gate_failures\":" << behavior_gate_failures
        << ",\"replay_period_us\":" << replay_period_us
        << ",\"solver_iteration_limit\":" << solver_iterations
        << ",\"commanded_body_height_m\":" << body_height_m
        << ",\"proximal_mu\":" << proximal_mu
        << ",\"contact_regularization\":" << contact_regularization
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
        << ",\"iteration_histogram\":[";
    bool first_iteration_bin = true;
    for (const auto& [iterations, frames] : result.iteration_histogram) {
        if (!first_iteration_bin) {
            out << ',';
        }
        first_iteration_bin = false;
        out << "{\"iterations\":" << iterations << ",\"frames\":" << frames << '}';
    }
    out << "],\"contact_count_iteration_profiles\":[";
    bool first_contact_profile = true;
    for (const auto& [contact_count, histogram] :
         result.contact_count_iteration_histograms) {
        if (!first_contact_profile) {
            out << ',';
        }
        first_contact_profile = false;
        std::uint64_t frame_count = 0;
        for (const auto& [iterations, frames] : histogram) {
            (void)iterations;
            frame_count += frames;
        }
        out << "{\"contacts\":" << contact_count
            << ",\"frames\":" << frame_count
            << ",\"p50_iterations\":" << iterationPercentile(histogram, 0.50)
            << ",\"p90_iterations\":" << iterationPercentile(histogram, 0.90)
            << ",\"p99_iterations\":" << iterationPercentile(histogram, 0.99)
            << ",\"max_iterations\":" << histogram.rbegin()->first << '}';
    }
    out << "],\"failure_reason_histogram\":[";
    bool first_failure_reason = true;
    for (std::size_t i = 1; i < result.failure_reason_histogram.size(); ++i) {
        if (result.failure_reason_histogram[i] == 0) {
            continue;
        }
        if (!first_failure_reason) {
            out << ',';
        }
        first_failure_reason = false;
        out << "{\"reason\":\"" << kFailureReasonNames[i]
            << "\",\"frames\":" << result.failure_reason_histogram[i] << '}';
    }
    out << ']'
        << ",\"phases\":[";
    for (std::size_t i = 0; i < result.phases.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        const PhaseResult& phase = result.phases[i];
        const CapturedPhaseMetrics& captured = captured_phase_metrics[i];
        const double dx = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_x_sum
                / static_cast<double>(phase.completed_trajectories);
        const double dy = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_y_sum
                / static_cast<double>(phase.completed_trajectories);
        const double body_forward = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_body_forward_sum
                / static_cast<double>(phase.completed_trajectories);
        const double body_lateral = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_body_lateral_sum
                / static_cast<double>(phase.completed_trajectories);
        const double yaw_delta = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_yaw_delta_sum
                / static_cast<double>(phase.completed_trajectories);
        const double horizontal_path = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_horizontal_path_sum
                / static_cast<double>(phase.completed_trajectories);
        const double horizontal_displacement = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_horizontal_displacement_sum
                / static_cast<double>(phase.completed_trajectories);
        const PhaseCommand command = phaseCommand(static_cast<ReplayPhase>(i));
        const double frames_per_trajectory = phase.completed_trajectories == 0 ? 0.0
            : static_cast<double>(phase.frames)
                / static_cast<double>(phase.completed_trajectories);
        const double commanded_translation = std::hypot(command.vx_mps, command.vy_mps)
            * frames_per_trajectory * static_cast<double>(replay_period_us) * 1.0e-6;
        const double commanded_yaw = command.yaw_rate_radps
            * frames_per_trajectory * static_cast<double>(replay_period_us) * 1.0e-6;
        double command_progress = 0.0;
        double command_lateral = 0.0;
        const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
        if (command_speed > 0.0) {
            const double ux = command.vx_mps / command_speed;
            const double uy = command.vy_mps / command_speed;
            command_progress = ux * body_forward + uy * body_lateral;
            command_lateral = -uy * body_forward + ux * body_lateral;
        }
        out << "{\"name\":\"" << kPhaseNames[i]
            << "\",\"frames\":" << phase.frames
            << ",\"healthy\":" << phase.healthy
            << ",\"recovered\":" << phase.recovered
            << ",\"held\":" << phase.held
            << ",\"unsupported\":" << phase.unsupported
            << ",\"read_failures\":" << phase.read_failures
            << ",\"captured_inhibited_frames\":" << captured.inhibited_frames
            << ",\"captured_walk_mode_frames\":" << captured.walk_mode_frames
            << ",\"captured_moving_target_frames\":" << captured.moving_target_frames
            << ",\"captured_max_target_step_rad\":" << captured.max_target_step
            << ",\"captured_max_target_span_rad\":" << captured.max_target_span
            << ",\"solver_not_converged\":" << phase.solver_not_converged
            << ",\"p50_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.50)
            << ",\"p90_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.90)
            << ",\"p99_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.99)
            << ",\"max_iterations\":" << phase.max_iterations
            << ",\"max_ncp_dual_residual\":" << phase.max_ncp_dual_residual
            << ",\"max_ncp_complementarity_residual\":"
            << phase.max_ncp_complementarity_residual
            << ",\"max_contact_penetration_m\":" << phase.max_contact_penetration
            << ",\"max_body_height_error_m\":" << phase.max_body_height_error
            << ",\"valid_delta_x_m\":" << dx
            << ",\"valid_delta_y_m\":" << dy
            << ",\"body_forward_delta_m\":" << body_forward
            << ",\"body_lateral_delta_m\":" << body_lateral
            << ",\"command_progress_m\":" << command_progress
            << ",\"command_lateral_m\":" << command_lateral
            << ",\"commanded_translation_m\":" << commanded_translation
            << ",\"yaw_delta_rad\":" << yaw_delta
            << ",\"commanded_yaw_rad\":" << commanded_yaw
            << ",\"horizontal_path_m\":" << horizontal_path
            << ",\"horizontal_displacement_m\":" << horizontal_displacement
            << ",\"completed_trajectories\":" << phase.completed_trajectories << '}';
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
        const double body_height_m = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_BODY_HEIGHT_M", 0.14);
        const double proximal_mu = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PROXIMAL_MU", 1.0e-6);
        const double contact_regularization = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_CONTACT_REGULARIZATION", 1.0e-10);
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
        const bool dense_admm = envEnabled("HEXAPOD_PINOCCHIO_DENSE_ADMM");
        const char* command_fixture_input =
            std::getenv("HEXAPOD_EXACT_REPLAY_COMMANDS_IN");
        const char* command_fixture_output =
            std::getenv("HEXAPOD_EXACT_REPLAY_COMMANDS_OUT");
        const bool command_fixture_loaded = command_fixture_input != nullptr
            && command_fixture_input[0] != '\0';
        const bool command_fixture_written = command_fixture_output != nullptr
            && command_fixture_output[0] != '\0';
        const physics_sim::PhysicsSolverMode replay_solver_mode =
            envEnabled("HEXAPOD_EXACT_REPLAY_LEGACY")
                ? physics_sim::PhysicsSolverMode::LegacyPgs
                : physics_sim::PhysicsSolverMode::PinocchioProximal;
        if (perturbation_seed_count > 1000) {
            throw std::runtime_error(
                "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS must be at most 1000");
        }
        const std::optional<ReplayPhase> selected_phase = selectedMotionPhase();
        const int base_port = 23500 + (static_cast<int>(::getpid()) % 4000);

        std::vector<CapturedFrame> frames{};
        if (command_fixture_loaded) {
            frames = loadCommandFixture(command_fixture_input);
        } else {
            pid_t capture_pid = launchSimulator(sim_exe, base_port);
            if (capture_pid < 0) {
                throw std::runtime_error("failed to fork legacy capture simulator");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{250});
            try {
                frames = captureCommands(
                    harness,
                    base_port,
                    stand_frames,
                    motion_frames,
                    transition_frames,
                    body_height_m,
                    selected_phase);
            } catch (...) {
                stopSimulator(capture_pid);
                throw;
            }
            stopSimulator(capture_pid);
        }

        const int motion_case_count = selected_phase.has_value() ? 1 : 5;
        const std::size_t expected_frames = static_cast<std::size_t>(
            stand_frames + motion_case_count * (motion_frames + transition_frames));
        if (frames.size() != expected_frames
            || !std::all_of(frames.begin(), frames.end(), [](const CapturedFrame& frame) {
                   return targetsAreFinite(frame.targets);
               })) {
            throw std::runtime_error("captured command stream is incomplete or non-finite");
        }
        if (command_fixture_written) {
            saveCommandFixture(command_fixture_output, frames);
        }
        const std::uint64_t command_hash = commandStreamHash(frames);

        ReplayResult result{};
        std::uint64_t behavior_gate_failures = 0;
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
                    replay_solver_mode,
                    solver_iterations,
                    body_height_m,
                    proximal_mu,
                    contact_regularization,
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
            if (!passesBehaviorGates(seed_result, replay_period_us)) {
                ++behavior_gate_failures;
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
        const bool behavior_gates_requested = envEnabled(
            "HEXAPOD_EXACT_REPLAY_ENFORCE_BEHAVIOR_GATES");
        const bool gates_ok = !gates_requested
            || (result.recovered == 0 && result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool safety_gates_ok = !safety_gates_requested
            || (result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool behavior_gates_ok = !behavior_gates_requested
            || behavior_gate_failures == 0;
        const bool passed = accounting_ok && gates_ok && safety_gates_ok
            && behavior_gates_ok;
        const std::string metrics = metricsJson(result,
                                                command_hash,
                                                frames,
                                                command_fixture_loaded,
                                                command_fixture_written,
                                                perturbation_seed_count,
                                                perturbation_seed_offset,
                                                perturbation_scale,
                                                fixed_initial_pose,
                                                fixed_contact_order,
                                                dense_admm,
                                                behavior_gates_requested,
                                                behavior_gate_failures,
                                                replay_period_us,
                                                solver_iterations,
                                                body_height_m,
                                                proximal_mu,
                                                contact_regularization,
                                                absolute_tolerance,
                                                relative_tolerance);
        if (emit_metrics_json) {
            std::cout << "{\"suite\":\"physics_sim_exact_command_replay\","
                         "\"case\":\"deterministic_capture_replay\",\"solver_mode\":\""
                      << (replay_solver_mode == physics_sim::PhysicsSolverMode::LegacyPgs
                              ? "legacy-pgs"
                              : "pinocchio-proximal")
                      << "\",\"passed\":" << (passed ? "true" : "false")
                      << ",\"metrics\":" << metrics << "}\n";
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
