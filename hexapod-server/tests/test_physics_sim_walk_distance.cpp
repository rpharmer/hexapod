#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

void runControlLoopStep(RobotRuntime& runtime, const MotionIntent& motion) {
    runtime.setMotionIntent(motion);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

struct MotionRunResult {
    Vec3 start_position{};
    Vec3 end_position{};
    double start_yaw_rad{0.0};
    double end_yaw_rad{0.0};
    double walk_path_length_m{0.0};
    double max_lateral_deviation_m{0.0};
    double average_horizontal_speed_mps{0.0};
    double peak_horizontal_speed_mps{0.0};
    double average_yaw_rate_radps{0.0};
    double peak_yaw_rate_radps{0.0};
    ControlStatus final_status{};
};

double wrapAngleDiff(double start, double end) {
    return std::atan2(std::sin(end - start), std::cos(end - start));
}

template <typename MotionT>
MotionRunResult runMotionSequence(RobotRuntime& runtime,
                                  CapturingPhysicsSimBridge& bridge,
                                  const ScenarioMotionIntent& stand_motion,
                                  const MotionT& motion) {
    constexpr int kStandWarmupSteps = 100;
    constexpr int kWalkSteps = 600;

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!bridge.last_state().has_value()) {
        throw std::runtime_error("bridge never produced an initial state");
    }

    MotionRunResult result{};
    result.start_position = positionFromState(bridge.last_state().value());
    result.start_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    Vec3 previous_position = result.start_position;
    double horizontal_speed_sum = 0.0;
    double yaw_speed_sum = 0.0;
    std::vector<Vec3> walk_samples{};
    walk_samples.reserve(static_cast<std::size_t>(kWalkSteps));

    for (int i = 0; i < kWalkSteps; ++i) {
        runControlLoopStep(runtime, motion);

        if (!bridge.last_state().has_value()) {
            throw std::runtime_error("bridge lost state during walk sequence");
        }

        const RobotState& state = bridge.last_state().value();
        const Vec3 current_position = positionFromState(state);
        walk_samples.push_back(current_position);
        result.walk_path_length_m += std::hypot(current_position.x - previous_position.x,
                                                current_position.y - previous_position.y);
        previous_position = current_position;
        const double speed_xy = std::hypot(
            state.body_twist_state.body_trans_mps.x,
            state.body_twist_state.body_trans_mps.y);
        horizontal_speed_sum += speed_xy;
        result.peak_horizontal_speed_mps = std::max(result.peak_horizontal_speed_mps, speed_xy);
        const double yaw_rate = std::abs(state.body_twist_state.twist_vel_radps.z);
        yaw_speed_sum += yaw_rate;
        result.peak_yaw_rate_radps = std::max(result.peak_yaw_rate_radps, yaw_rate);
        result.final_status = runtime.getStatus();
    }

    result.end_position = positionFromState(bridge.last_state().value());
    result.end_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    result.average_horizontal_speed_mps = horizontal_speed_sum / static_cast<double>(kWalkSteps);
    result.average_yaw_rate_radps = yaw_speed_sum / static_cast<double>(kWalkSteps);

    const Vec3 travel = result.end_position - result.start_position;
    const double travel_len = std::hypot(travel.x, travel.y);
    if (travel_len > 1e-9) {
        const double denom = travel_len;
        for (const Vec3& sample : walk_samples) {
            const Vec3 offset = sample - result.start_position;
            const double deviation = std::abs(travel.x * offset.y - travel.y * offset.x) / denom;
            result.max_lateral_deviation_m = std::max(result.max_lateral_deviation_m, deviation);
        }
    }

    return result;
}

bool checkWalkCase(const std::string& label,
                   RobotRuntime& runtime,
                   CapturingPhysicsSimBridge& bridge,
                   const ScenarioMotionIntent& stand_motion,
                   const ScenarioMotionIntent& walk_motion) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, walk_motion);
    } catch (const std::exception& ex) {
        return expect(false, label + ": " + ex.what());
    }

    const RobotState fused_snapshot = runtime.estimatedSnapshot();
    if (!expect(fused_snapshot.has_fusion_diagnostics,
                label + ": estimator should publish fusion diagnostics during live sim walking")) {
        return false;
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;

    constexpr double kMinPathLengthM = 0.08;
    constexpr double kMinPeakHorizontalSpeedMps = 0.02;
    constexpr double kMinAverageSpeedRatio = 0.05;
    constexpr double kMaxAverageSpeedRatio = 0.25;

    if (!expect(result.walk_path_length_m >= kMinPathLengthM,
                label + ": walk should accumulate measurable horizontal travel")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " peak_speed=" << result.peak_horizontal_speed_mps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps >= commanded_speed * kMinAverageSpeedRatio,
                label + ": average projected speed should stay above the lower band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps <= commanded_speed * kMaxAverageSpeedRatio,
                label + ": average projected speed should stay below the upper band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.peak_horizontal_speed_mps >= kMinPeakHorizontalSpeedMps,
                label + ": walk should produce a non-trivial horizontal body speed")) {
        return false;
    }

    if (!expect(result.final_status.active_mode == RobotMode::WALK,
                label + ": runtime should remain in WALK mode while the motion command is active")) {
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " horiz=" << horizontal_distance
              << " path=" << result.walk_path_length_m
              << " avg_speed=" << result.average_horizontal_speed_mps
              << " ratio=" << average_ratio
              << " peak_speed=" << result.peak_horizontal_speed_mps << '\n';
    return true;
}

bool checkStraightWalkCase(const std::string& label,
                           RobotRuntime& runtime,
                           CapturingPhysicsSimBridge& bridge,
                           const ScenarioMotionIntent& stand_motion,
                           const ScenarioMotionIntent& walk_motion) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, walk_motion);
    } catch (const std::exception& ex) {
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;

    constexpr double kMinPathLengthM = 0.08;
    constexpr double kMaxLateralDeviationM = 0.03;
    constexpr double kMinPeakHorizontalSpeedMps = 0.02;
    constexpr double kMinAverageSpeedRatio = 0.05;
    constexpr double kMaxAverageSpeedRatio = 0.25;

    if (!expect(result.walk_path_length_m >= kMinPathLengthM,
                label + ": straight walk should accumulate measurable horizontal travel")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " peak_speed=" << result.peak_horizontal_speed_mps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        return false;
    }

    if (!expect(result.max_lateral_deviation_m <= kMaxLateralDeviationM,
                label + ": straight walk should stay close to its start-to-finish path")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.max_lateral_deviation_m <= result.walk_path_length_m * 0.35,
                label + ": straight walk should not crab away from its travel line")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps >= commanded_speed * kMinAverageSpeedRatio,
                label + ": average projected speed should stay above the lower band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps <= commanded_speed * kMaxAverageSpeedRatio,
                label + ": average projected speed should stay below the upper band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.peak_horizontal_speed_mps >= kMinPeakHorizontalSpeedMps,
                label + ": straight walk should produce a non-trivial horizontal body speed")) {
        return false;
    }

    if (!expect(result.final_status.active_mode == RobotMode::WALK,
                label + ": runtime should remain in WALK mode while the motion command is active")) {
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " lateral_deviation=" << result.max_lateral_deviation_m
              << " path=" << result.walk_path_length_m
              << " avg_speed=" << result.average_horizontal_speed_mps
              << " ratio=" << average_ratio
              << " peak_speed=" << result.peak_horizontal_speed_mps << '\n';
    return true;
}

bool checkTurnCase(const std::string& label,
                   RobotRuntime& runtime,
                   CapturingPhysicsSimBridge& bridge,
                   const ScenarioMotionIntent& stand_motion,
                   const MotionIntent& turn_motion) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, turn_motion);
    } catch (const std::exception& ex) {
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double yaw_delta = wrapAngleDiff(result.start_yaw_rad, result.end_yaw_rad);
    const double commanded_yaw_rate = std::abs(turn_motion.twist.twist_vel_radps.z);
    const double average_ratio =
        commanded_yaw_rate > 0.0 ? (result.average_yaw_rate_radps / commanded_yaw_rate) : 0.0;

    constexpr double kMaxPathLengthM = 0.60;
    constexpr double kMaxNetHorizontalDistanceM = 0.12;
    constexpr double kMinPeakYawRateRadps = 0.02;
    constexpr double kMinAverageYawRateRatio = 0.05;
    constexpr double kMaxAverageYawRateRatio = 2.25;
    constexpr double kMinYawDeltaRad = 0.05;

    if (!expect(horizontal_distance <= kMaxNetHorizontalDistanceM,
                label + ": turn-in-place should keep net horizontal drift bounded")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio
                  << " peak_yaw_rate=" << result.peak_yaw_rate_radps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        return false;
    }

    if (!expect(result.walk_path_length_m <= kMaxPathLengthM,
                label + ": turn-in-place should keep path length bounded")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio
                  << " peak_yaw_rate=" << result.peak_yaw_rate_radps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        return false;
    }

    if (!expect(std::abs(yaw_delta) >= kMinYawDeltaRad,
                label + ": turn-in-place should produce measurable yaw change")) {
        std::cerr << label << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.average_yaw_rate_radps >= commanded_yaw_rate * kMinAverageYawRateRatio,
                label + ": average yaw speed should stay above the lower band")) {
        std::cerr << label << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.average_yaw_rate_radps <= commanded_yaw_rate * kMaxAverageYawRateRatio,
                label + ": average yaw speed should stay below the upper band")) {
        std::cerr << label << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        return false;
    }

    if (!expect(result.peak_yaw_rate_radps >= kMinPeakYawRateRadps,
                label + ": turn-in-place should produce a non-trivial yaw rate")) {
        return false;
    }

    if (!expect(result.final_status.active_mode == RobotMode::WALK,
                label + ": runtime should remain in WALK mode while the turn command is active")) {
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " horiz=" << horizontal_distance
              << " path=" << result.walk_path_length_m
              << " yaw_delta=" << yaw_delta
              << " avg_yaw_rate=" << result.average_yaw_rate_radps
              << " ratio=" << average_ratio
              << " peak_yaw_rate=" << result.peak_yaw_rate_radps << '\n';
    return true;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_physics_sim_walk_distance: Linux-only\n";
    return EXIT_SUCCESS;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_walk_distance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int kPort = 22000 + (static_cast<int>(::getpid()) % 6000);
    const int kBusLoopPeriodUs = 20000;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(kPort);
        ::execl(sim_exe,
                sim_exe,
                "--serve",
                "--serve-port",
                port_str.c_str(),
                nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>("127.0.0.1", kPort, kBusLoopPeriodUs, 24);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg);
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{
        true,
        RobotMode::STAND,
        GaitType::TRIPOD,
        0.06,
        0.0,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_forward_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        0.06,
        0.20,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_reverse_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        0.06,
        0.20,
        kPi,
        0.0};
    MotionIntent turn_in_place_motion = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.06);
    turn_in_place_motion.speed_mps = LinearRateMps{0.0};
    turn_in_place_motion.heading_rad = AngleRad{0.0};
    turn_in_place_motion.twist.twist_vel_radps = Vec3{0.0, 0.0, 0.45};
    turn_in_place_motion.twist.twist_pos_rad = Vec3{0.0, 0.0, 0.0};

    if (!checkWalkCase("forward_walk", runtime, *bridge_ptr, stand_motion, walk_forward_motion)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkWalkCase("reverse_walk", runtime, *bridge_ptr, stand_motion, walk_reverse_motion)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkStraightWalkCase("straight_walk", runtime, *bridge_ptr, stand_motion, walk_forward_motion)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkTurnCase("turn_in_place", runtime, *bridge_ptr, stand_motion, turn_in_place_motion)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    return 0;
#endif
}
