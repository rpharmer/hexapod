#include "control_config.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_metrics_emit.hpp"
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
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

class CapturingBridge final : public IHardwareBridge {
public:
    CapturingBridge(std::string host,
                    int port,
                    int busLoopPeriodUs,
                    PhysicsSimSolverSettings solverSettings)
        : inner_(std::move(host), port, busLoopPeriodUs, solverSettings, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        telemetry_ = inner_.latestSolverTelemetry();
        if (ok) {
            state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override { return inner_.write(in); }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    bool supportsAutomaticBusTimeoutRecovery() const override {
        return inner_.supportsAutomaticBusTimeoutRecovery();
    }

    bool latestSampleHealthyForAutomaticRecovery() const override {
        return inner_.latestSampleHealthyForAutomaticRecovery();
    }

    const std::optional<RobotState>& state() const { return state_; }
    const std::optional<PhysicsSimSolverTelemetry>& telemetry() const { return telemetry_; }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> state_{};
    std::optional<PhysicsSimSolverTelemetry> telemetry_{};
};

double positiveEnvDouble(const char* key, double fallback) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end != value && *end == '\0' && std::isfinite(parsed) && parsed > 0.0
        ? parsed
        : fallback;
}

int positiveEnvInt(const char* key, int fallback) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    return end != value && *end == '\0' && parsed > 0
        ? static_cast<int>(parsed)
        : fallback;
}

void runControlLoopStep(RobotRuntime& runtime,
                        const ScenarioMotionIntent& stand,
                        TimePointUs commandTime) {
    MotionIntent intent = makeMotionIntent(stand);
    intent.timestamp_us = commandTime;
    intent.sample_id = 0;
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

BodyPose bodyPoseFromState(const RobotState& state) {
    BodyPose pose{};
    pose.position = state.body_twist_state.body_trans_m;
    pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};
    return pose;
}

struct Metrics {
    double maxBodyHeightErrorM{0.0};
    double stanceFootDriftSquaredSum{0.0};
    std::uint64_t stanceFootDriftSamples{0};
    std::uint64_t healthySteps{0};
    std::uint64_t recoveredSteps{0};
    std::uint64_t heldSteps{0};
    std::uint64_t unsupportedSteps{0};
    std::uint64_t nonConvergedSteps{0};
    std::uint64_t rollbackStart{0};
    std::uint64_t rollbackEnd{0};
    int maxIterations{0};
    std::vector<int> iterations{};
    double peakLinearSpeed{0.0};
    double peakAngularSpeed{0.0};
};

int percentile99(std::vector<int> values) {
    if (values.empty()) {
        return 0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(
        values.size() - 1U,
        (99U * values.size() + 99U) / 100U - 1U);
    return values[index];
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    (void)argc;
    (void)argv;
    std::cout << "skip test_physics_sim_proximal_stand_acceptance (Linux-only)\n";
    return 0;
#else
    bool emitMetricsJson = false;
    const char* simExe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emitMetricsJson, simExe);
    if (simExe == nullptr || simExe[0] == '\0') {
        std::cout << "skip test_physics_sim_proximal_stand_acceptance "
                     "(pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings(true);
    PhysicsSimSolverSettings solver{};
    solver.mode = physics_sim::PhysicsSolverMode::PinocchioProximal;
    solver.iterations = positiveEnvInt("HEXAPOD_PROXIMAL_STAND_ITERATIONS", 50);
    solver.proximal_mu = static_cast<float>(
        positiveEnvDouble("HEXAPOD_PROXIMAL_STAND_MU", 1.0e-6));
    solver.absolute_tolerance = static_cast<float>(
        positiveEnvDouble("HEXAPOD_PROXIMAL_STAND_ABSOLUTE_TOLERANCE", 1.0e-8));
    solver.relative_tolerance = static_cast<float>(
        positiveEnvDouble("HEXAPOD_PROXIMAL_STAND_RELATIVE_TOLERANCE", 1.0e-6));
    solver.contact_regularization = static_cast<float>(
        positiveEnvDouble("HEXAPOD_PROXIMAL_STAND_CONTACT_REGULARIZATION", 1.0e-10));

    const double commandedHeightM = positiveEnvDouble(
        "HEXAPOD_PROXIMAL_STAND_BODY_HEIGHT_M", 0.14);
    const double durationS = positiveEnvDouble(
        "HEXAPOD_PROXIMAL_STAND_DURATION_S", 60.0);
    const double warmupS = positiveEnvDouble(
        "HEXAPOD_PROXIMAL_STAND_WARMUP_S", 2.0);
    const int port = 25000 + (static_cast<int>(::getpid()) % 5000);
    const pid_t child = ::fork();
    if (child < 0) {
        std::perror("fork");
        return 2;
    }
    if (child == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string portText = std::to_string(port);
        ::execl(simExe, simExe, "--serve", "--serve-port", portText.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }
    const auto stopChild = [&]() {
        ::kill(child, SIGTERM);
        ::waitpid(child, nullptr, 0);
    };

    std::this_thread::sleep_for(std::chrono::milliseconds{250});
    auto bridge = std::make_unique<CapturingBridge>(
        "127.0.0.1", port, harness.bus_loop_period_us, solver);
    CapturingBridge* capture = bridge.get();
    control_config::ControlConfig config = harness.control_cfg;
    config.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    config.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, config);
    if (!runtime.init()) {
        stopChild();
        std::cerr << "FAIL: proximal standing runtime did not initialize\n";
        return 1;
    }

    const ScenarioMotionIntent stand{
        true, RobotMode::STAND, GaitType::TRIPOD, commandedHeightM, 0.0, 0.0, 0.0};
    const int warmupSteps = std::max(
        1, static_cast<int>(std::lround(warmupS * 1.0e6 / harness.bus_loop_period_us)));
    const int measureSteps = std::max(
        1, static_cast<int>(std::lround(durationS * 1.0e6 / harness.bus_loop_period_us)));
    TimePointUs commandTime{now_us().value + 3'600'000'000ULL};
    const auto step = [&]() {
        commandTime.value += static_cast<std::uint64_t>(harness.bus_loop_period_us);
        runControlLoopStep(runtime, stand, commandTime);
    };
    for (int i = 0; i < warmupSteps; ++i) {
        step();
    }

    Metrics metrics{};
    if (capture->telemetry().has_value()) {
        metrics.rollbackStart = capture->telemetry()->rollback_count;
    }
    LegFK fk{};
    const HexapodGeometry geometry = geometry_config::activeHexapodGeometry();
    std::array<std::optional<Vec3>, kNumLegs> footReferences{};
    for (int i = 0; i < measureSteps; ++i) {
        step();
        if (!capture->state().has_value() || !capture->telemetry().has_value()) {
            continue;
        }
        const RobotState& state = *capture->state();
        const PhysicsSimSolverTelemetry& telemetry = *capture->telemetry();
        metrics.maxBodyHeightErrorM = std::max(
            metrics.maxBodyHeightErrorM,
            std::abs(state.body_twist_state.body_trans_m.z - commandedHeightM));
        const BodyPose pose = bodyPoseFromState(state);
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (!state.foot_contacts[leg]) {
                continue;
            }
            const Vec3 foot = fk.footInWorldFrame(
                state.leg_states[leg], pose, geometry.legGeometry[leg]).pos_body_m;
            if (!footReferences[leg].has_value()) {
                footReferences[leg] = foot;
                continue;
            }
            const Vec3 delta = foot - *footReferences[leg];
            metrics.stanceFootDriftSquaredSum += delta.x * delta.x
                + delta.y * delta.y + delta.z * delta.z;
            ++metrics.stanceFootDriftSamples;
        }
        switch (telemetry.status) {
            case physics_sim::SolverStatus::Healthy:
                ++metrics.healthySteps;
                break;
            case physics_sim::SolverStatus::RecoveredRetry:
                ++metrics.recoveredSteps;
                break;
            case physics_sim::SolverStatus::HeldLastGood:
                ++metrics.heldSteps;
                break;
            case physics_sim::SolverStatus::UnsupportedIsland:
                ++metrics.unsupportedSteps;
                break;
        }
        if (telemetry.failure_reason == physics_sim::SolverFailureReason::SolverNotConverged) {
            ++metrics.nonConvergedSteps;
        }
        metrics.maxIterations = std::max(
            metrics.maxIterations, static_cast<int>(telemetry.iterations));
        metrics.iterations.push_back(static_cast<int>(telemetry.iterations));
        metrics.peakLinearSpeed = std::max(
            metrics.peakLinearSpeed,
            static_cast<double>(telemetry.preintegration_linear_speed));
        metrics.peakAngularSpeed = std::max(
            metrics.peakAngularSpeed,
            static_cast<double>(telemetry.preintegration_angular_speed));
        metrics.rollbackEnd = telemetry.rollback_count;
    }
    const ControlStatus status = runtime.getStatus();
    stopChild();

    const double footDriftRmsM = metrics.stanceFootDriftSamples == 0
        ? std::numeric_limits<double>::infinity()
        : std::sqrt(metrics.stanceFootDriftSquaredSum
                    / static_cast<double>(metrics.stanceFootDriftSamples));
    const std::uint64_t rollbacks = metrics.rollbackEnd - metrics.rollbackStart;
    const int p99Iterations = percentile99(metrics.iterations);
    const bool passed = metrics.heldSteps == 0
        && metrics.unsupportedSteps == 0
        && metrics.nonConvergedSteps == 0
        && rollbacks == 0
        && metrics.maxBodyHeightErrorM <= 0.010
        && footDriftRmsM <= 0.003
        && p99Iterations <= 20
        && metrics.maxIterations < solver.iterations
        && metrics.peakLinearSpeed <= 2.0
        && metrics.peakAngularSpeed <= 10.0
        && status.active_mode == RobotMode::STAND
        && status.active_fault == FaultCode::NONE;

    std::cout << "proximal_stand_acceptance passed=" << (passed ? 1 : 0)
              << " duration_s=" << durationS
              << " measured_steps=" << metrics.iterations.size()
              << " healthy=" << metrics.healthySteps
              << " recovered=" << metrics.recoveredSteps
              << " held=" << metrics.heldSteps
              << " unsupported=" << metrics.unsupportedSteps
              << " non_converged=" << metrics.nonConvergedSteps
              << " rollbacks=" << rollbacks
              << " max_height_error_m=" << metrics.maxBodyHeightErrorM
              << " stance_foot_drift_rms_m=" << footDriftRmsM
              << " p99_iterations=" << p99Iterations
              << " max_iterations=" << metrics.maxIterations
              << " peak_linear_speed_mps=" << metrics.peakLinearSpeed
              << " peak_angular_speed_radps=" << metrics.peakAngularSpeed
              << " final_mode=" << static_cast<int>(status.active_mode)
              << " final_fault=" << static_cast<int>(status.active_fault)
              << '\n';
    if (emitMetricsJson) {
        std::ostringstream limits;
        limits << "{\"duration_s\":" << durationS
               << ",\"commanded_body_height_m\":" << commandedHeightM
               << ",\"max_body_height_error_m\":0.01"
               << ",\"max_stance_foot_drift_rms_m\":0.003"
               << ",\"max_p99_iterations\":20"
               << ",\"max_linear_speed_mps\":2.0"
               << ",\"max_angular_speed_radps\":10.0"
               << ",\"require_zero_recovered_steps\":true"
               << ",\"require_zero_held_steps\":true"
               << ",\"require_zero_unsupported_steps\":true"
               << ",\"require_zero_non_converged_steps\":true"
               << ",\"require_zero_rollbacks\":true}";
        std::ostringstream metricsJson;
        metricsJson << "{\"measured_steps\":" << metrics.iterations.size()
                << ",\"healthy_steps\":" << metrics.healthySteps
                << ",\"recovered_steps\":" << metrics.recoveredSteps
                << ",\"held_steps\":" << metrics.heldSteps
                << ",\"unsupported_steps\":" << metrics.unsupportedSteps
                << ",\"non_converged_steps\":" << metrics.nonConvergedSteps
                << ",\"rollbacks\":" << rollbacks
                << ",\"max_body_height_error_m\":" << metrics.maxBodyHeightErrorM
                << ",\"stance_foot_drift_rms_m\":" << footDriftRmsM
                << ",\"p99_iterations\":" << p99Iterations
                << ",\"max_iterations\":" << metrics.maxIterations
                << ",\"peak_linear_speed_mps\":" << metrics.peakLinearSpeed
                << ",\"peak_angular_speed_radps\":" << metrics.peakAngularSpeed
                << ",\"final_mode\":" << static_cast<int>(status.active_mode)
                << ",\"final_fault\":" << static_cast<int>(status.active_fault)
                << '}';
        physics_sim_metrics::emitLine(
            "physics_sim_proximal_stand_acceptance",
            "production_height_stand",
            passed,
            limits.str(),
            metricsJson.str());
    }
    return passed ? 0 : 1;
#endif
}
