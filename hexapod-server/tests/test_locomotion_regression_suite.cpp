#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "replay_logger.hpp"
#include "locomotion_metrics.hpp"
#include "locomotion_motion_sequence.hpp"
#include "replay_json.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "telemetry_json.hpp"
#include "test_limits_manifest.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

using namespace locomotion_test;

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

std::string gaitName(const GaitType gait) {
    switch (gait) {
    case GaitType::TRIPOD:
        return "TRIPOD";
    case GaitType::RIPPLE:
        return "RIPPLE";
    case GaitType::WAVE:
        return "WAVE";
    case GaitType::CRAWL:
        return "CRAWL";
    case GaitType::TURN_IN_PLACE:
        return "TURN_IN_PLACE";
    }
    return "UNKNOWN";
}

struct PhysicsSimProcess {
    PhysicsSimProcess(std::string exe_path, int port)
        : exe_path_(std::move(exe_path)), port_(port) {}

    ~PhysicsSimProcess() {
        stop();
    }

    bool start() {
#if !defined(__linux__)
        return false;
#else
        pid_ = ::fork();
        if (pid_ < 0) {
            std::cerr << "fork failed\n";
            return false;
        }
        if (pid_ == 0) {
            physics_sim_test_utils::quietChildProcessStdIo();
            const std::string port_str = std::to_string(port_);
            ::execl(exe_path_.c_str(), exe_path_.c_str(), "--serve", "--serve-port", port_str.c_str(), nullptr);
            std::perror("execl");
            _exit(127);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
        return true;
#endif
    }

    void stop() {
#if defined(__linux__)
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            ::waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
#endif
    }

private:
    std::string exe_path_{};
    int port_{0};
#if defined(__linux__)
    pid_t pid_{-1};
#endif
};

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              int physics_solver_iterations)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override { return inner_.write(in); }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const { return last_state_; }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
};

class CollectingReplayLogger final : public replay::IReplayLogger {
public:
    explicit CollectingReplayLogger(std::string file_path)
        : file_path_(std::move(file_path)) {}

    void write(const replay_json::ReplayTelemetryRecord& record) override {
        records.push_back(record);
    }

    void flush() const {
        if (file_path_.empty()) {
            return;
        }
        std::filesystem::create_directories(std::filesystem::path(file_path_).parent_path());
        std::ofstream out(file_path_, std::ios::trunc);
        for (const auto& record : records) {
            out << replay_json::serializeReplayTelemetryRecord(record) << '\n';
        }
    }

    std::string file_path_{};
    std::vector<replay_json::ReplayTelemetryRecord> records{};
};

struct CaseResult {
    std::string name{};
    std::string description{};
    bool passed{false};
    std::string failure_reason{};
    std::filesystem::path replay_path{};
    std::filesystem::path geometry_path{};
    std::filesystem::path summary_path{};
    std::filesystem::path metrics_path{};
    std::vector<MotionSample> samples{};
    LocomotionMetrics metrics{};
};

struct CaseSpec {
    std::string name{};
    std::string description{};
    std::vector<MotionPhase> phases{};
    bool stress_case{false};
    bool use_live_tilt_safety_trip{false};
    std::function<bool(const CaseResult&, std::string&)> evaluate{};
    std::function<void(control_config::ControlConfig&)> configure{};
};

enum class CaseProfile {
    Canonical,
    Stress,
    All,
};

std::filesystem::path defaultArtifactRoot() {
    if (const char* env = std::getenv("HEXAPOD_LOCOMOTION_ARTIFACT_DIR"); env != nullptr && env[0] != '\0') {
        return std::filesystem::path(env);
    }
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    std::filesystem::path root = std::filesystem::temp_directory_path();
    root /= "hexapod_locomotion_regression";
    root /= std::to_string(ms);
    root /= std::to_string(::getpid());
    return root;
}

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << text;
}

std::filesystem::path resolveExistingPath(const std::vector<std::filesystem::path>& candidates) {
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec)) {
            return std::filesystem::weakly_canonical(candidate, ec);
        }
    }
    return {};
}

std::vector<MotionPhase> buildPhasesFromScenario(const std::filesystem::path& path, const std::string& label) {
    if (path.empty()) {
        throw std::runtime_error(label + ": scenario file not found");
    }
    ScenarioDefinition def{};
    std::string error;
    const std::string path_str = path.string();
    if (!ScenarioDriver::loadFromToml(path_str, def, error)) {
        throw std::runtime_error(label + ": failed to load scenario " + path_str + ": " + error);
    }

    std::vector<MotionPhase> phases;
    if (def.events.empty()) {
        return phases;
    }

    std::vector<ScenarioEvent> events = def.events;
    std::sort(events.begin(), events.end(), [](const ScenarioEvent& lhs, const ScenarioEvent& rhs) {
        return lhs.at_ms < rhs.at_ms;
    });

    for (std::size_t i = 0; i < events.size(); ++i) {
        const ScenarioEvent& event = events[i];
        const uint64_t next_at_ms = (i + 1 < events.size()) ? events[i + 1].at_ms : def.duration_ms;
        const uint64_t span_ms = next_at_ms > event.at_ms ? next_at_ms - event.at_ms : def.tick_ms;
        MotionPhase phase{};
        phase.label = label + "_phase_" + std::to_string(i);
        phase.motion = event.motion;
        phase.steps = static_cast<std::size_t>(std::max<uint64_t>(1, span_ms / def.tick_ms));
        phase.refresh_each_step = def.refresh_motion_intent;
        if (event.has_safety_overrides && event.safety.has_legs_enabled) {
            phase.safety_leg_enabled_mask = event.safety.legs_enabled;
        }
        phases.push_back(std::move(phase));
    }
    return phases;
}

MotionPhase makePhase(const std::string& label,
                      const ScenarioMotionIntent& motion,
                      const std::size_t steps,
                      const bool refresh_each_step = true) {
    MotionPhase phase{};
    phase.label = label;
    phase.motion = motion;
    phase.steps = steps;
    phase.refresh_each_step = refresh_each_step;
    return phase;
}

void scalePhasesForHarness(std::vector<MotionPhase>& phases, const int bus_loop_period_us) {
    for (MotionPhase& phase : phases) {
        phase.steps = physics_sim_test_utils::scaledLegacyStepCount(phase.steps, bus_loop_period_us);
    }
}

std::size_t samplesForDuration(const double duration_s, const double sample_period_s) {
    if (sample_period_s <= 0.0) {
        return 1;
    }
    return std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(duration_s / sample_period_s)));
}

std::string locomotionRegressionLimitsAppliedJson(const std::string& case_name, const LocomotionMetrics& m) {
    std::ostringstream o;
    o << "{\"case_name\":\"" << jsonEscape(case_name) << '"';
    if (case_name == "steady_forward_walk") {
        o << ",\"walk_sample_count_min\":250"
          << ",\"stride_count_min\":5"
          << ",\"path_length_m_min\":0.12"
          << ",\"net_displacement_m_min\":0.08"
          << ",\"max_abs_roll_rad_max\":0.65"
          << ",\"max_abs_pitch_rad_max\":0.65"
          << ",\"mean_horizontal_speed_mps_min\":0.02"
          << ",\"mean_horizontal_speed_mps_max\":0.22"
          << ",\"expect_fault\":false";
    } else if (case_name == "turn_in_place") {
        o << ",\"yaw_delta_abs_min_rad\":0.25"
          << ",\"net_displacement_m_max\":0.20"
          << ",\"path_length_m_max\":2.5"
          << ",\"stride_count_min\":4"
          << ",\"expect_fault\":false";
    } else if (case_name == "gait_transition_stability") {
        o << ",\"mode_transition_count_min\":2"
          << ",\"gait_segments_min\":4"
          << ",\"step_length_delta_min_m\":0.001"
          << ",\"swing_height_delta_min_m\":0.001"
          << ",\"expect_fault\":false";
    } else if (case_name == "aggressive_governor") {
        o << ",\"min_command_scale_max\":0.95"
          << ",\"min_cadence_scale_max\":0.97"
          << ",\"max_governed_speed_mps_min\":0.01"
          << ",\"stride_count_min\":2"
          << ",\"expect_fault\":false";
    } else if (case_name == "command_timeout_fallback") {
        o << ",\"expect_fault\":true"
          << ",\"expected_fault\":\"COMMAND_TIMEOUT\""
          << ",\"final_mode\":\"SAFE_IDLE\""
          << ",\"first_fault_step_min\":20";
    } else if (case_name == "low_support_walk") {
        o << ",\"walk_sample_count_min\":250"
          << ",\"stride_count_min\":4"
          << ",\"min_support_margin_m_max\":0.06"
          << ",\"governor_attenuation_required\":true"
          << ",\"expect_fault\":false";
    } else if (case_name == "long_walk_observability") {
        constexpr const char* kSuite = "locomotion_regression";
        constexpr const char* kCase = "long_walk_observability";
        const std::size_t walk_min =
            samplesForDuration(test_limits::getDouble(kSuite, kCase, "", "min_walk_duration_before_fault_s", 10.0),
                               m.sample_period_s);
        const std::size_t trans_win = samplesForDuration(1.2, m.sample_period_s);
        const std::size_t base_tail = samplesForDuration(0.8, m.sample_period_s);
        const std::size_t stride_min = test_limits::getSizeT(kSuite, kCase, "", "min_stride_count", 6);
        const double path_min = test_limits::getDouble(kSuite, kCase, "", "min_path_length_m", 0.75);
        const double fault_min_s = test_limits::getDouble(kSuite, kCase, "", "min_walk_fault_time_s", 10.0);
        const double fault_max_s = test_limits::getDouble(kSuite, kCase, "", "max_walk_fault_time_s", 65.0);
        const double trans_improve =
            test_limits::getDouble(kSuite, kCase, "", "transition_improvement_min_m", 0.001);
        const double trans_max_h =
            test_limits::getDouble(kSuite, kCase, "", "transition_max_body_height_m", 0.141);
        const double max_anchor_drift =
            test_limits::getDouble(kSuite, kCase, "", "max_contact_anchor_max_drift_m", 0.08);
        const double min_foot_z =
            test_limits::getDouble(kSuite, kCase, "", "min_measured_foot_world_z_m", -0.03);
        const double max_track_err =
            test_limits::getDouble(kSuite, kCase, "", "max_contact_tracking_error_m", 0.10);
        o << ",\"expect_fault\":true"
          << ",\"expected_fault_any\":[\"TIP_OVER\",\"BODY_COLLAPSE\"]"
          << ",\"walk_sample_count_min\":" << walk_min
          << ",\"stride_count_min\":" << stride_min
          << ",\"path_length_m_min\":" << formatDouble(path_min)
          << ",\"first_fault_time_s_min\":" << formatDouble(fault_min_s)
          << ",\"first_fault_time_s_max\":" << formatDouble(fault_max_s)
          << ",\"transition_improvement_body_height_min_m\":" << formatDouble(trans_improve)
          << ",\"transition_max_body_height_m\":" << formatDouble(trans_max_h)
          << ",\"max_contact_anchor_max_drift_m\":" << formatDouble(max_anchor_drift)
          << ",\"min_measured_foot_world_z_m\":" << formatDouble(min_foot_z)
          << ",\"max_contact_tracking_error_m\":" << formatDouble(max_track_err)
          << ",\"transition_window_samples\":" << trans_win
          << ",\"baseline_tail_samples\":" << base_tail;
    } else if (case_name == "tilt_safety_trip") {
        o << ",\"expect_fault\":true"
          << ",\"expected_fault\":\"TIP_OVER\""
          << ",\"first_fault_step_min\":40"
          << ",\"path_length_m_min\":0.1"
          << ",\"configured_max_tilt_rad\":0.25"
          << ",\"configured_rapid_body_rate_radps\":0.45";
    }
    o << '}';
    return o.str();
}

std::string caseResultSummaryJson(const CaseResult& result) {
    std::ostringstream out;
    out << '{'
        << "\"suite\":\"locomotion_regression\","
        << "\"name\":\"" << jsonEscape(result.name) << "\","
        << "\"description\":\"" << jsonEscape(result.description) << "\","
        << "\"passed\":" << (result.passed ? "true" : "false") << ','
        << "\"failure_reason\":\"" << jsonEscape(result.failure_reason) << "\","
        << "\"replay_path\":\"" << jsonEscape(result.replay_path.string()) << "\","
        << "\"geometry_path\":\"" << jsonEscape(result.geometry_path.string()) << "\","
        << "\"summary_path\":\"" << jsonEscape(result.summary_path.string()) << "\","
        << "\"metrics_path\":\"" << jsonEscape(result.metrics_path.string()) << "\","
        << "\"limits_applied\":" << locomotionRegressionLimitsAppliedJson(result.name, result.metrics) << ','
        << "\"metrics\":" << metricsToJson(result.metrics)
        << '}';
    return out.str();
}

std::string motionTimelineString(const std::vector<MotionPhase>& phases) {
    std::ostringstream out;
    for (std::size_t i = 0; i < phases.size(); ++i) {
        if (i > 0) {
            out << " -> ";
        }
        const MotionPhase& phase = phases[i];
        out << '[' << phase.label
            << " mode=" << robotModeName(phase.motion.mode)
            << " gait=" << gaitName(phase.motion.gait)
            << " height=" << phase.motion.body_height_m
            << " speed=" << phase.motion.speed_mps
            << " yaw_rate=" << phase.motion.yaw_rate_radps
            << " steps=" << phase.steps
            << ']';
    }
    return out.str();
}

CaseResult runCase(const std::string& sim_exe,
                   const std::filesystem::path& artifact_root,
                   const CaseSpec& spec) {
    CaseResult result{};
    result.name = spec.name;
    result.description = spec.description;

    const std::filesystem::path case_dir = artifact_root / spec.name;
    std::filesystem::create_directories(case_dir);
    result.replay_path = case_dir / "replay.ndjson";
    result.geometry_path = case_dir / "geometry.json";
    result.summary_path = case_dir / "summary.json";
    result.metrics_path = case_dir / "metrics.json";

    writeTextFile(result.geometry_path, telemetry_json::serializeGeometryPacket(geometry_config::activeHexapodGeometry()));

    const int port = 27000 + (static_cast<int>(::getpid()) % 4000) + static_cast<int>(std::hash<std::string>{}(spec.name) % 1000);
    PhysicsSimProcess sim(sim_exe, port);
    if (!sim.start()) {
        throw std::runtime_error(spec.name + ": failed to start physics sim process");
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1",
        port,
        harness.bus_loop_period_us,
        harness.physics_solver_iterations);
    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.control_loop_trace_enabled = false;
    cfg.telemetry.enabled = false;
    cfg.replay_log.enabled = true;
    cfg.replay_log.file_path = result.replay_path.string();
    if (spec.configure) {
        spec.configure(cfg);
    }

    std::vector<MotionPhase> scaled_phases = spec.phases;
    scalePhasesForHarness(scaled_phases, harness.bus_loop_period_us);
    std::vector<MotionSample> samples{};
    LocomotionMetrics metrics{};
    metrics.sample_period_s = physics_sim_test_utils::controlLoopPeriodSeconds(cfg);
    auto replay_logger = std::make_unique<CollectingReplayLogger>(result.replay_path.string());
    const auto* replay_logger_ptr = replay_logger.get();
    {
        RobotRuntime runtime(
            std::move(bridge),
            std::make_unique<PhysicsSimEstimator>(),
            nullptr,
            cfg,
            telemetry::makeNoopTelemetryPublisher(),
            std::move(replay_logger));
        if (!runtime.init()) {
            throw std::runtime_error(spec.name + ": runtime init failed");
        }

        if (!runMotionSequence(runtime, scaled_phases, samples, metrics)) {
            throw std::runtime_error(spec.name + ": motion runner failed unexpectedly");
        }
        replay_logger_ptr->flush();
    }

    result.samples = std::move(samples);
    result.metrics = metrics;

    std::string failure_reason;
    result.passed = spec.evaluate(result, failure_reason);
    result.failure_reason = failure_reason;

    std::ofstream metrics_out(result.metrics_path);
    metrics_out << metricsToJson(result.metrics) << '\n';
    std::ofstream summary_out(result.summary_path);
    summary_out << caseResultSummaryJson(result) << '\n';

    return result;
}

bool caseSteadyForwardWalk(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!(m.walk_sample_count >= 250)) {
        reason = "steady walk should sustain many active samples";
        return false;
    }
    if (!(m.stride_count >= 5)) {
        reason = "steady walk should cover multiple strides";
        return false;
    }
    if (!(m.path_length_m >= 0.12)) {
        reason = "steady walk should accumulate measurable path length";
        return false;
    }
    if (!(m.net_displacement_m >= 0.08)) {
        reason = "steady walk should move the body forward";
        return false;
    }
    if (!(m.max_abs_roll_rad < 0.65 && m.max_abs_pitch_rad < 0.65)) {
        reason = "steady walk should remain inside the tilt envelope";
        return false;
    }
    if (!(m.saw_fault == false)) {
        reason = "steady walk should not trip a fault";
        return false;
    }
    if (!(m.mean_horizontal_speed_mps >= 0.02 && m.mean_horizontal_speed_mps <= 0.22)) {
        reason = "steady walk mean speed should stay in the expected band";
        return false;
    }
    return true;
}

bool caseTurnInPlace(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!(std::abs(m.yaw_delta_rad) >= 0.25)) {
        reason = "turn-in-place should accumulate yaw";
        return false;
    }
    if (!(m.net_displacement_m <= 0.20)) {
        reason = "turn-in-place should keep net translation bounded";
        return false;
    }
    if (!(m.path_length_m <= 2.5)) {
        reason = "turn-in-place path length should stay bounded";
        return false;
    }
    if (!(m.stride_count >= 4)) {
        reason = "turn-in-place should still step through multiple strides";
        return false;
    }
    if (m.saw_fault) {
        reason = "turn-in-place should not fault";
        return false;
    }
    return true;
}

bool caseGaitTransitionStability(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (m.saw_fault) {
        reason = "gait transition case should not fault";
        return false;
    }
    if (m.mode_transition_count < 2) {
        reason = "gait transition case should show at least two mode transitions";
        return false;
    }
    if (m.gait_segments.size() < 4) {
        reason = "gait transition case should preserve multiple gait segments";
        return false;
    }
    if (!(m.max_step_length_m > m.min_step_length_m + 0.001)) {
        reason = "gait transition case should change step length across gait changes";
        return false;
    }
    if (!(m.max_swing_height_m > m.min_swing_height_m + 0.001)) {
        reason = "gait transition case should change swing height across gait changes";
        return false;
    }
    return true;
}

bool caseLongWalkObservability(const CaseResult& result, std::string& reason) {
    constexpr const char* kSuite = "locomotion_regression";
    constexpr const char* kCase = "long_walk_observability";
    const auto& m = result.metrics;
    const double min_walk_duration_s =
        test_limits::getDouble(kSuite, kCase, "", "min_walk_duration_before_fault_s", 10.0);
    const std::size_t min_stride = test_limits::getSizeT(kSuite, kCase, "", "min_stride_count", 6);
    const double min_path_m = test_limits::getDouble(kSuite, kCase, "", "min_path_length_m", 0.75);
    const double min_fault_time_s = test_limits::getDouble(kSuite, kCase, "", "min_walk_fault_time_s", 10.0);
    const double max_fault_time_s = test_limits::getDouble(kSuite, kCase, "", "max_walk_fault_time_s", 65.0);
    const double kTransitionImprovementMinM =
        test_limits::getDouble(kSuite, kCase, "", "transition_improvement_min_m", 0.001);
    const double kTransitionMaxBodyHeightM =
        test_limits::getDouble(kSuite, kCase, "", "transition_max_body_height_m", 0.141);
    const double kMaxContactAnchorMaxDriftM =
        test_limits::getDouble(kSuite, kCase, "", "max_contact_anchor_max_drift_m", 0.08);
    const double kMinMeasuredFootWorldZM =
        test_limits::getDouble(kSuite, kCase, "", "min_measured_foot_world_z_m", -0.03);
    const double kMaxContactTrackingErrorM =
        test_limits::getDouble(kSuite, kCase, "", "max_contact_tracking_error_m", 0.10);
    if (!m.saw_fault) {
        reason = "long walk observability should eventually reach the safety envelope";
        return false;
    }
    if (m.first_fault != FaultCode::TIP_OVER && m.first_fault != FaultCode::BODY_COLLAPSE) {
        reason =
            "long walk observability should trip TIP_OVER or BODY_COLLAPSE at the end of the stress window";
        return false;
    }
    if (!(m.walk_sample_count >= samplesForDuration(min_walk_duration_s, m.sample_period_s))) {
        reason = "long walk should sustain a large walking window before the fault";
        return false;
    }
    if (!(m.stride_count >= min_stride)) {
        reason = "long walk should accumulate many strides";
        return false;
    }
    if (!(m.path_length_m >= min_path_m)) {
        reason = "long walk should accumulate path length";
        return false;
    }
    const double first_fault_time_s = static_cast<double>(m.first_fault_step) * m.sample_period_s;
    if (!(first_fault_time_s > min_fault_time_s && first_fault_time_s < max_fault_time_s)) {
        reason = "long walk should fault only after a sustained observation window";
        return false;
    }

    constexpr const char* kSlowTripodPhaseLabel = "long_walk_observability_phase_2";
    constexpr const char* kFastTripodPhaseLabel = "long_walk_observability_phase_3";
    const std::size_t kTransitionWindowSamples = samplesForDuration(1.2, m.sample_period_s);
    const std::size_t kBaselineTailSamples = samplesForDuration(0.8, m.sample_period_s);
    std::vector<double> slow_tripod_body_heights_m{};
    std::size_t transition_samples = 0;
    double transition_sum_body_height_m = 0.0;
    double transition_max_body_height_m = -std::numeric_limits<double>::infinity();
    for (const MotionSample& sample : result.samples) {
        if (sample.phase_label == kSlowTripodPhaseLabel) {
            slow_tripod_body_heights_m.push_back(sample.position.z);
        }
        if (sample.phase_label != kFastTripodPhaseLabel) {
            continue;
        }
        transition_sum_body_height_m += sample.position.z;
        transition_max_body_height_m = std::max(transition_max_body_height_m, sample.position.z);
        ++transition_samples;
        if (transition_samples >= kTransitionWindowSamples) {
            break;
        }
    }
    if (slow_tripod_body_heights_m.size() < kBaselineTailSamples) {
        reason = "long walk should expose enough slow-tripod samples before the fast transition";
        return false;
    }
    if (transition_samples < kTransitionWindowSamples) {
        reason = "long walk should expose enough early fast-tripod samples for transition validation";
        return false;
    }
    double baseline_tail_sum_body_height_m = 0.0;
    for (std::size_t i = slow_tripod_body_heights_m.size() - kBaselineTailSamples;
         i < slow_tripod_body_heights_m.size();
         ++i) {
        baseline_tail_sum_body_height_m += slow_tripod_body_heights_m[i];
    }
    const double baseline_tail_mean_body_height_m =
        baseline_tail_sum_body_height_m / static_cast<double>(kBaselineTailSamples);
    const double transition_mean_body_height_m =
        transition_sum_body_height_m / static_cast<double>(transition_samples);
    if (!(transition_mean_body_height_m >= baseline_tail_mean_body_height_m + kTransitionImprovementMinM)) {
        reason = "fast tripod transition should reduce sag promptly instead of carrying the slow-phase body-height offset";
        return false;
    }
    if (!(transition_max_body_height_m <= kTransitionMaxBodyHeightM)) {
        reason = "fast tripod transition should stay near commanded body height without overshooting upward";
        return false;
    }
    if (!(m.max_contact_anchor_max_drift_m <= kMaxContactAnchorMaxDriftM)) {
        reason = "long walk should keep stance foot contact anchors from drifting excessively while contact is held";
        return false;
    }
    if (!(m.min_measured_foot_world_z_m >= kMinMeasuredFootWorldZM)) {
        reason = "long walk should keep measured feet from penetrating materially below the ground plane";
        return false;
    }
    if (!(m.max_contact_tracking_error_m <= kMaxContactTrackingErrorM)) {
        reason = "long walk should avoid sustained large commanded-versus-measured stance foot error";
        return false;
    }
    return true;
}

bool caseTimeoutFallback(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!m.saw_fault) {
        reason = "timeout fallback should eventually reject the stale intent";
        return false;
    }
    if (m.first_fault != FaultCode::COMMAND_TIMEOUT) {
        reason = "timeout fallback should trip COMMAND_TIMEOUT";
        return false;
    }
    if (!(m.final_mode == RobotMode::SAFE_IDLE)) {
        reason = "timeout fallback should land in SAFE_IDLE";
        return false;
    }
    if (!(m.first_fault_step > 20 && m.first_fault_step < result.samples.size())) {
        reason = "timeout fallback should trip after the initial hold window";
        return false;
    }
    return true;
}

bool caseSparseSupportWalk(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!(m.walk_sample_count >= 250)) {
        reason = "low-support walk should keep moving";
        return false;
    }
    if (!(m.stride_count >= 4)) {
        reason = "low-support walk should still generate stride cycles";
        return false;
    }
    if (!(m.min_support_margin_m < 0.06)) {
        reason = "low-support walk should actually exercise the lower margin envelope";
        return false;
    }
    if (!(m.min_command_scale < 0.99 || m.min_cadence_scale < 0.99)) {
        reason = "low-support walk should trigger some governor shaping";
        return false;
    }
    if (m.saw_fault) {
        reason = "low-support walk should not fault";
        return false;
    }
    return true;
}

bool caseAggressiveGovernor(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!(m.min_command_scale < 0.95)) {
        reason = "aggressive command should be attenuated by the governor";
        return false;
    }
    if (!(m.min_cadence_scale < 0.97)) {
        reason = "aggressive command should reduce cadence";
        return false;
    }
    if (!(m.max_governed_speed_mps > 0.01)) {
        reason = "aggressive command should not collapse to zero";
        return false;
    }
    if (!(m.stride_count >= 2)) {
        reason = "aggressive command should still produce stepping";
        return false;
    }
    if (m.saw_fault) {
        reason = "aggressive governor case should remain in bounds";
        return false;
    }
    return true;
}

bool caseTiltSafetyTrip(const CaseResult& result, std::string& reason) {
    const auto& m = result.metrics;
    if (!m.saw_fault) {
        reason = "tilt safety case should trip a fault";
        return false;
    }
    if (m.first_fault != FaultCode::TIP_OVER) {
        reason = "tilt safety case should trip TIP_OVER";
        return false;
    }
    if (!(m.first_fault_step > 40 && m.first_fault_step < result.samples.size())) {
        reason = "tilt safety case should fault after motion begins";
        return false;
    }
    if (!(m.path_length_m > 0.1)) {
        reason = "tilt safety case should accumulate motion before the fault";
        return false;
    }
    return true;
}

std::vector<CaseSpec> buildCaseCatalog() {
    const std::filesystem::path nominal_scenario = resolveExistingPath({
        "scenarios/01_nominal_stand_walk.toml",
        "../scenarios/01_nominal_stand_walk.toml",
        "hexapod-server/scenarios/01_nominal_stand_walk.toml",
    });
    const std::filesystem::path long_walk_scenario = resolveExistingPath({
        "scenarios/05_long_walk_observability.toml",
        "../scenarios/05_long_walk_observability.toml",
        "hexapod-server/scenarios/05_long_walk_observability.toml",
    });
    const std::filesystem::path timeout_scenario = resolveExistingPath({
        "scenarios/02_command_timeout_fallback.toml",
        "../scenarios/02_command_timeout_fallback.toml",
        "hexapod-server/scenarios/02_command_timeout_fallback.toml",
    });

    const std::vector<MotionPhase> nominal_stand_walk = buildPhasesFromScenario(
        nominal_scenario,
        "nominal_stand_walk");
    const std::vector<MotionPhase> long_walk = buildPhasesFromScenario(
        long_walk_scenario,
        "long_walk_observability");
    const std::vector<MotionPhase> timeout_fallback = buildPhasesFromScenario(
        timeout_scenario,
        "command_timeout_fallback");

    std::vector<CaseSpec> cases;

    cases.push_back(CaseSpec{
        "steady_forward_walk",
        "Sustained forward walk should accumulate displacement over many strides without tipping.",
        {
            makePhase("stand_settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 100),
            makePhase("steady_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::TRIPOD, 0.10, 0.08, 0.0, 0.0}, 700),
            makePhase("stand_exit", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 80),
        },
        false,
        false,
        caseSteadyForwardWalk,
    });

    cases.push_back(CaseSpec{
        "turn_in_place",
        "A turn-in-place command should rotate the body while keeping translation bounded.",
        {
            makePhase("stand_settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 100),
            makePhase("turn", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0, 0.45}, 700),
            makePhase("stand_exit", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 60),
        },
        false,
        false,
        caseTurnInPlace,
    });

    cases.push_back(CaseSpec{
        "gait_transition_stability",
        "Mixed gait changes should preserve motion without an abrupt safety stop.",
        {
            makePhase("settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 100),
            makePhase("tripod_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::TRIPOD, 0.10, 0.06, 0.0, 0.0}, 260),
            makePhase("ripple_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::RIPPLE, 0.10, 0.05, 0.12, 0.18}, 240),
            makePhase("wave_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::WAVE, 0.10, 0.04, -0.18, -0.12}, 240),
            makePhase("recover", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 100),
        },
        false,
        false,
        caseGaitTransitionStability,
    });

    cases.push_back(CaseSpec{
        "aggressive_governor",
        "Aggressive commands should be softened continuously instead of being hard-stopped.",
        {
            makePhase("settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 100),
            makePhase("aggressive_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::TRIPOD, 0.10, 0.28, 0.24, 0.0}, 140),
            makePhase("recover", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.10, 0.0, 0.0, 0.0}, 80),
        },
        false,
        false,
        caseAggressiveGovernor,
    });

    cases.push_back(CaseSpec{
        "command_timeout_fallback",
        "A stale command stream should fall back through freshness gating rather than keep walking indefinitely.",
        timeout_fallback,
        false,
        false,
        caseTimeoutFallback,
        [](control_config::ControlConfig& cfg) {
            cfg.freshness.intent.max_allowed_age_us = DurationUs{250'000};
            cfg.freshness.intent.require_timestamp = true;
            cfg.freshness.intent.require_nonzero_sample_id = true;
            cfg.freshness.intent.require_monotonic_sample_id = true;
        },
    });

    cases.push_back(CaseSpec{
        "low_support_walk",
        "A low-body, sparse-support walk should be shaped by the governor but still keep moving.",
        {
            makePhase("settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.08, 0.0, 0.0, 0.0}, 100),
            makePhase("sparse_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::WAVE, 0.08, 0.03, 0.0, 0.05}, 500),
            makePhase("recover", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.08, 0.0, 0.0, 0.0}, 80),
        },
        false,
        false,
        caseSparseSupportWalk,
    });

    cases.push_back(CaseSpec{
        "long_walk_observability",
        "The long-walk scenario should keep moving for a long window and only trip safety late in the stress segment.",
        long_walk,
        true,
        false,
        caseLongWalkObservability,
    });

    cases.push_back(CaseSpec{
        "tilt_safety_trip",
        "A deliberately tightened tilt envelope should still trigger safety before the robot falls flat.",
        {
            makePhase("settle", ScenarioMotionIntent{true, RobotMode::STAND, GaitType::TRIPOD, 0.12, 0.0, 0.0, 0.0}, 100),
            makePhase("unsafe_walk", ScenarioMotionIntent{true, RobotMode::WALK, GaitType::TRIPOD, 0.12, 0.45, 1.57, 0.0}, 600),
        },
        false,
        true,
        caseTiltSafetyTrip,
        [](control_config::ControlConfig& cfg) {
            cfg.safety.max_tilt_rad = AngleRad{0.25};
            cfg.safety.rapid_body_rate_radps = 0.45;
            cfg.safety.rapid_body_rate_max_contacts = 4;
        },
    });

    return cases;
}

bool profileAllowsCase(const CaseProfile profile, const CaseSpec& spec) {
    switch (profile) {
    case CaseProfile::Canonical:
        return !spec.stress_case;
    case CaseProfile::Stress:
        return spec.stress_case;
    case CaseProfile::All:
        return true;
    }
    return true;
}

void writeBundleIndex(const std::filesystem::path& artifact_root, const std::vector<CaseResult>& results) {
    const std::filesystem::path path = artifact_root / "manifest.json";
    std::ofstream out(path);
    out << "{\n  \"cases\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        if (i > 0) {
            out << ",\n";
        }
        out << "    " << caseResultSummaryJson(results[i]);
    }
    out << "\n  ]\n}\n";
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_locomotion_regression_suite (Linux-only)\n";
    return 0;
#else
    std::optional<std::string> requested_case{};
    std::optional<std::filesystem::path> requested_artifact_dir{};
    const char* sim_exe = nullptr;
    CaseProfile requested_profile = CaseProfile::Canonical;
    bool emit_metrics_json = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--emit-metrics-json") {
            emit_metrics_json = true;
            continue;
        }
        if (arg == "--limits-manifest") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --limits-manifest\n";
                return EXIT_FAILURE;
            }
            ++i;
            continue;
        }
        if (arg == "--case") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --case\n";
                return EXIT_FAILURE;
            }
            requested_case = argv[++i];
            continue;
        }
        if (arg == "--artifact-dir") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --artifact-dir\n";
                return EXIT_FAILURE;
            }
            requested_artifact_dir = std::filesystem::path(argv[++i]);
            continue;
        }
        if (arg == "--profile") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --profile\n";
                return EXIT_FAILURE;
            }
            const std::string profile = argv[++i];
            if (profile == "canonical") {
                requested_profile = CaseProfile::Canonical;
            } else if (profile == "stress") {
                requested_profile = CaseProfile::Stress;
            } else if (profile == "all") {
                requested_profile = CaseProfile::All;
            } else {
                std::cerr << "Unknown profile '" << profile << "'\n";
                return EXIT_FAILURE;
            }
            continue;
        }
        if (arg == "--sim") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --sim\n";
                return EXIT_FAILURE;
            }
            sim_exe = argv[++i];
            continue;
        }
        if (arg.rfind("--", 0) == 0) {
            std::cerr << "Unknown argument: " << arg << '\n';
            return EXIT_FAILURE;
        }
        if (sim_exe == nullptr) {
            sim_exe = argv[i];
        } else {
            std::cerr << "Unexpected positional argument: " << arg << '\n';
            return EXIT_FAILURE;
        }
    }

    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }

    if (sim_exe == nullptr) {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_locomotion_regression_suite (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const std::filesystem::path artifact_root = requested_artifact_dir.has_value()
                                                    ? *requested_artifact_dir
                                                    : defaultArtifactRoot();
    std::filesystem::create_directories(artifact_root);

    std::vector<CaseSpec> cases = buildCaseCatalog();
    if (requested_case.has_value()) {
        const auto it = std::find_if(cases.begin(), cases.end(), [&](const CaseSpec& spec) {
            return spec.name == *requested_case;
        });
        if (it == cases.end()) {
            std::cerr << "Unknown case '" << *requested_case << "'\n";
            std::cerr << "Available cases:\n";
            for (const CaseSpec& spec : cases) {
                std::cerr << "  " << spec.name << '\n';
            }
            return EXIT_FAILURE;
        }
        cases = {*it};
    } else {
        std::vector<CaseSpec> filtered;
        filtered.reserve(cases.size());
        for (const CaseSpec& spec : cases) {
            if (profileAllowsCase(requested_profile, spec)) {
                filtered.push_back(spec);
            }
        }
        cases = std::move(filtered);
    }

    std::vector<CaseResult> results;
    results.reserve(cases.size());

    bool all_passed = true;
    for (const CaseSpec& spec : cases) {
        try {
            CaseResult result = runCase(sim_exe, artifact_root, spec);
            all_passed = all_passed && result.passed;
            std::cout << spec.name
                      << " passed=" << (result.passed ? 1 : 0)
                      << " samples=" << result.metrics.sample_count
                      << " stride_count=" << result.metrics.stride_count
                      << " path_m=" << formatDouble(result.metrics.path_length_m)
                      << " disp_m=" << formatDouble(result.metrics.net_displacement_m)
                      << " roll_max=" << formatDouble(result.metrics.max_abs_roll_rad)
                      << " pitch_max=" << formatDouble(result.metrics.max_abs_pitch_rad)
                      << " min_trust=" << formatDouble(result.metrics.min_model_trust)
                      << " min_scale=" << formatDouble(result.metrics.min_command_scale)
                      << " min_cadence=" << formatDouble(result.metrics.min_cadence_scale)
                      << " fault=" << faultName(result.metrics.final_fault)
                      << '\n';
            if (emit_metrics_json) {
                std::cout << "{\"suite\":\"locomotion_regression\",\"name\":\"" << jsonEscape(spec.name) << "\",\"passed\":"
                          << (result.passed ? "true" : "false") << ",\"limits_applied\":"
                          << locomotionRegressionLimitsAppliedJson(spec.name, result.metrics) << ",\"metrics\":"
                          << metricsToJson(result.metrics) << "}\n";
            }
            if (!result.passed) {
                std::cerr << spec.name << ": " << result.failure_reason << '\n';
                std::cerr << spec.name << " metrics: first_fault=" << faultName(result.metrics.first_fault)
                          << " first_fault_step=" << result.metrics.first_fault_step
                          << " max_body_rate_radps=" << formatDouble(result.metrics.max_body_rate_radps)
                          << " max_abs_roll_rad=" << formatDouble(result.metrics.max_abs_roll_rad)
                          << " max_abs_pitch_rad=" << formatDouble(result.metrics.max_abs_pitch_rad) << '\n';
            }
            results.push_back(std::move(result));
        } catch (const std::exception& ex) {
            std::cerr << spec.name << ": exception: " << ex.what() << '\n';
            all_passed = false;
        }
    }

    writeBundleIndex(artifact_root, results);

    if (!all_passed) {
        std::cerr << "locomotion regression artifacts: " << artifact_root << '\n';
        std::cerr << "To replay one case in the visualiser:\n";
        std::cerr << "  python3 scripts/replay_locomotion_bundle.py --geometry "
                  << (artifact_root / "CASE/geometry.json") << " --replay "
                  << (artifact_root / "CASE/replay.ndjson") << " --udp-port 9870\n";
        return EXIT_FAILURE;
    }

    std::cout << "locomotion regression artifacts: " << artifact_root << '\n';
    return EXIT_SUCCESS;
#endif
}
