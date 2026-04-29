#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "replay_logger.hpp"
#include "replay_json.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "toml_parser.hpp"
#include "telemetry_json.hpp"

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

constexpr double kControlLoopPeriodS = 0.020;

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

std::string jsonEscape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
        case '\\':
        case '"':
            escaped.push_back('\\');
            escaped.push_back(ch);
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            escaped.push_back(ch);
            break;
        }
    }
    return escaped;
}

std::string formatDouble(const double value, const int precision = 6) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    std::string text = out.str();
    const auto trailing = text.find_last_not_of('0');
    if (trailing != std::string::npos && text[trailing] == '.') {
        text.erase(trailing);
    } else if (trailing != std::string::npos) {
        text.erase(trailing + 1);
    }
    if (text.empty()) {
        return "0";
    }
    return text;
}

std::string robotModeName(const RobotMode mode) {
    switch (mode) {
    case RobotMode::SAFE_IDLE:
        return "SAFE_IDLE";
    case RobotMode::HOMING:
        return "HOMING";
    case RobotMode::STAND:
        return "STAND";
    case RobotMode::WALK:
        return "WALK";
    case RobotMode::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
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

std::string faultName(const FaultCode fault) {
    switch (fault) {
    case FaultCode::NONE:
        return "NONE";
    case FaultCode::BUS_TIMEOUT:
        return "BUS_TIMEOUT";
    case FaultCode::ESTOP:
        return "ESTOP";
    case FaultCode::TIP_OVER:
        return "TIP_OVER";
    case FaultCode::ESTIMATOR_INVALID:
        return "ESTIMATOR_INVALID";
    case FaultCode::MOTOR_FAULT:
        return "MOTOR_FAULT";
    case FaultCode::JOINT_LIMIT:
        return "JOINT_LIMIT";
    case FaultCode::COMMAND_TIMEOUT:
        return "COMMAND_TIMEOUT";
    }
    return "UNKNOWN";
}

Vec3 positionFromState(const RobotState& state) {
    return Vec3{
        state.body_twist_state.body_trans_m.x,
        state.body_twist_state.body_trans_m.y,
        state.body_twist_state.body_trans_m.z,
    };
}

double horizontalSpeedFromState(const RobotState& state) {
    return std::hypot(state.body_twist_state.body_trans_mps.x,
                      state.body_twist_state.body_trans_mps.y);
}

double bodyTiltFromState(const RobotState& state) {
    if (!state.has_body_twist_state) {
        return 0.0;
    }
    return std::hypot(state.body_twist_state.twist_pos_rad.x, state.body_twist_state.twist_pos_rad.y);
}

double bodyRateFromState(const RobotState& state) {
    if (state.has_imu && state.imu.valid) {
        return std::hypot(state.imu.gyro_radps.x, state.imu.gyro_radps.y);
    }
    return std::hypot(state.body_twist_state.twist_vel_radps.x,
                      state.body_twist_state.twist_vel_radps.y);
}

double wrapAngleDiff(const double start, const double end) {
    return std::atan2(std::sin(end - start), std::cos(end - start));
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

struct MotionPhase {
    std::string label{};
    ScenarioMotionIntent motion{};
    std::size_t steps{0};
    bool refresh_each_step{true};
};

struct MotionSample {
    std::size_t step_index{0};
    std::size_t phase_index{0};
    std::string phase_label{};
    ScenarioMotionIntent requested_motion{};
    ControlStatus status{};
    RobotState estimated{};
    GaitState gait{};
    CommandGovernorState governor{};
    SafetyState safety{};
    Vec3 position{};
    double horizontal_speed_mps{0.0};
    double body_tilt_rad{0.0};
    double body_rate_radps{0.0};
    double yaw_rate_radps{0.0};
    double support_margin_m{0.0};
    double stride_phase_rate_hz{0.0};
    double step_length_m{0.0};
    double swing_height_m{0.0};
    double duty_factor{0.0};
    double model_trust{1.0};
    double contact_mismatch_ratio{0.0};
};

struct ModeSegment {
    RobotMode mode{RobotMode::SAFE_IDLE};
    std::size_t start_step{0};
    std::size_t end_step{0};
};

struct GaitTransitionSegment {
    std::string phase_label{};
    std::size_t start_step{0};
    std::size_t end_step{0};
    double mean_step_length_m{0.0};
    double mean_duty_factor{0.0};
};

struct LocomotionMetrics {
    std::size_t sample_count{0};
    std::size_t walk_sample_count{0};
    std::size_t stride_count{0};
    std::size_t mode_transition_count{0};
    std::size_t fault_transition_count{0};
    std::size_t first_fault_step{0};
    bool saw_fault{false};
    FaultCode first_fault{FaultCode::NONE};
    RobotMode final_mode{RobotMode::SAFE_IDLE};
    FaultCode final_fault{FaultCode::NONE};
    double duration_s{0.0};
    double path_length_m{0.0};
    double net_displacement_m{0.0};
    double lateral_deviation_m{0.0};
    double max_abs_roll_rad{0.0};
    double max_abs_pitch_rad{0.0};
    double max_body_rate_radps{0.0};
    double mean_horizontal_speed_mps{0.0};
    double peak_horizontal_speed_mps{0.0};
    double mean_yaw_rate_radps{0.0};
    double peak_yaw_rate_radps{0.0};
    double yaw_delta_rad{0.0};
    double min_support_margin_m{std::numeric_limits<double>::infinity()};
    double min_model_trust{1.0};
    double max_contact_mismatch_ratio{0.0};
    double min_command_scale{1.0};
    double min_cadence_scale{1.0};
    double max_governor_severity{0.0};
    double max_governed_speed_mps{0.0};
    double max_governed_yaw_rate_radps{0.0};
    double min_governed_speed_mps{1e9};
    double min_governed_yaw_rate_radps{1e9};
    double max_step_length_m{0.0};
    double min_step_length_m{1e9};
    double max_swing_height_m{0.0};
    double min_swing_height_m{1e9};
    std::vector<ModeSegment> mode_segments{};
    std::vector<GaitTransitionSegment> gait_segments{};
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

std::string modeSegmentJson(const ModeSegment& segment) {
    std::ostringstream out;
    out << "{\"mode\":\"" << robotModeName(segment.mode) << "\","
        << "\"start_step\":" << segment.start_step << ','
        << "\"end_step\":" << segment.end_step << '}';
    return out.str();
}

std::string metricsToJson(const LocomotionMetrics& metrics) {
    std::ostringstream out;
    out << '{'
        << "\"sample_count\":" << metrics.sample_count << ','
        << "\"walk_sample_count\":" << metrics.walk_sample_count << ','
        << "\"stride_count\":" << metrics.stride_count << ','
        << "\"mode_transition_count\":" << metrics.mode_transition_count << ','
        << "\"fault_transition_count\":" << metrics.fault_transition_count << ','
        << "\"saw_fault\":" << (metrics.saw_fault ? "true" : "false") << ','
        << "\"first_fault\":\"" << faultName(metrics.first_fault) << "\","
        << "\"first_fault_step\":" << metrics.first_fault_step << ','
        << "\"final_mode\":\"" << robotModeName(metrics.final_mode) << "\","
        << "\"final_fault\":\"" << faultName(metrics.final_fault) << "\","
        << "\"duration_s\":" << formatDouble(metrics.duration_s) << ','
        << "\"path_length_m\":" << formatDouble(metrics.path_length_m) << ','
        << "\"net_displacement_m\":" << formatDouble(metrics.net_displacement_m) << ','
        << "\"lateral_deviation_m\":" << formatDouble(metrics.lateral_deviation_m) << ','
        << "\"max_abs_roll_rad\":" << formatDouble(metrics.max_abs_roll_rad) << ','
        << "\"max_abs_pitch_rad\":" << formatDouble(metrics.max_abs_pitch_rad) << ','
        << "\"max_body_rate_radps\":" << formatDouble(metrics.max_body_rate_radps) << ','
        << "\"mean_horizontal_speed_mps\":" << formatDouble(metrics.mean_horizontal_speed_mps) << ','
        << "\"peak_horizontal_speed_mps\":" << formatDouble(metrics.peak_horizontal_speed_mps) << ','
        << "\"mean_yaw_rate_radps\":" << formatDouble(metrics.mean_yaw_rate_radps) << ','
        << "\"peak_yaw_rate_radps\":" << formatDouble(metrics.peak_yaw_rate_radps) << ','
        << "\"yaw_delta_rad\":" << formatDouble(metrics.yaw_delta_rad) << ','
        << "\"min_support_margin_m\":" << formatDouble(metrics.min_support_margin_m) << ','
        << "\"min_model_trust\":" << formatDouble(metrics.min_model_trust) << ','
        << "\"max_contact_mismatch_ratio\":" << formatDouble(metrics.max_contact_mismatch_ratio) << ','
        << "\"min_command_scale\":" << formatDouble(metrics.min_command_scale) << ','
        << "\"min_cadence_scale\":" << formatDouble(metrics.min_cadence_scale) << ','
        << "\"max_governor_severity\":" << formatDouble(metrics.max_governor_severity) << ','
        << "\"max_governed_speed_mps\":" << formatDouble(metrics.max_governed_speed_mps) << ','
        << "\"max_governed_yaw_rate_radps\":" << formatDouble(metrics.max_governed_yaw_rate_radps) << ','
        << "\"min_governed_speed_mps\":" << formatDouble(metrics.min_governed_speed_mps) << ','
        << "\"min_governed_yaw_rate_radps\":" << formatDouble(metrics.min_governed_yaw_rate_radps) << ','
        << "\"max_step_length_m\":" << formatDouble(metrics.max_step_length_m) << ','
        << "\"min_step_length_m\":" << formatDouble(metrics.min_step_length_m) << ','
        << "\"max_swing_height_m\":" << formatDouble(metrics.max_swing_height_m) << ','
        << "\"min_swing_height_m\":" << formatDouble(metrics.min_swing_height_m) << ','
        << "\"mode_segments\":[";
    for (std::size_t i = 0; i < metrics.mode_segments.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << modeSegmentJson(metrics.mode_segments[i]);
    }
    out << "],\"gait_segments\":[";
    for (std::size_t i = 0; i < metrics.gait_segments.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        const auto& segment = metrics.gait_segments[i];
        out << "{\"phase_label\":\"" << jsonEscape(segment.phase_label) << "\","
            << "\"start_step\":" << segment.start_step << ','
            << "\"end_step\":" << segment.end_step << ','
            << "\"mean_step_length_m\":" << formatDouble(segment.mean_step_length_m) << ','
            << "\"mean_duty_factor\":" << formatDouble(segment.mean_duty_factor) << '}';
    }
    out << "]}";
    return out.str();
}

std::string caseResultSummaryJson(const CaseResult& result) {
    std::ostringstream out;
    out << '{'
        << "\"name\":\"" << jsonEscape(result.name) << "\","
        << "\"description\":\"" << jsonEscape(result.description) << "\","
        << "\"passed\":" << (result.passed ? "true" : "false") << ','
        << "\"failure_reason\":\"" << jsonEscape(result.failure_reason) << "\","
        << "\"replay_path\":\"" << jsonEscape(result.replay_path.string()) << "\","
        << "\"geometry_path\":\"" << jsonEscape(result.geometry_path.string()) << "\","
        << "\"summary_path\":\"" << jsonEscape(result.summary_path.string()) << "\","
        << "\"metrics_path\":\"" << jsonEscape(result.metrics_path.string()) << "\","
        << "\"metrics\":" << metricsToJson(result.metrics)
        << '}';
    return out.str();
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

void appendSample(LocomotionMetrics& metrics,
                  std::vector<MotionSample>& samples,
                  const MotionSample& sample) {
    samples.push_back(sample);
    metrics.sample_count = samples.size();
    metrics.final_mode = sample.status.active_mode;
    metrics.final_fault = sample.status.active_fault;
    metrics.walk_sample_count += sample.status.active_mode == RobotMode::WALK ? 1 : 0;
    metrics.max_abs_roll_rad = std::max(metrics.max_abs_roll_rad, std::abs(sample.estimated.body_twist_state.twist_pos_rad.x));
    metrics.max_abs_pitch_rad = std::max(metrics.max_abs_pitch_rad, std::abs(sample.estimated.body_twist_state.twist_pos_rad.y));
    metrics.max_body_rate_radps = std::max(metrics.max_body_rate_radps, sample.body_rate_radps);
    metrics.max_governor_severity = std::max(metrics.max_governor_severity, sample.governor.severity);
    metrics.min_support_margin_m = std::min(metrics.min_support_margin_m, sample.support_margin_m);
    metrics.min_model_trust = std::min(metrics.min_model_trust, sample.model_trust);
    metrics.max_contact_mismatch_ratio = std::max(metrics.max_contact_mismatch_ratio, sample.contact_mismatch_ratio);
    metrics.min_command_scale = std::min(metrics.min_command_scale, sample.governor.command_scale);
    metrics.min_cadence_scale = std::min(metrics.min_cadence_scale, sample.governor.cadence_scale);
    metrics.max_governed_speed_mps = std::max(metrics.max_governed_speed_mps, sample.governor.governed_planar_speed_mps);
    metrics.max_governed_yaw_rate_radps = std::max(metrics.max_governed_yaw_rate_radps, sample.governor.governed_yaw_rate_radps);
    metrics.min_governed_speed_mps = std::min(metrics.min_governed_speed_mps, sample.governor.governed_planar_speed_mps);
    metrics.min_governed_yaw_rate_radps = std::min(metrics.min_governed_yaw_rate_radps, sample.governor.governed_yaw_rate_radps);
    metrics.max_step_length_m = std::max(metrics.max_step_length_m, sample.step_length_m);
    metrics.min_step_length_m = std::min(metrics.min_step_length_m, sample.step_length_m);
    metrics.max_swing_height_m = std::max(metrics.max_swing_height_m, sample.swing_height_m);
    metrics.min_swing_height_m = std::min(metrics.min_swing_height_m, sample.swing_height_m);
    metrics.path_length_m += sample.horizontal_speed_mps * kControlLoopPeriodS;
    metrics.mean_horizontal_speed_mps += sample.horizontal_speed_mps;
    metrics.mean_yaw_rate_radps += std::abs(sample.yaw_rate_radps);
    metrics.peak_horizontal_speed_mps = std::max(metrics.peak_horizontal_speed_mps, sample.horizontal_speed_mps);
    metrics.peak_yaw_rate_radps = std::max(metrics.peak_yaw_rate_radps, std::abs(sample.yaw_rate_radps));
    metrics.yaw_delta_rad = wrapAngleDiff(samples.front().estimated.body_twist_state.twist_pos_rad.z,
                                          sample.estimated.body_twist_state.twist_pos_rad.z);

    if (sample.status.active_fault != FaultCode::NONE && !metrics.saw_fault) {
        metrics.saw_fault = true;
        metrics.first_fault = sample.status.active_fault;
        metrics.first_fault_step = sample.step_index;
    }
}

void finalizeMetrics(LocomotionMetrics& metrics, const std::vector<MotionSample>& samples) {
    if (samples.empty()) {
        metrics.min_support_margin_m = 0.0;
        metrics.min_model_trust = 0.0;
        metrics.min_command_scale = 0.0;
        metrics.min_cadence_scale = 0.0;
        metrics.min_governed_speed_mps = 0.0;
        metrics.min_governed_yaw_rate_radps = 0.0;
        metrics.min_step_length_m = 0.0;
        metrics.min_swing_height_m = 0.0;
        return;
    }

    metrics.duration_s = static_cast<double>(samples.size()) * kControlLoopPeriodS;
    metrics.mean_horizontal_speed_mps /= static_cast<double>(samples.size());
    metrics.mean_yaw_rate_radps /= static_cast<double>(samples.size());
    metrics.net_displacement_m = std::hypot(
        samples.back().position.x - samples.front().position.x,
        samples.back().position.y - samples.front().position.y);
    metrics.yaw_delta_rad = wrapAngleDiff(samples.front().estimated.body_twist_state.twist_pos_rad.z,
                                          samples.back().estimated.body_twist_state.twist_pos_rad.z);

    const Vec3 travel{
        samples.back().position.x - samples.front().position.x,
        samples.back().position.y - samples.front().position.y,
        0.0};
    const double denom = std::hypot(travel.x, travel.y);
    if (denom > 1.0e-9) {
        for (const MotionSample& sample : samples) {
            const Vec3 offset{
                sample.position.x - samples.front().position.x,
                sample.position.y - samples.front().position.y,
                0.0};
            const double deviation = std::abs(travel.x * offset.y - travel.y * offset.x) / denom;
            metrics.lateral_deviation_m = std::max(metrics.lateral_deviation_m, deviation);
        }
    }

    metrics.mode_segments.clear();
    if (!samples.empty()) {
        ModeSegment current{samples.front().status.active_mode, samples.front().step_index, samples.front().step_index};
        for (std::size_t i = 1; i < samples.size(); ++i) {
            if (samples[i].status.active_mode != current.mode) {
                current.end_step = samples[i - 1].step_index;
                metrics.mode_segments.push_back(current);
                ++metrics.mode_transition_count;
                if (samples[i].status.active_fault != samples[i - 1].status.active_fault) {
                    ++metrics.fault_transition_count;
                }
                current = ModeSegment{samples[i].status.active_mode, samples[i].step_index, samples[i].step_index};
            } else if (samples[i].status.active_fault != samples[i - 1].status.active_fault) {
                ++metrics.fault_transition_count;
            }
        }
        current.end_step = samples.back().step_index;
        metrics.mode_segments.push_back(current);
    }

    metrics.gait_segments.clear();
    if (!samples.empty()) {
        GaitTransitionSegment current{
            samples.front().phase_label,
            samples.front().step_index,
            samples.front().step_index,
            samples.front().step_length_m,
            samples.front().duty_factor,
        };
        std::size_t gait_segment_samples = 1;
        for (std::size_t i = 1; i < samples.size(); ++i) {
            const MotionSample& prev = samples[i - 1];
            const MotionSample& sample = samples[i];
            if (sample.phase_label != prev.phase_label) {
                current.end_step = samples[i - 1].step_index;
                current.mean_step_length_m /= static_cast<double>(gait_segment_samples);
                current.mean_duty_factor /= static_cast<double>(gait_segment_samples);
                metrics.gait_segments.push_back(current);
                current = GaitTransitionSegment{sample.phase_label, sample.step_index, sample.step_index, sample.step_length_m, sample.duty_factor};
                gait_segment_samples = 1;
            } else {
                current.mean_step_length_m += sample.step_length_m;
                current.mean_duty_factor += sample.duty_factor;
                ++gait_segment_samples;
                current.end_step = sample.step_index;
            }
        }
        current.mean_step_length_m /= static_cast<double>(gait_segment_samples);
        current.mean_duty_factor /= static_cast<double>(gait_segment_samples);
        metrics.gait_segments.push_back(current);
    }

    if (metrics.min_support_margin_m == std::numeric_limits<double>::infinity()) {
        metrics.min_support_margin_m = 0.0;
    }
}

bool runMotionSequence(RobotRuntime& runtime,
                       const std::vector<MotionPhase>& phases,
                       std::vector<MotionSample>& samples,
                       LocomotionMetrics& metrics) {
    std::size_t step_index = 0;
    double stride_cycles_accum{0.0};
    for (std::size_t phase_index = 0; phase_index < phases.size(); ++phase_index) {
        const MotionPhase& phase = phases[phase_index];
        if (phase.steps == 0) {
            continue;
        }

        for (std::size_t step_in_phase = 0; step_in_phase < phase.steps; ++step_in_phase) {
            if (phase.refresh_each_step || step_in_phase == 0) {
                runtime.setMotionIntent(makeMotionIntent(phase.motion));
            }
            runtime.busStep();
            runtime.estimatorStep();
            runtime.safetyStep();
            runtime.controlStep();

            const RobotState estimated = runtime.estimatedSnapshot();
            const GaitState gait = runtime.gaitSnapshot();
            const ControlStatus status = runtime.getStatus();
            const SafetyState safety = runtime.getSafetyState();
            const CommandGovernorState governor = runtime.commandGovernorSnapshot();
            MotionSample sample{};
            sample.step_index = step_index;
            sample.phase_index = phase_index;
            sample.phase_label = phase.label;
            sample.requested_motion = phase.motion;
            sample.status = status;
            sample.estimated = estimated;
            sample.gait = gait;
            sample.governor = governor;
            sample.safety = safety;
            sample.position = positionFromState(estimated);
            sample.horizontal_speed_mps = horizontalSpeedFromState(estimated);
            sample.body_tilt_rad = bodyTiltFromState(estimated);
            sample.body_rate_radps = bodyRateFromState(estimated);
            sample.yaw_rate_radps = estimated.body_twist_state.twist_vel_radps.z;
            sample.support_margin_m = gait.static_stability_margin_m;
            sample.stride_phase_rate_hz = gait.stride_phase_rate_hz.value;
            sample.step_length_m = gait.step_length_m;
            sample.swing_height_m = gait.swing_height_m;
            sample.duty_factor = gait.duty_factor;
            sample.model_trust = estimated.has_fusion_diagnostics ? estimated.fusion.model_trust : 1.0;
            sample.contact_mismatch_ratio =
                estimated.has_fusion_diagnostics ? estimated.fusion.residuals.contact_mismatch_ratio : 0.0;

            if (sample.status.active_mode == RobotMode::WALK && gait.stride_phase_rate_hz.value > 0.05) {
                stride_cycles_accum += gait.stride_phase_rate_hz.value * kControlLoopPeriodS;
            }
            appendSample(metrics, samples, sample);
            ++step_index;
        }
    }

    finalizeMetrics(metrics, samples);
    metrics.stride_count = std::max<std::size_t>(
        metrics.stride_count,
        static_cast<std::size_t>(std::floor(stride_cycles_accum)));
    return true;
}

control_config::ControlConfig loadPhysicsSimConfig() {
    ParsedToml parsed{};
    const std::filesystem::path config_path = resolveExistingPath({
        "config.physics-sim-wsl.txt",
        "../config.physics-sim-wsl.txt",
        "hexapod-server/config.physics-sim-wsl.txt",
    });
    if (config_path.empty()) {
        throw std::runtime_error("unable to locate config.physics-sim-wsl.txt");
    }
    TomlParser parser{};
    if (!parser.parse(config_path.string(), parsed)) {
        throw std::runtime_error("failed to parse " + config_path.string());
    }
    return control_config::fromParsedToml(parsed);
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

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>("127.0.0.1", port, 20000, 24);
    control_config::ControlConfig cfg = loadPhysicsSimConfig();
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.control_loop_trace_enabled = false;
    cfg.telemetry.enabled = false;
    cfg.replay_log.enabled = true;
    cfg.replay_log.file_path = result.replay_path.string();
    if (spec.configure) {
        spec.configure(cfg);
    }

    std::vector<MotionSample> samples{};
    LocomotionMetrics metrics{};
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

        if (!runMotionSequence(runtime, spec.phases, samples, metrics)) {
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
    const auto& m = result.metrics;
    if (!m.saw_fault) {
        reason = "long walk observability should eventually reach the safety envelope";
        return false;
    }
    if (m.first_fault != FaultCode::TIP_OVER) {
        reason = "long walk observability should trip TIP_OVER at the end of the stress window";
        return false;
    }
    if (!(m.walk_sample_count >= 500)) {
        reason = "long walk should sustain a large walking window before the fault";
        return false;
    }
    if (!(m.stride_count >= 6)) {
        reason = "long walk should accumulate many strides";
        return false;
    }
    if (!(m.path_length_m >= 0.75)) {
        reason = "long walk should accumulate path length";
        return false;
    }
    if (!(m.first_fault_step > 500 && m.first_fault_step < 2500)) {
        reason = "long walk should fault only after a sustained observation window";
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
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
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
            if (!result.passed) {
                std::cerr << spec.name << ": " << result.failure_reason << '\n';
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
