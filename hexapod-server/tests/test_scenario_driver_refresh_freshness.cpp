#include "estimator.hpp"
#include "logger.hpp"
#include "robot_control.hpp"
#include "scenario_driver.hpp"
#include "sim_hardware_bridge.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <numeric>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

struct CapturedLog {
    logging::LogLevel level;
    std::string message;
};

class CollectingSink final : public logging::LogSink {
public:
    void Write(logging::LogLevel level,
               std::string_view,
               std::string_view message,
               const logging::SourceLocation&) override {
        entries.push_back(CapturedLog{level, std::string(message)});
    }

    std::vector<CapturedLog> entries;
};

std::optional<double> extractFieldValue(const std::string& message, const std::string& field) {
    const std::string colon_token = field + ":";
    const std::string equals_token = field + "=";

    std::size_t start = message.find(colon_token);
    std::size_t value_start = std::string::npos;
    if (start != std::string::npos) {
        value_start = start + colon_token.size();
    } else {
        start = message.find(equals_token);
        if (start != std::string::npos) {
            value_start = start + equals_token.size();
        }
    }

    if (value_start == std::string::npos) {
        return std::nullopt;
    }
    std::size_t value_end = value_start;
    while (value_end < message.size() && message[value_end] != ',' && message[value_end] != '}' &&
           message[value_end] != ' ') {
        ++value_end;
    }
    try {
        return std::stod(message.substr(value_start, value_end - value_start));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<double> extractEqualsFieldValue(const std::string& message, const std::string& field) {
    const std::string token = field + "=";
    const std::size_t start = message.find(token);
    if (start == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t value_start = start + token.size();
    std::size_t value_end = value_start;
    while (value_end < message.size() && message[value_end] != ' ' && message[value_end] != ',' &&
           message[value_end] != '}') {
        ++value_end;
    }
    try {
        return std::stod(message.substr(value_start, value_end - value_start));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<std::string> latestInfoLineContaining(const std::vector<CapturedLog>& entries,
                                                    const std::string& token) {
    for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
        if (it->level == logging::LogLevel::Info && it->message.find(token) != std::string::npos) {
            return it->message;
        }
    }
    return std::nullopt;
}

std::optional<double> maxFieldValueAcrossInfoLines(const std::vector<CapturedLog>& entries,
                                                   const std::string& token,
                                                   const std::string& field) {
    std::optional<double> max_value;
    for (const auto& entry : entries) {
        if (entry.level != logging::LogLevel::Info || entry.message.find(token) == std::string::npos) {
            continue;
        }
        const auto value = extractFieldValue(entry.message, field);
        if (!value.has_value()) {
            continue;
        }
        if (!max_value.has_value() || value.value() > max_value.value()) {
            max_value = value.value();
        }
    }
    return max_value;
}

std::vector<double> collectEqualsFieldValuesAcrossInfoLines(const std::vector<CapturedLog>& entries,
                                                            const std::string& token,
                                                            const std::string& field) {
    std::vector<double> values;
    values.reserve(entries.size());
    for (const auto& entry : entries) {
        if (entry.level != logging::LogLevel::Info || entry.message.find(token) == std::string::npos) {
            continue;
        }
        const auto value = extractEqualsFieldValue(entry.message, field);
        if (value.has_value()) {
            values.push_back(*value);
        }
    }
    return values;
}

std::vector<double> collectFieldValuesAcrossInfoLines(const std::vector<CapturedLog>& entries,
                                                      const std::string& token,
                                                      const std::string& field) {
    std::vector<double> values;
    values.reserve(entries.size());
    for (const auto& entry : entries) {
        if (entry.level != logging::LogLevel::Info || entry.message.find(token) == std::string::npos) {
            continue;
        }
        const auto value = extractFieldValue(entry.message, field);
        if (value.has_value()) {
            values.push_back(*value);
        }
    }
    return values;
}

std::optional<double> minFieldValueAcrossInfoLines(const std::vector<CapturedLog>& entries,
                                                   const std::string& token,
                                                   const std::string& field) {
    std::optional<double> min_value;
    for (const auto& entry : entries) {
        if (entry.level != logging::LogLevel::Info || entry.message.find(token) == std::string::npos) {
            continue;
        }
        const auto value = extractFieldValue(entry.message, field);
        if (!value.has_value()) {
            continue;
        }
        if (!min_value.has_value() || value.value() < min_value.value()) {
            min_value = value.value();
        }
    }
    return min_value;
}

class PassthroughEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override {
        RobotState out = raw;
        out.valid = true;
        out.has_valid_flag = true;
        out.has_body_pose_state = true;
        return out;
    }
};

control_config::ControlConfig makeStrictFreshnessConfig() {
    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{100'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{100'000};
    cfg.freshness.intent.require_timestamp = true;
    cfg.freshness.intent.require_nonzero_sample_id = true;
    cfg.freshness.intent.require_monotonic_sample_id = true;
    return cfg;
}

struct DistributionSummary {
    double min{0.0};
    double p10{0.0};
    double p50{0.0};
    double p90{0.0};
    double max{0.0};
    double mean{0.0};
};

DistributionSummary summarizeDistribution(const std::vector<double>& input) {
    DistributionSummary summary{};
    if (input.empty()) {
        return summary;
    }
    std::vector<double> sorted = input;
    std::sort(sorted.begin(), sorted.end());
    auto quantile = [&](double q) {
        const double clamped_q = std::clamp(q, 0.0, 1.0);
        const std::size_t idx = static_cast<std::size_t>(clamped_q * static_cast<double>(sorted.size() - 1));
        return sorted[idx];
    };
    summary.min = sorted.front();
    summary.p10 = quantile(0.10);
    summary.p50 = quantile(0.50);
    summary.p90 = quantile(0.90);
    summary.max = sorted.back();
    summary.mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(sorted.size());
    return summary;
}

struct Scenario05ImuMatrixRunResult {
    bool run_ok{false};
    std::size_t status_samples{0};
    std::size_t fault_samples{0};
    std::vector<double> peak_command_velocity_samples_radps{};
    std::vector<double> stability_margin_samples_m{};
};

Scenario05ImuMatrixRunResult runScaledScenario05Once(bool imu_reads_enabled) {
    Scenario05ImuMatrixRunResult result{};
    const auto logger = std::make_shared<logging::AsyncLogger>(
        imu_reads_enabled ? "test-scenario-05-imu-enabled" : "test-scenario-05-imu-disabled",
        logging::LogLevel::Info,
        4096);
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(collecting_sink);

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassthroughEstimator>();
    auto cfg = makeStrictFreshnessConfig();
    cfg.runtime_imu.enable_reads = imu_reads_enabled;
    RobotControl robot(std::move(bridge), std::move(estimator), logger, cfg);

    if (!robot.init()) {
        logger->Stop();
        return result;
    }
    robot.start();

    namespace fs = std::filesystem;
    const fs::path test_file = fs::path(__FILE__);
    const fs::path scenario_path = test_file.parent_path().parent_path() / "scenarios" / "05_long_walk_observability.toml";

    ScenarioDefinition scenario{};
    std::string error;
    if (!ScenarioDriver::loadFromToml(scenario_path.string(), scenario, error, ScenarioDriver::ValidationMode::Strict)) {
        robot.stop();
        logger->Stop();
        return result;
    }

    constexpr uint64_t kScaleDivisor = 10;
    scenario.duration_ms = std::max<uint64_t>(1, scenario.duration_ms / kScaleDivisor);
    scenario.tick_ms = std::max<uint64_t>(1, scenario.tick_ms / kScaleDivisor);
    scenario.motion_ramp_ms = scenario.motion_ramp_ms / kScaleDivisor;
    for (auto& event : scenario.events) {
        event.at_ms /= kScaleDivisor;
    }

    std::atomic<bool> run_done{false};
    std::thread scenario_thread([&]() {
        result.run_ok = ScenarioDriver::run(robot, scenario, nullptr);
        run_done.store(true);
    });

    while (!run_done.load()) {
        const ControlStatus status = robot.getStatus();
        ++result.status_samples;
        if (status.active_fault != FaultCode::NONE) {
            ++result.fault_samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    scenario_thread.join();
    robot.stop();
    logger->Flush();
    result.peak_command_velocity_samples_radps =
        collectFieldValuesAcrossInfoLines(collecting_sink->entries, "runtime.metrics", "peak_velocity_radps");
    result.stability_margin_samples_m =
        collectEqualsFieldValuesAcrossInfoLines(collecting_sink->entries, "runtime.metrics", "safety_stability_margin_m");
    logger->Stop();

    return result;
}

bool test_refresh_motion_intent_monotonic_sample_ids_across_long_ramp_run() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassthroughEstimator>();
    RobotControl robot(std::move(bridge), std::move(estimator), nullptr, makeStrictFreshnessConfig());

    if (!expect(robot.init(), "robot init should succeed")) {
        return false;
    }
    robot.start();

    ScenarioDefinition scenario{};
    scenario.name = "refresh-motion-intent-monotonic";
    scenario.duration_ms = 900;
    scenario.tick_ms = 10;
    scenario.refresh_motion_intent = true;
    scenario.motion_ramp_ms = 150;
    scenario.events = {
        ScenarioEvent{.at_ms = 0,
                      .motion = ScenarioMotionIntent{.enabled = true, .mode = RobotMode::WALK, .gait = GaitType::TRIPOD,
                                                     .body_height_m = 0.20, .speed_mps = 0.06, .heading_rad = 0.0, .yaw_rad = 0.0}},
        ScenarioEvent{.at_ms = 300,
                      .motion = ScenarioMotionIntent{.enabled = true, .mode = RobotMode::WALK, .gait = GaitType::TRIPOD,
                                                     .body_height_m = 0.20, .speed_mps = 0.20, .heading_rad = 0.3, .yaw_rad = 0.0}},
        ScenarioEvent{.at_ms = 600,
                      .motion = ScenarioMotionIntent{.enabled = true, .mode = RobotMode::WALK, .gait = GaitType::TRIPOD,
                                                     .body_height_m = 0.20, .speed_mps = 0.02, .heading_rad = -0.2, .yaw_rad = 0.0}},
    };

    std::atomic<bool> run_ok{false};
    std::atomic<bool> run_done{false};
    std::thread scenario_thread([&]() {
        run_ok.store(ScenarioDriver::run(robot, scenario, nullptr));
        run_done.store(true);
    });

    bool saw_invalid_intent_fault = false;
    while (!run_done.load()) {
        const ControlStatus status = robot.getStatus();
        if (status.active_fault == FaultCode::COMMAND_TIMEOUT) {
            saw_invalid_intent_fault = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    scenario_thread.join();
    const ControlStatus final_status = robot.getStatus();
    robot.stop();

    return expect(run_ok.load(), "scenario run should succeed") &&
           expect(!saw_invalid_intent_fault, "refresh/ramp updates should not trip invalid intent freshness fault") &&
           expect(final_status.active_fault != FaultCode::COMMAND_TIMEOUT,
                  "final status should not report COMMAND_TIMEOUT");
}

bool test_scaled_scenario_05_keeps_locomotion_active_without_freshness_faults() {
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-scenario-05-locomotion", logging::LogLevel::Info, 2048);
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(collecting_sink);

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassthroughEstimator>();
    RobotControl robot(std::move(bridge), std::move(estimator), logger, makeStrictFreshnessConfig());

    if (!expect(robot.init(), "robot init should succeed (scenario 05 scaled)")) {
        logger->Stop();
        return false;
    }
    robot.start();

    namespace fs = std::filesystem;
    const fs::path test_file = fs::path(__FILE__);
    const fs::path scenario_path = test_file.parent_path().parent_path() / "scenarios" / "05_long_walk_observability.toml";

    ScenarioDefinition scenario{};
    std::string error;
    if (!expect(ScenarioDriver::loadFromToml(scenario_path.string(), scenario, error,
                                             ScenarioDriver::ValidationMode::Strict),
                "scenario 05 should parse in strict mode: " + error)) {
        robot.stop();
        logger->Stop();
        return false;
    }

    constexpr uint64_t kScaleDivisor = 10;
    scenario.duration_ms = std::max<uint64_t>(1, scenario.duration_ms / kScaleDivisor);
    scenario.tick_ms = std::max<uint64_t>(1, scenario.tick_ms / kScaleDivisor);
    scenario.motion_ramp_ms = scenario.motion_ramp_ms / kScaleDivisor;
    for (auto& event : scenario.events) {
        event.at_ms /= kScaleDivisor;
    }

    std::atomic<bool> run_ok{false};
    std::atomic<bool> run_done{false};
    std::thread scenario_thread([&]() {
        run_ok.store(ScenarioDriver::run(robot, scenario, nullptr));
        run_done.store(true);
    });

    std::size_t status_samples = 0;
    std::size_t walk_samples = 0;
    std::size_t command_timeout_samples = 0;
    while (!run_done.load()) {
        const ControlStatus status = robot.getStatus();
        ++status_samples;
        if (status.active_mode == RobotMode::WALK && status.active_fault == FaultCode::NONE) {
            ++walk_samples;
        }
        if (status.active_fault == FaultCode::COMMAND_TIMEOUT) {
            ++command_timeout_samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    scenario_thread.join();
    robot.stop();
    logger->Flush();

    const auto latest_metrics = latestInfoLineContaining(collecting_sink->entries, "locomotion_diag={");
    const auto peak_total_body_movement =
        maxFieldValueAcrossInfoLines(collecting_sink->entries, "locomotion_diag={", "total_body_movement_m");
    logger->Stop();

    const double walk_ratio = status_samples > 0
                                  ? static_cast<double>(walk_samples) / static_cast<double>(status_samples)
                                  : 0.0;
    return expect(run_ok.load(), "scaled scenario 05 run should succeed") &&
           expect(command_timeout_samples == 0,
                  "scaled scenario 05 should not trigger COMMAND_TIMEOUT freshness faults") &&
           expect(walk_ratio > 0.30,
                  "scaled scenario 05 should spend most of runtime in WALK without faults") &&
           expect(latest_metrics.has_value(),
                  "scaled scenario 05 should emit runtime locomotion diagnostics") &&
           expect(peak_total_body_movement.has_value(),
                  "scaled scenario 05 should expose movement distance metric for locomotion diagnosis");
}

bool test_scaled_scenario_05_imu_read_matrix_compares_fault_rate_velocity_and_stability_distribution() {
    const Scenario05ImuMatrixRunResult imu_disabled = runScaledScenario05Once(false);
    const Scenario05ImuMatrixRunResult imu_enabled = runScaledScenario05Once(true);

    const double fault_rate_disabled = imu_disabled.status_samples > 0
                                           ? static_cast<double>(imu_disabled.fault_samples) /
                                                 static_cast<double>(imu_disabled.status_samples)
                                           : 1.0;
    const double fault_rate_enabled = imu_enabled.status_samples > 0
                                          ? static_cast<double>(imu_enabled.fault_samples) /
                                                static_cast<double>(imu_enabled.status_samples)
                                          : 1.0;
    const DistributionSummary peak_vel_disabled = summarizeDistribution(imu_disabled.peak_command_velocity_samples_radps);
    const DistributionSummary peak_vel_enabled = summarizeDistribution(imu_enabled.peak_command_velocity_samples_radps);
    const DistributionSummary stability_disabled = summarizeDistribution(imu_disabled.stability_margin_samples_m);
    const DistributionSummary stability_enabled = summarizeDistribution(imu_enabled.stability_margin_samples_m);

    return expect(imu_disabled.run_ok, "scaled scenario 05 should run with Runtime.Imu.EnableReads=false") &&
           expect(imu_enabled.run_ok, "scaled scenario 05 should run with Runtime.Imu.EnableReads=true (sim/noop)") &&
           expect(!imu_disabled.peak_command_velocity_samples_radps.empty(),
                  "imu-disabled run should emit runtime.metrics peak command velocity samples") &&
           expect(!imu_enabled.peak_command_velocity_samples_radps.empty(),
                  "imu-enabled run should emit runtime.metrics peak command velocity samples") &&
           expect(!imu_disabled.stability_margin_samples_m.empty(),
                  "imu-disabled run should emit runtime.metrics stability margin samples") &&
           expect(!imu_enabled.stability_margin_samples_m.empty(),
                  "imu-enabled run should emit runtime.metrics stability margin samples") &&
           expect(fault_rate_disabled < 0.05,
                  "imu-disabled scenario 05 fault rate should remain low") &&
           expect(fault_rate_enabled < 0.05,
                  "imu-enabled scenario 05 fault rate should remain low") &&
           expect(std::abs(fault_rate_disabled - fault_rate_enabled) < 0.03,
                  "imu read matrix fault-rate delta should remain small for sim/noop IMU") &&
           expect(std::abs(peak_vel_disabled.p90 - peak_vel_enabled.p90) < 0.25,
                  "imu read matrix peak command velocity distributions should remain close at p90") &&
           expect(std::abs(stability_disabled.p10 - stability_enabled.p10) < 0.02 &&
                      std::abs(stability_disabled.p50 - stability_enabled.p50) < 0.02 &&
                      std::abs(stability_disabled.p90 - stability_enabled.p90) < 0.02,
                  "imu read matrix stability margin distribution (p10/p50/p90) should remain close");
}

bool test_scaled_scenario_10_long_duration_bounds_faults_velocity_stability_and_freshness() {
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-scenario-10-cruise-bounds", logging::LogLevel::Info, 4096);
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(collecting_sink);

    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<PassthroughEstimator>();
    RobotControl robot(std::move(bridge), std::move(estimator), logger, makeStrictFreshnessConfig());

    if (!expect(robot.init(), "robot init should succeed (scenario 10 scaled)")) {
        logger->Stop();
        return false;
    }
    robot.start();

    namespace fs = std::filesystem;
    const fs::path test_file = fs::path(__FILE__);
    const fs::path scenario_path = test_file.parent_path().parent_path() / "scenarios" / "10_sustained_fast_cruise.toml";

    ScenarioDefinition scenario{};
    std::string error;
    if (!expect(ScenarioDriver::loadFromToml(scenario_path.string(), scenario, error,
                                             ScenarioDriver::ValidationMode::Strict),
                "scenario 10 should parse in strict mode: " + error)) {
        robot.stop();
        logger->Stop();
        return false;
    }

    constexpr uint64_t kScaleDivisor = 30;
    scenario.duration_ms = std::max<uint64_t>(1, scenario.duration_ms / kScaleDivisor);
    scenario.tick_ms = std::max<uint64_t>(1, scenario.tick_ms / kScaleDivisor);
    scenario.motion_ramp_ms = scenario.motion_ramp_ms / kScaleDivisor;
    for (auto& event : scenario.events) {
        event.at_ms /= kScaleDivisor;
    }

    std::atomic<bool> run_ok{false};
    std::atomic<bool> run_done{false};
    std::thread scenario_thread([&]() {
        run_ok.store(ScenarioDriver::run(robot, scenario, nullptr));
        run_done.store(true);
    });

    std::size_t samples = 0;
    std::size_t walk_samples = 0;
    std::size_t fault_samples = 0;
    std::size_t command_timeout_samples = 0;
    while (!run_done.load()) {
        const ControlStatus status = robot.getStatus();
        ++samples;
        if (status.active_mode == RobotMode::WALK) {
            ++walk_samples;
        }
        if (status.active_fault != FaultCode::NONE) {
            ++fault_samples;
        }
        if (status.active_fault == FaultCode::COMMAND_TIMEOUT) {
            ++command_timeout_samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    scenario_thread.join();
    robot.stop();
    logger->Flush();

    const auto peak_joint_velocity_radps =
        maxFieldValueAcrossInfoLines(collecting_sink->entries, "joint_cmd_diag={", "peak_velocity_radps");
    const auto peak_foot_velocity_mps =
        maxFieldValueAcrossInfoLines(collecting_sink->entries, "leg_target_diag={", "peak_foot_vel_mps");
    const auto min_stability_margin_m =
        minFieldValueAcrossInfoLines(collecting_sink->entries, "runtime.metrics", "safety_stability_margin_m");
    const auto freshness_rejects_total =
        maxFieldValueAcrossInfoLines(collecting_sink->entries, "runtime.metrics", "freshness_rejects_total");
    logger->Stop();

    const double walk_ratio =
        samples > 0 ? static_cast<double>(walk_samples) / static_cast<double>(samples) : 0.0;

    constexpr double kMaxAllowedPeakJointVelocityRadps = 25.0;
    constexpr double kMaxAllowedPeakFootVelocityMps = 2.0;
    constexpr double kMinAllowedSafetyStabilityMarginM = 0.005;

    return expect(run_ok.load(), "scaled scenario 10 run should succeed") &&
           expect(command_timeout_samples == 0,
                  "scaled scenario 10 should keep command freshness valid (no COMMAND_TIMEOUT samples)") &&
           expect(fault_samples == 0, "scaled scenario 10 should not latch unexpected non-NONE faults") &&
           expect(walk_ratio > 0.40, "scaled scenario 10 should spend significant runtime in WALK mode") &&
           expect(peak_joint_velocity_radps.has_value() && *peak_joint_velocity_radps < kMaxAllowedPeakJointVelocityRadps,
                  "scaled scenario 10 should keep peak joint velocity within bounded regression threshold") &&
           expect(peak_foot_velocity_mps.has_value() && *peak_foot_velocity_mps < kMaxAllowedPeakFootVelocityMps,
                  "scaled scenario 10 should keep peak foot target velocity within bounded regression threshold") &&
           expect(min_stability_margin_m.has_value() && *min_stability_margin_m > kMinAllowedSafetyStabilityMarginM,
                  "scaled scenario 10 should avoid excessive stability margin degradation") &&
           expect(freshness_rejects_total.has_value() && *freshness_rejects_total == 0.0,
                  "scaled scenario 10 should sustain zero freshness rejects");
}

} // namespace

int main() {
    if (!test_refresh_motion_intent_monotonic_sample_ids_across_long_ramp_run()) {
        return EXIT_FAILURE;
    }
    if (!test_scaled_scenario_05_keeps_locomotion_active_without_freshness_faults()) {
        return EXIT_FAILURE;
    }
    if (!test_scaled_scenario_05_imu_read_matrix_compares_fault_rate_velocity_and_stability_distribution()) {
        return EXIT_FAILURE;
    }
    if (!test_scaled_scenario_10_long_duration_bounds_faults_velocity_stability_and_freshness()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
