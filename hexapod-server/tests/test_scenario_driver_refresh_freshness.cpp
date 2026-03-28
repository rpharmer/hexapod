#include "estimator.hpp"
#include "logger.hpp"
#include "robot_control.hpp"
#include "scenario_driver.hpp"
#include "sim_hardware_bridge.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
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
    const std::string token = field + ":";
    const std::size_t start = message.find(token);
    if (start == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t value_start = start + token.size();
    std::size_t value_end = value_start;
    while (value_end < message.size() && message[value_end] != ',' && message[value_end] != '}') {
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

} // namespace

int main() {
    if (!test_refresh_motion_intent_monotonic_sample_ids_across_long_ramp_run()) {
        return EXIT_FAILURE;
    }
    if (!test_scaled_scenario_05_keeps_locomotion_active_without_freshness_faults()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
