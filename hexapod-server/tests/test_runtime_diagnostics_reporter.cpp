#include "runtime_diagnostics_reporter.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

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

class BlockingSink final : public logging::LogSink {
public:
    void Write(logging::LogLevel,
               std::string_view,
               std::string_view,
               const logging::SourceLocation&) override {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return released_; });
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        cv_.notify_all();
    }

private:
    std::mutex mutex_{};
    std::condition_variable cv_{};
    bool released_{false};
};

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool containsTokens(const std::vector<CapturedLog>& entries,
                    logging::LogLevel level,
                    const std::vector<std::string>& tokens) {
    for (const auto& entry : entries) {
        if (entry.level != level) {
            continue;
        }
        bool all_found = true;
        for (const auto& token : tokens) {
            if (entry.message.find(token) == std::string::npos) {
                all_found = false;
                break;
            }
        }
        if (all_found) {
            return true;
        }
    }
    return false;
}

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

bool testRuntimeMetricsIncludeDroppedMessageAndQueueSignals() {
    FreshnessPolicy freshness_policy{};
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-diag-reporter", logging::LogLevel::Trace, 2);
    const auto blocking_sink = std::make_shared<BlockingSink>();
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(blocking_sink);
    logger->AddSink(collecting_sink);

    for (int idx = 0; idx < 32; ++idx) {
        logger->Log(logging::LogLevel::Info, "preload_message", LOG_SOURCE_LOCATION);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    blocking_sink->release();
    logger->Flush();

    const std::size_t dropped_before_report = logger->DroppedMessageCount();
    if (!expect(dropped_before_report > 0, "preload traffic should generate dropped messages")) {
        logger->Stop();
        return false;
    }

    RuntimeDiagnosticsReporter reporter(logger, freshness_policy);
    ControlStatus status{};
    status.active_mode = RobotMode::STAND;
    status.estimator_valid = true;
    status.active_fault = FaultCode::NONE;
    RobotState estimated_state{};
    SafetyState safety_state{};

    reporter.report(status,
                    estimated_state,
                    safety_state,
                    std::nullopt,
                    100,
                    1000,
                    50,
                    LoopTimingRollingMetrics{},
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0);
    logger->Flush();

    const bool has_runtime_diag = containsTokens(
        collecting_sink->entries,
        logging::LogLevel::Info,
        {"runtime.metrics",
         "logger_diag={dropped_messages:",
         "queue_depth:",
         "queue_capacity:",
         "worker_busy:",
         "logger_diag={dropped_messages:" + std::to_string(dropped_before_report)});
    logger->Stop();

    return expect(has_runtime_diag,
                  "runtime.metrics should include non-zero dropped_messages and queue state fields");
}

bool testLocomotionDiagnosticsEmitSaneRanges() {
    FreshnessPolicy freshness_policy{};
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-locomotion-diag", logging::LogLevel::Trace, 256);
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(collecting_sink);

    RuntimeDiagnosticsReporter reporter(logger, freshness_policy);
    ControlStatus status{};
    status.active_mode = RobotMode::WALK;
    status.estimator_valid = true;
    status.active_fault = FaultCode::NONE;

    SafetyState safety_state{};
    safety_state.stable = true;

    JointTargets joints{};
    LegTargets leg_targets{};
    RobotState est{};
    est.valid = true;
    est.has_body_pose_state = true;

    uint64_t now_us_value = 1'000'000;
    constexpr double kPiValue = 3.14159265358979323846;
    for (int step = 0; step < 9; ++step) {
        const double progress = static_cast<double>(step) / 8.0;
        if (step == 0) {
            status.dynamic_gait.leg_in_stance.fill(true);
            leg_targets.feet[0].pos_body_m = Vec3{0.0, 0.0, 0.0};
        } else if (step <= 4) {
            status.dynamic_gait.leg_in_stance.fill(false);
            const double alpha = static_cast<double>(step - 1) / 3.0;
            leg_targets.feet[0].pos_body_m = Vec3{0.10 * alpha, 0.0, 0.03 * std::sin(alpha * kPiValue)};
        } else {
            status.dynamic_gait.leg_in_stance.fill(true);
            leg_targets.feet[0].pos_body_m = Vec3{0.10, 0.0, 0.0};
        }
        for (int leg = 1; leg < kNumLegs; ++leg) {
            leg_targets.feet[leg].pos_body_m = leg_targets.feet[0].pos_body_m;
        }

        est.body_pose_state.body_trans_m = Vec3{0.02 * progress, 0.0, 0.0};

        reporter.recordControlOutputs(joints, status, TimePointUs{now_us_value}, &leg_targets);
        reporter.report(status,
                        est,
                        safety_state,
                        std::nullopt,
                        static_cast<uint64_t>(step + 1),
                        4000,
                        200,
                        LoopTimingRollingMetrics{},
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0);
        now_us_value += 20'000;
    }
    logger->Flush();

    const auto latest = latestInfoLineContaining(collecting_sink->entries, "locomotion_diag={");
    if (!expect(latest.has_value(), "runtime.metrics should include locomotion_diag payload")) {
        logger->Stop();
        return false;
    }

    const auto strides_taken = extractFieldValue(*latest, "strides_taken");
    const auto stride_length_avg = extractFieldValue(*latest, "stride_length_avg_m");
    const auto stride_height_max = extractFieldValue(*latest, "stride_height_max_m");
    const auto foothold_directness_avg = extractFieldValue(*latest, "foothold_directness_avg");
    const auto total_body_movement = extractFieldValue(*latest, "total_body_movement_m");

    logger->Stop();
    return expect(strides_taken.has_value() && *strides_taken >= 1.0,
                  "locomotion diagnostics should report at least one completed stride") &&
           expect(stride_length_avg.has_value() && *stride_length_avg > 0.05 && *stride_length_avg < 0.15,
                  "stride length avg should be within a sane synthetic-walk range") &&
           expect(stride_height_max.has_value() && *stride_height_max > 0.015 && *stride_height_max < 0.05,
                  "stride height max should be within sane synthetic-walk range") &&
           expect(foothold_directness_avg.has_value() && *foothold_directness_avg >= 0.0 &&
                      *foothold_directness_avg <= 1.0,
                  "foothold directness avg should stay normalized in [0, 1]") &&
           expect(total_body_movement.has_value() && *total_body_movement > 0.01,
                  "total body movement should accumulate over reported walk samples");
}

bool testPhaseRateUsesWrappedUnitIntervalDistance() {
    FreshnessPolicy freshness_policy{};
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-phase-wrap-diag", logging::LogLevel::Trace, 256);
    const auto collecting_sink = std::make_shared<CollectingSink>();
    logger->AddSink(collecting_sink);

    RuntimeDiagnosticsReporter reporter(logger, freshness_policy);
    ControlStatus status{};
    status.active_mode = RobotMode::WALK;
    status.estimator_valid = true;
    status.active_fault = FaultCode::NONE;
    status.dynamic_gait.leg_phase.fill(0.99);

    JointTargets joints{};
    LegTargets leg_targets{};

    constexpr uint64_t kStartTimeUs = 1'000'000;
    constexpr uint64_t kRealisticDtUs = 20'000;
    constexpr double kRealisticDtS = static_cast<double>(kRealisticDtUs) / 1'000'000.0;
    constexpr double kWrappedDelta = 0.02;
    constexpr double kNaiveSpikeDelta = 0.98;

    reporter.recordControlOutputs(joints, status, TimePointUs{kStartTimeUs}, &leg_targets);

    status.dynamic_gait.leg_phase.fill(0.01);
    reporter.recordControlOutputs(joints, status, TimePointUs{kStartTimeUs + kRealisticDtUs}, &leg_targets);

    RobotState estimated_state{};
    estimated_state.valid = true;
    estimated_state.has_body_pose_state = true;
    SafetyState safety_state{};
    safety_state.stable = true;

    reporter.report(status,
                    estimated_state,
                    safety_state,
                    std::nullopt,
                    2,
                    1000,
                    100,
                    LoopTimingRollingMetrics{},
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0);
    logger->Flush();

    const auto latest = latestInfoLineContaining(collecting_sink->entries, "runtime.metrics");
    if (!expect(latest.has_value(), "runtime.metrics should include gait_variability_diag payload")) {
        logger->Stop();
        return false;
    }
    const auto peak_phase_delta_per_s = extractFieldValue(*latest, "peak_phase_delta_per_s");
    logger->Stop();

    const double expected_wrapped_rate = kWrappedDelta / kRealisticDtS;
    const double naive_spike_rate = kNaiveSpikeDelta / kRealisticDtS;

    return expect(peak_phase_delta_per_s.has_value(), "gait variability should report peak_phase_delta_per_s") &&
           expect(std::abs(*peak_phase_delta_per_s - expected_wrapped_rate) < 0.05,
                  "phase-rate should use shortest wrapped distance for 0.99->0.01 transition") &&
           expect(*peak_phase_delta_per_s < naive_spike_rate * 0.1,
                  "phase-rate should not spike toward naive near-1/dt wrap discontinuity");
}

} // namespace

int main() {
    if (!testRuntimeMetricsIncludeDroppedMessageAndQueueSignals()) {
        return EXIT_FAILURE;
    }
    if (!testLocomotionDiagnosticsEmitSaneRanges()) {
        return EXIT_FAILURE;
    }
    if (!testPhaseRateUsesWrappedUnitIntervalDistance()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
