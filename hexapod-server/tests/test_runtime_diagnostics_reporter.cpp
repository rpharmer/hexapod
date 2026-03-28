#include "runtime_diagnostics_reporter.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
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

    reporter.report(status,
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

} // namespace

int main() {
    if (!testRuntimeMetricsIncludeDroppedMessageAndQueueSignals()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
