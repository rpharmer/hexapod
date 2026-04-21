#include "runtime_diagnostics_reporter.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "logger.hpp"

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
        entries.push_back(CapturedLog{level, std::string{message}});
    }

    std::vector<CapturedLog> entries;
};

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool containsEntry(const std::vector<CapturedLog>& entries,
                   logging::LogLevel level,
                   const std::vector<std::string>& tokens) {
    for (const auto& entry : entries) {
        if (entry.level != level) {
            continue;
        }

        bool matches = true;
        for (const std::string& token : tokens) {
            if (entry.message.find(token) == std::string::npos) {
                matches = false;
                break;
            }
        }
        if (matches) {
            return true;
        }
    }
    return false;
}

bool testDiagnosticsReporterIncludesProcessResources() {
    const auto sink = std::make_shared<CollectingSink>();
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-diagnostics", logging::LogLevel::Trace, 1024);
    logger->AddSink(sink);

    FreshnessPolicy freshness_policy{};
    RuntimeDiagnosticsReporter reporter(logger, freshness_policy);

    const resource_monitoring::ProcessResourceSnapshot resources{
        .cpu_percent = 12.5,
        .cpu_window_us = 500'000,
        .rss_bytes = 1'234'567,
        .vms_bytes = 8'765'432,
    };
    const ControlStatus status{
        .active_mode = RobotMode::STAND,
        .estimator_valid = true,
        .bus_ok = true,
        .active_fault = FaultCode::NONE,
        .loop_counter = 42,
    };

    reporter.report(status, std::nullopt, 42, 4000, 125, 3, 7, resources);
    logger->Flush();

    const bool found = containsEntry(
        sink->entries,
        logging::LogLevel::Debug,
        {"runtime.metrics", "process_resource=", "cpu_percent=12.5", "cpu_window_ms=500",
         "rss_bytes=1234567", "vms_bytes=8765432"});

    logger->Stop();
    return expect(found, "runtime diagnostics should include process resource metrics");
}

} // namespace

int main() {
    if (!testDiagnosticsReporterIncludesProcessResources()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
