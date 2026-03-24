#include "shutdown_summary.hpp"

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

bool entryMatches(const CapturedLog& entry,
                  logging::LogLevel level,
                  const std::vector<std::string>& tokens) {
    if (entry.level != level) {
        return false;
    }

    for (const std::string& token : tokens) {
        if (entry.message.find(token) == std::string::npos) {
            return false;
        }
    }

    return true;
}

bool containsEntry(const std::vector<CapturedLog>& entries,
                   logging::LogLevel level,
                   const std::vector<std::string>& tokens) {
    for (const auto& entry : entries) {
        if (entryMatches(entry, level, tokens)) {
            return true;
        }
    }
    return false;
}

bool test_clean_shutdown_logs_info_summary() {
    const auto sink = std::make_shared<CollectingSink>();
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-shutdown-summary", logging::LogLevel::Trace, 1024);
    logger->AddSink(sink);

    app::logShutdownSummary(logger, true, 0, 0);
    logger->Flush();

    const bool found = containsEntry(
        sink->entries,
        logging::LogLevel::Info,
        {"shutdown_summary", "success=1", "teardown_ok=1", "dropped_messages=0", "runner_rc=0"});

    logger->Stop();
    return expect(found, "clean shutdown should emit INFO shutdown_summary tokens");
}

bool test_teardown_failure_logs_error_summary() {
    const auto sink = std::make_shared<CollectingSink>();
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-shutdown-summary", logging::LogLevel::Trace, 1024);
    logger->AddSink(sink);

    app::logShutdownSummary(logger, false, 0, 1);
    logger->Flush();

    const bool found = containsEntry(
        sink->entries,
        logging::LogLevel::Error,
        {"shutdown_summary", "success=0", "teardown_ok=0", "dropped_messages=0", "runner_rc=1"});

    logger->Stop();
    return expect(found, "teardown failure should emit ERROR shutdown_summary tokens");
}

} // namespace

int main() {
    if (!test_clean_shutdown_logs_info_summary()) {
        return EXIT_FAILURE;
    }

    if (!test_teardown_failure_logs_error_summary()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
