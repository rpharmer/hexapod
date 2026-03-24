#include "runtime_teardown.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
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

        bool all_tokens_present = true;
        for (const std::string& token : tokens) {
            if (entry.message.find(token) == std::string::npos) {
                all_tokens_present = false;
                break;
            }
        }

        if (all_tokens_present) {
            return true;
        }
    }

    return false;
}

bool testCleanShutdownLogsInfoSummary() {
    const auto sink = std::make_shared<CollectingSink>();
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-teardown", logging::LogLevel::Trace, 1024);
    logger->AddSink(sink);

    app::runRuntimeTeardown(logger, [] {}, 0);
    logger->Flush();

    const bool found = containsEntry(
        sink->entries,
        logging::LogLevel::Info,
        {"shutdown_summary", "success=1", "teardown_ok=1", "dropped_messages=0", "runner_rc=0"});

    logger->Stop();
    return expect(found, "clean shutdown should emit INFO summary tokens");
}

bool testTeardownFailureLogsErrorSummary() {
    const auto sink = std::make_shared<CollectingSink>();
    const auto logger = std::make_shared<logging::AsyncLogger>(
        "test-runtime-teardown", logging::LogLevel::Trace, 1024);
    logger->AddSink(sink);

    app::runRuntimeTeardown(
        logger,
        [] { throw std::runtime_error("stop failed"); },
        1);
    logger->Flush();

    const bool found = containsEntry(
        sink->entries,
        logging::LogLevel::Error,
        {"shutdown_summary", "success=0", "teardown_ok=0", "dropped_messages=0", "runner_rc=1"});

    logger->Stop();
    return expect(found, "teardown failure should emit ERROR summary tokens");
}

} // namespace

int main() {
    if (!testCleanShutdownLogsInfoSummary()) {
        return EXIT_FAILURE;
    }

    if (!testTeardownFailureLogsErrorSummary()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
