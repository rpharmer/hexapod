#include "replay_logger.hpp"

#include <fstream>
#include <mutex>

namespace replay {
namespace {

class NoopReplayLogger final : public IReplayLogger {
public:
    void write(const replay_json::ReplayTelemetryRecord&) override {}
};

class FileReplayLogger final : public IReplayLogger {
public:
    FileReplayLogger(std::string path, std::shared_ptr<logging::AsyncLogger> logger)
        : logger_(std::move(logger))
    {
        file_.open(path, std::ios::out | std::ios::app);
        if (!file_.is_open() && logger_) {
            LOG_WARN(logger_, "replay logging disabled: failed to open '", path, "'");
        }
    }

    void write(const replay_json::ReplayTelemetryRecord& record) override
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!file_.is_open()) {
            return;
        }
        file_ << replay_json::serializeReplayTelemetryRecord(record) << '\n';
        file_.flush();
    }

private:
    std::ofstream file_{};
    std::mutex mutex_{};
    std::shared_ptr<logging::AsyncLogger> logger_{};
};

} // namespace

std::unique_ptr<IReplayLogger> makeNoopReplayLogger()
{
    return std::make_unique<NoopReplayLogger>();
}

std::unique_ptr<IReplayLogger> makeFileReplayLogger(const std::string& path,
                                                    std::shared_ptr<logging::AsyncLogger> logger)
{
    if (path.empty()) {
        if (logger) {
            LOG_WARN(logger, "replay logging disabled: empty file path");
        }
        return makeNoopReplayLogger();
    }
    return std::make_unique<FileReplayLogger>(path, std::move(logger));
}

} // namespace replay
