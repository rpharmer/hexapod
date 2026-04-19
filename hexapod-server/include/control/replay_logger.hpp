#pragma once

#include "logger.hpp"
#include "replay_json.hpp"

#include <memory>
#include <string>

namespace replay {

class IReplayLogger {
public:
    virtual ~IReplayLogger() = default;

    virtual void write(const replay_json::ReplayTelemetryRecord& record) = 0;
};

std::unique_ptr<IReplayLogger> makeNoopReplayLogger();
std::unique_ptr<IReplayLogger> makeFileReplayLogger(const std::string& path,
                                                    std::shared_ptr<logging::AsyncLogger> logger);

} // namespace replay
