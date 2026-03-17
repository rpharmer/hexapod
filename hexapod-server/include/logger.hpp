#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace logging {

enum class LogLevel {
    Trace = 0,
    Debug,
    Info,
    Warn,
    Error,
    Fatal
};

const char* ToString(LogLevel level);

struct SourceLocation {
    const char* file;
    int line;
    const char* function;
};

struct LogMessage {
    LogLevel level;
    std::string loggerName;
    std::string text;
    SourceLocation location;
};

class LogSink {
public:
    virtual ~LogSink() = default;

    virtual void Write(
        LogLevel level,
        std::string_view loggerName,
        std::string_view message,
        const SourceLocation& location) = 0;

    virtual void Flush() {}
};

class ConsoleSink final : public LogSink {
public:
    void Write(
        LogLevel level,
        std::string_view loggerName,
        std::string_view message,
        const SourceLocation& location) override;

    void Flush() override;

private:
    std::mutex mutex_;
};

class FileSink final : public LogSink {
public:
    explicit FileSink(const std::string& path, bool append = true);
    ~FileSink() override;

    void Write(
        LogLevel level,
        std::string_view loggerName,
        std::string_view message,
        const SourceLocation& location) override;

    void Flush() override;
    bool IsOpen() const;

private:
    std::ofstream file_;
    mutable std::mutex mutex_;
};

class AsyncLogger {
public:
    explicit AsyncLogger(
        std::string name,
        LogLevel minLevel = LogLevel::Info,
        std::size_t maxQueueSize = 10000);

    ~AsyncLogger();

    AsyncLogger(const AsyncLogger&) = delete;
    AsyncLogger& operator=(const AsyncLogger&) = delete;

    AsyncLogger(AsyncLogger&&) = delete;
    AsyncLogger& operator=(AsyncLogger&&) = delete;

    void AddSink(std::shared_ptr<LogSink> sink);
    void SetLevel(LogLevel minLevel);
    LogLevel GetLevel() const;

    void Log(
        LogLevel level,
        std::string_view message,
        const SourceLocation& location);

    template <typename... Args>
    void LogStream(
        LogLevel level,
        const SourceLocation& location,
        Args&&... args) {
        if (level < GetLevel()) {
            return;
        }

        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        Log(level, oss.str(), location);
    }

    void Flush();
    void Stop();

    std::size_t DroppedMessageCount() const;
    const std::string& Name() const;

private:
    void WorkerLoop();
    void WriteToSinks(const LogMessage& msg);

    std::string name_;
    LogLevel minLevel_;
    std::vector<std::shared_ptr<LogSink>> sinks_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable drainedCv_;
    std::deque<LogMessage> queue_;

    std::thread worker_;
    std::size_t maxQueueSize_;
    std::size_t droppedMessages_;
    bool stopRequested_;
    bool workerRunning_;
    
    bool workerBusy_ = false;
};

} // namespace logging

#define LOG_SOURCE_LOCATION ::logging::SourceLocation{__FILE__, __LINE__, __func__}

#define LOG_TRACE(logger, ...) (logger)->LogStream(::logging::LogLevel::Trace, LOG_SOURCE_LOCATION, __VA_ARGS__)
#define LOG_DEBUG(logger, ...) (logger)->LogStream(::logging::LogLevel::Debug, LOG_SOURCE_LOCATION, __VA_ARGS__)
#define LOG_INFO(logger, ...)  (logger)->LogStream(::logging::LogLevel::Info,  LOG_SOURCE_LOCATION, __VA_ARGS__)
#define LOG_WARN(logger, ...)  (logger)->LogStream(::logging::LogLevel::Warn,  LOG_SOURCE_LOCATION, __VA_ARGS__)
#define LOG_ERROR(logger, ...) (logger)->LogStream(::logging::LogLevel::Error, LOG_SOURCE_LOCATION, __VA_ARGS__)
#define LOG_FATAL(logger, ...) (logger)->LogStream(::logging::LogLevel::Fatal, LOG_SOURCE_LOCATION, __VA_ARGS__)