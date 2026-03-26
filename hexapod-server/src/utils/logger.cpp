#include "logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace logging {
namespace {

std::mutex g_defaultLoggerMutex;
std::shared_ptr<AsyncLogger> g_defaultLogger;

std::string CurrentTimestamp() {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto time = system_clock::to_time_t(now);

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &time);
#else
    localtime_r(&time, &tm);
#endif

    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.'
        << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string FormatMessage(
    LogLevel level,
    std::string_view loggerName,
    std::string_view message,
    const SourceLocation& location) {
    std::ostringstream oss;
    oss << "[" << CurrentTimestamp() << "] "
        << "[" << ToString(level) << "] "
        << "[" << loggerName << "] "
        << message
        << " (" << location.file << ":" << location.line
        << ", " << location.function << ")";
    return oss.str();
}

} // namespace

const char* ToString(LogLevel level) {
    switch (level) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
        case LogLevel::Fatal: return "FATAL";
        default:              return "UNKNOWN";
    }
}

void ConsoleSink::Write(
    LogLevel level,
    std::string_view loggerName,
    std::string_view message,
    const SourceLocation& location) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostream& out = (level >= LogLevel::Error) ? std::cerr : std::cout;
    out << FormatMessage(level, loggerName, message, location) << '\n';
}

void ConsoleSink::Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout.flush();
    std::cerr.flush();
}

FileSink::FileSink(const std::string& path, bool append) {
    auto mode = std::ios::out;
    mode |= append ? std::ios::app : std::ios::trunc;

    file_.open(path, mode);
    if (!file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + path);
    }
}

FileSink::~FileSink() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

bool FileSink::IsOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_.is_open();
}

void FileSink::Write(
    LogLevel level,
    std::string_view loggerName,
    std::string_view message,
    const SourceLocation& location) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) {
        return;
    }

    file_ << FormatMessage(level, loggerName, message, location) << '\n';
}

void FileSink::Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.flush();
    }
}


void SetDefaultLogger(std::shared_ptr<AsyncLogger> logger) {
    std::lock_guard<std::mutex> lock(g_defaultLoggerMutex);
    g_defaultLogger = std::move(logger);
}

std::shared_ptr<AsyncLogger> GetDefaultLogger() {
    std::lock_guard<std::mutex> lock(g_defaultLoggerMutex);
    return g_defaultLogger;
}

AsyncLogger::AsyncLogger(
    std::string name,
    LogLevel minLevel,
    std::size_t maxQueueSize)
    : name_(std::move(name)),
      minLevel_(minLevel),
      worker_(),
      maxQueueSize_(maxQueueSize),
      droppedMessages_(0),
      stopRequested_(false),
      workerRunning_(true) {
    worker_ = std::thread(&AsyncLogger::WorkerLoop, this);
}

AsyncLogger::~AsyncLogger() {
    Stop();
}

void AsyncLogger::AddSink(std::shared_ptr<LogSink> sink) {
    std::lock_guard<std::mutex> lock(mutex_);
    sinks_.push_back(std::move(sink));
}

void AsyncLogger::SetLevel(LogLevel minLevel) {
    std::lock_guard<std::mutex> lock(mutex_);
    minLevel_ = minLevel;
}

LogLevel AsyncLogger::GetLevel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return minLevel_;
}

void AsyncLogger::Log(
    LogLevel level,
    std::string_view message,
    const SourceLocation& location) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopRequested_ || level < minLevel_) {
        return;
    }

    if (queue_.size() >= maxQueueSize_) {
        ++droppedMessages_;
        return;
    }

    queue_.push_back(LogMessage{
        level,
        name_,
        std::string(message),
        location
    });

    cv_.notify_one();
}

void AsyncLogger::Flush() {
    std::unique_lock<std::mutex> lock(mutex_);
    drainedCv_.wait(lock, [this]() {
        return queue_.empty() && !workerBusy_;
    });

    auto sinksCopy = sinks_;
    lock.unlock();

    for (const auto& sink : sinksCopy) {
        if (sink) {
            sink->Flush();
        }
    }
}

void AsyncLogger::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopRequested_) {
            return;
        }
        stopRequested_ = true;
    }

    cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }
}

std::size_t AsyncLogger::DroppedMessageCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return droppedMessages_;
}

AsyncLogger::QueueState AsyncLogger::CurrentQueueState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return QueueState{
        queue_.size(),
        maxQueueSize_,
        workerBusy_
    };
}

const std::string& AsyncLogger::Name() const {
    return name_;
}

void AsyncLogger::WorkerLoop() {
    for (;;) {
        LogMessage msg;
        bool hasMessage = false;
        std::vector<std::shared_ptr<LogSink>> sinksCopy;

        {
          std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() {
                return stopRequested_ || !queue_.empty();
            });

            if (stopRequested_ && queue_.empty()) {
                workerRunning_ = false;
                drainedCv_.notify_all();
                break;
            }

            workerBusy_ = true;
            msg = std::move(queue_.front());
            queue_.pop_front();
            sinksCopy = sinks_;
            hasMessage = true;

            if (queue_.empty()) {
                drainedCv_.notify_all();
            }
        }

        if (hasMessage) {
            for (const auto& sink : sinksCopy) {
              if (sink) {
                  sink->Write(msg.level, msg.loggerName, msg.text, msg.location);
              }
            }

            std::lock_guard<std::mutex> lock(mutex_);
            workerBusy_ = false;
            if (queue_.empty()) {
                drainedCv_.notify_all();
            }
        }
    }

    std::vector<std::shared_ptr<LogSink>> sinksCopy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      workerBusy_ = false;
      sinksCopy = sinks_;
      if (queue_.empty()) {
          drainedCv_.notify_all();
      }
    }

    for (const auto& sink : sinksCopy) {
        if (sink) {
            sink->Flush();
        }
    }
}

} // namespace logging
