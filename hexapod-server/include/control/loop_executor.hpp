#pragma once

#include "loop_timing.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class LoopExecutor {
public:
    struct CancellationState {
        std::atomic<bool> stop_requested{false};
        mutable std::mutex mutex{};
        std::condition_variable cv{};
    };

    class CancellationToken;

    class CancellationSource {
    public:
        CancellationSource();

        [[nodiscard]] CancellationToken token() const;
        void requestStop() const;

    private:
        std::shared_ptr<CancellationState> state_{};
    };

    class CancellationToken {
    public:
        [[nodiscard]] bool stopRequested() const;
        [[nodiscard]] bool waitUntil(const Clock::time_point& deadline) const;
        void notifyWaiters() const;

    private:
        friend class CancellationSource;
        explicit CancellationToken(std::shared_ptr<CancellationState> state);

        std::shared_ptr<CancellationState> state_{};
    };

    struct IterationTelemetry {
        Clock::time_point cycle_start{};
        Clock::time_point cycle_finish{};
        Clock::time_point deadline{};
        std::uint64_t overruns{0};
    };

    struct Task {
        std::chrono::microseconds period;
        std::function<void()> step;
        std::function<void(const IterationTelemetry&)> on_iteration{};
    };

    struct TimingDependencies {
        std::function<Clock::time_point()> now{};
        std::function<void(const Clock::time_point&, const CancellationToken&)> sleep_until{};

        static TimingDependencies realtime();
    };

    LoopExecutor();
    explicit LoopExecutor(TimingDependencies timing);
    ~LoopExecutor();

    LoopExecutor(const LoopExecutor&) = delete;
    LoopExecutor& operator=(const LoopExecutor&) = delete;

    void start(const std::vector<Task>& tasks, std::atomic<bool>& running_flag);
    void stop();

private:
    void runTask(Task task, std::atomic<bool>& running_flag, CancellationToken stop_token) const;
    static void joinThread(std::thread& thread);

    std::vector<std::thread> threads_;
    TimingDependencies timing_{};
    CancellationSource stop_source_{};
};
