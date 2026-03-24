#pragma once

#include "loop_timing.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>

class LoopExecutor {
public:
    struct Task {
        std::chrono::microseconds period;
        std::function<void()> step;
    };

    struct TimingDependencies {
        std::function<Clock::time_point()> now{};
        std::function<void(const Clock::time_point&, const std::atomic<bool>&)> sleep_until{};

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
    void runTask(Task task, std::atomic<bool>& running_flag) const;
    static void joinThread(std::thread& thread);

    std::vector<std::thread> threads_;
    TimingDependencies timing_{};
};
