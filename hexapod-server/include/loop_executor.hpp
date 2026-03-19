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

    LoopExecutor() = default;
    ~LoopExecutor();

    LoopExecutor(const LoopExecutor&) = delete;
    LoopExecutor& operator=(const LoopExecutor&) = delete;

    void start(const std::vector<Task>& tasks, std::atomic<bool>& running_flag);
    void stop();

private:
    static void runTask(Task task, std::atomic<bool>& running_flag);
    static void joinThread(std::thread& thread);

    std::vector<std::thread> threads_;
};
