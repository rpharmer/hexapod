#include "loop_executor.hpp"

LoopExecutor::~LoopExecutor() {
    stop();
}

void LoopExecutor::start(const std::vector<Task>& tasks, std::atomic<bool>& running_flag) {
    stop();
    threads_.reserve(tasks.size());

    for (const Task& task : tasks) {
        threads_.emplace_back(&LoopExecutor::runTask, task, std::ref(running_flag));
    }
}

void LoopExecutor::stop() {
    for (auto& thread : threads_) {
        joinThread(thread);
    }
    threads_.clear();
}

void LoopExecutor::runTask(Task task, std::atomic<bool>& running_flag) {
    while (running_flag.load()) {
        const auto cycle_start = Clock::now();
        task.step();
        loop_timing::sleepUntil(cycle_start, task.period);
    }
}

void LoopExecutor::joinThread(std::thread& thread) {
    if (thread.joinable()) {
        thread.join();
    }
}
