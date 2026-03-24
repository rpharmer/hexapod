#include "loop_executor.hpp"

#include <utility>

LoopExecutor::TimingDependencies LoopExecutor::TimingDependencies::realtime() {
    return TimingDependencies{
        .now = []() { return Clock::now(); },
        .sleep_until = [](const Clock::time_point& deadline, const std::atomic<bool>&) {
            loop_timing::sleepUntil(deadline);
        },
    };
}

LoopExecutor::LoopExecutor()
    : LoopExecutor(TimingDependencies::realtime()) {}

LoopExecutor::LoopExecutor(TimingDependencies timing)
    : timing_(std::move(timing)) {}

LoopExecutor::~LoopExecutor() {
    stop();
}

void LoopExecutor::start(const std::vector<Task>& tasks, std::atomic<bool>& running_flag) {
    stop();
    threads_.reserve(tasks.size());

    for (const Task& task : tasks) {
        threads_.emplace_back(&LoopExecutor::runTask, this, task, std::ref(running_flag));
    }
}

void LoopExecutor::stop() {
    for (auto& thread : threads_) {
        joinThread(thread);
    }
    threads_.clear();
}

void LoopExecutor::runTask(Task task, std::atomic<bool>& running_flag) const {
    while (running_flag.load()) {
        const Clock::time_point cycle_start = timing_.now();
        task.step();
        timing_.sleep_until(cycle_start + task.period, running_flag);
    }
}

void LoopExecutor::joinThread(std::thread& thread) {
    if (thread.joinable()) {
        thread.join();
    }
}
