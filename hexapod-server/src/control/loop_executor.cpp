#include "loop_executor.hpp"

#include <utility>

LoopExecutor::CancellationSource::CancellationSource()
    : state_(std::make_shared<CancellationState>()) {}

LoopExecutor::CancellationToken LoopExecutor::CancellationSource::token() const {
    return CancellationToken(state_);
}

void LoopExecutor::CancellationSource::requestStop() const {
    state_->stop_requested.store(true);
    {
        std::scoped_lock<std::mutex> lock(state_->mutex);
    }
    state_->cv.notify_all();
}

LoopExecutor::CancellationToken::CancellationToken(std::shared_ptr<CancellationState> state)
    : state_(std::move(state)) {}

bool LoopExecutor::CancellationToken::stopRequested() const {
    return !state_ || state_->stop_requested.load();
}

bool LoopExecutor::CancellationToken::waitUntil(const Clock::time_point& deadline) const {
    if (!state_) {
        return false;
    }
    std::unique_lock<std::mutex> lock(state_->mutex);
    return state_->cv.wait_until(lock, deadline, [this]() { return state_->stop_requested.load(); });
}

void LoopExecutor::CancellationToken::notifyWaiters() const {
    if (state_) {
        state_->cv.notify_all();
    }
}

LoopExecutor::TimingDependencies LoopExecutor::TimingDependencies::realtime() {
    return TimingDependencies{
        .now = []() { return Clock::now(); },
        .sleep_until = [](const Clock::time_point& deadline, const CancellationToken& stop_token) {
            (void)stop_token.waitUntil(deadline);
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
    stop_source_ = CancellationSource{};
    threads_.reserve(tasks.size());

    for (const Task& task : tasks) {
        threads_.emplace_back(&LoopExecutor::runTask, this, task, std::ref(running_flag), stop_source_.token());
    }
}

void LoopExecutor::stop() {
    stop_source_.requestStop();
    for (auto& thread : threads_) {
        joinThread(thread);
    }
    threads_.clear();
}

void LoopExecutor::runTask(Task task, std::atomic<bool>& running_flag, CancellationToken stop_token) const {
    while (running_flag.load() && !stop_token.stopRequested()) {
        const Clock::time_point cycle_start = timing_.now();
        const Clock::time_point deadline = cycle_start + task.period;
        task.step();
        const Clock::time_point cycle_finish = timing_.now();

        std::uint64_t overruns = 0;
        Clock::time_point next_deadline = deadline;
        if (cycle_finish >= deadline + task.period) {
            const auto behind = std::chrono::duration_cast<std::chrono::microseconds>(cycle_finish - deadline);
            overruns = static_cast<std::uint64_t>(behind / task.period);
            next_deadline = deadline + task.period * static_cast<std::chrono::microseconds::rep>(overruns + 1U);
        }

        if (task.on_iteration) {
            task.on_iteration(IterationTelemetry{
                .label = task.label,
                .cycle_start = cycle_start,
                .cycle_finish = cycle_finish,
                .deadline = deadline,
                .overruns = overruns,
            });
        }

        if (!running_flag.load() || stop_token.stopRequested()) {
            break;
        }
        if (cycle_finish < deadline) {
            timing_.sleep_until(deadline, stop_token);
        } else if (cycle_finish >= deadline + task.period && cycle_finish < next_deadline) {
            timing_.sleep_until(next_deadline, stop_token);
        }
    }
}

void LoopExecutor::joinThread(std::thread& thread) {
    if (thread.joinable()) {
        thread.join();
    }
}
