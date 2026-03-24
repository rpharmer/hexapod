#include "loop_executor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class ControlledTicker {
public:
    Clock::time_point now() const {
        std::scoped_lock<std::mutex> lock(mutex_);
        return now_;
    }

    void sleepUntil(const Clock::time_point& deadline, const std::atomic<bool>& running) {
        std::unique_lock<std::mutex> lock(mutex_);
        while (now_ < deadline && running.load()) {
            cv_.wait(lock);
        }
        if (now_ < deadline && !running.load()) {
            ++cancelled_waits_;
        }
    }

    void advance(std::chrono::microseconds delta) {
        {
            std::scoped_lock<std::mutex> lock(mutex_);
            now_ += delta;
        }
        cv_.notify_all();
    }

    void wakeAll() {
        cv_.notify_all();
    }

    int cancelledWaits() const {
        std::scoped_lock<std::mutex> lock(mutex_);
        return cancelled_waits_;
    }

private:
    mutable std::mutex mutex_{};
    std::condition_variable cv_{};
    Clock::time_point now_{};
    int cancelled_waits_{0};
};

bool testCadenceWithFakeTicker() {
    constexpr auto kPeriod = std::chrono::milliseconds(10);
    ControlledTicker ticker{};
    std::atomic<bool> running{true};
    std::vector<std::chrono::microseconds> starts{};
    std::mutex starts_mutex{};
    std::condition_variable starts_cv{};

    LoopExecutor executor({
        .now = [&ticker]() { return ticker.now(); },
        .sleep_until = [&ticker](const Clock::time_point& deadline, const std::atomic<bool>& running_flag) {
            ticker.sleepUntil(deadline, running_flag);
        },
    });

    executor.start({LoopExecutor::Task{
                       .period = std::chrono::duration_cast<std::chrono::microseconds>(kPeriod),
                       .step = [&]() {
                           {
                               std::scoped_lock<std::mutex> lock(starts_mutex);
                               starts.push_back(std::chrono::duration_cast<std::chrono::microseconds>(
                                   ticker.now().time_since_epoch()));
                           }
                           starts_cv.notify_all();
                       },
                   }},
                   running);

    {
        std::unique_lock<std::mutex> lock(starts_mutex);
        starts_cv.wait(lock, [&starts]() { return starts.size() == 1; });
    }
    for (std::size_t expected = 2; expected <= 4; ++expected) {
        ticker.advance(kPeriod);
        std::unique_lock<std::mutex> lock(starts_mutex);
        starts_cv.wait(lock, [&starts, expected]() { return starts.size() == expected; });
    }

    running.store(false);
    ticker.wakeAll();
    executor.stop();

    if (!expect(starts.size() == 4, "cadence test should execute exactly four iterations")) {
        return false;
    }

    return expect(starts[0] == std::chrono::microseconds{0}, "first iteration should start at t=0us") &&
           expect(starts[1] == std::chrono::microseconds{10'000}, "second iteration should start at t=10ms") &&
           expect(starts[2] == std::chrono::microseconds{20'000}, "third iteration should start at t=20ms") &&
           expect(starts[3] == std::chrono::microseconds{30'000}, "fourth iteration should start at t=30ms");
}

bool testCancellationLatencyWithoutClockAdvance() {
    ControlledTicker ticker{};
    std::atomic<bool> running{true};
    std::atomic<int> steps{0};
    std::mutex step_mutex{};
    std::condition_variable step_cv{};

    LoopExecutor executor({
        .now = [&ticker]() { return ticker.now(); },
        .sleep_until = [&ticker](const Clock::time_point& deadline, const std::atomic<bool>& running_flag) {
            ticker.sleepUntil(deadline, running_flag);
        },
    });

    executor.start({LoopExecutor::Task{
                       .period = std::chrono::seconds(1),
                       .step = [&]() {
                           ++steps;
                           step_cv.notify_all();
                       },
                   }},
                   running);

    {
        std::unique_lock<std::mutex> lock(step_mutex);
        step_cv.wait(lock, [&steps]() { return steps.load() >= 1; });
    }

    running.store(false);
    ticker.wakeAll();
    executor.stop();

    return expect(steps.load() == 1, "cancellation should stop loop before second step") &&
           expect(ticker.cancelledWaits() >= 1, "sleep should be interrupted by cancellation");
}

bool testCleanShutdownWithQueuedWork() {
    constexpr auto kPeriod = std::chrono::milliseconds(5);
    ControlledTicker ticker{};
    std::atomic<bool> running{true};
    std::mutex queue_mutex{};
    std::condition_variable queue_cv{};
    int queued = 5;
    int processed = 0;

    LoopExecutor executor({
        .now = [&ticker]() { return ticker.now(); },
        .sleep_until = [&ticker](const Clock::time_point& deadline, const std::atomic<bool>& running_flag) {
            ticker.sleepUntil(deadline, running_flag);
        },
    });

    executor.start({LoopExecutor::Task{
                       .period = std::chrono::duration_cast<std::chrono::microseconds>(kPeriod),
                       .step = [&]() {
                           {
                               std::scoped_lock<std::mutex> lock(queue_mutex);
                               if (queued > 0) {
                                   --queued;
                                   ++processed;
                               }
                           }
                           queue_cv.notify_all();
                       },
                   }},
                   running);

    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&processed]() { return processed == 1; });
    }
    for (int expected = 2; expected <= 3; ++expected) {
        ticker.advance(kPeriod);
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&processed, expected]() { return processed == expected; });
    }

    running.store(false);
    ticker.wakeAll();
    executor.stop();

    return expect(processed == 3, "shutdown should finish current cadence without draining entire queue") &&
           expect(queued == 2, "queued work should remain after clean shutdown");
}

}  // namespace

int main() {
    if (!testCadenceWithFakeTicker()) {
        return EXIT_FAILURE;
    }
    if (!testCancellationLatencyWithoutClockAdvance()) {
        return EXIT_FAILURE;
    }
    if (!testCleanShutdownWithQueuedWork()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
