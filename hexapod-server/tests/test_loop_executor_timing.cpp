#include "loop_executor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <string_view>
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

    void sleepUntil(const Clock::time_point& deadline, const LoopExecutor::CancellationToken& stop_token) {
        std::unique_lock<std::mutex> lock(mutex_);
        while (now_ < deadline && !stop_token.stopRequested()) {
            cv_.wait_for(lock, std::chrono::milliseconds(1));
        }
        if (now_ < deadline && stop_token.stopRequested()) {
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
        .sleep_until = [&ticker](const Clock::time_point& deadline, const LoopExecutor::CancellationToken& stop_token) {
            ticker.sleepUntil(deadline, stop_token);
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
        .sleep_until = [&ticker](const Clock::time_point& deadline, const LoopExecutor::CancellationToken& stop_token) {
            ticker.sleepUntil(deadline, stop_token);
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
        .sleep_until = [&ticker](const Clock::time_point& deadline, const LoopExecutor::CancellationToken& stop_token) {
            ticker.sleepUntil(deadline, stop_token);
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

bool testOverrunAccountingAndDeterministicScheduling() {
    constexpr auto kPeriod = std::chrono::milliseconds(10);
    ControlledTicker ticker{};
    std::atomic<bool> running{true};
    std::atomic<int> iteration_index{0};
    std::mutex telemetry_mutex{};
    std::condition_variable telemetry_cv{};
    std::vector<LoopExecutor::IterationTelemetry> telemetry{};

    LoopExecutor executor({
        .now = [&ticker]() { return ticker.now(); },
        .sleep_until = [&ticker](const Clock::time_point& deadline, const LoopExecutor::CancellationToken& stop_token) {
            ticker.sleepUntil(deadline, stop_token);
        },
    });

    executor.start({LoopExecutor::Task{
                       .label = "test-overrun",
                       .period = std::chrono::duration_cast<std::chrono::microseconds>(kPeriod),
                       .step = [&ticker, &iteration_index]() {
                           const int iteration = iteration_index.fetch_add(1);
                           if (iteration == 0) {
                               ticker.advance(std::chrono::milliseconds(27));
                               return;
                           }
                           ticker.advance(std::chrono::milliseconds(12));
                       },
                       .on_iteration =
                           [&](const LoopExecutor::IterationTelemetry& sample) {
                               {
                                   std::scoped_lock<std::mutex> lock(telemetry_mutex);
                                   telemetry.push_back(sample);
                               }
                               telemetry_cv.notify_all();
                           },
                   }},
                   running);

    {
        std::unique_lock<std::mutex> lock(telemetry_mutex);
        telemetry_cv.wait(lock, [&telemetry]() { return !telemetry.empty(); });
    }
    ticker.advance(std::chrono::milliseconds(3));
    {
        std::unique_lock<std::mutex> lock(telemetry_mutex);
        telemetry_cv.wait(lock, [&telemetry]() { return telemetry.size() >= 3; });
    }

    running.store(false);
    ticker.wakeAll();
    executor.stop();

    const auto first_start = std::chrono::duration_cast<std::chrono::microseconds>(
        telemetry[0].cycle_start.time_since_epoch());
    const auto second_start = std::chrono::duration_cast<std::chrono::microseconds>(
        telemetry[1].cycle_start.time_since_epoch());
    const auto delta = second_start - first_start;

    return expect(telemetry[0].overruns == 1, "27ms work on 10ms period should count one missed period") &&
           expect(telemetry[0].label != nullptr && std::string_view{telemetry[0].label} == "test-overrun",
                  "iteration telemetry should include the task label") &&
           expect(delta == std::chrono::microseconds{30'000},
                  "overrun scheduling should resume at the next 30ms-aligned boundary") &&
           expect(telemetry[2].cycle_start == telemetry[1].cycle_finish,
                  "near-deadline path should restart immediately without extra sleep");
}

bool testStopLatencyBoundWithSlowTask() {
    using namespace std::chrono_literals;
    std::atomic<bool> running{true};
    std::atomic<bool> entered_step{false};

    LoopExecutor executor{};
    executor.start({LoopExecutor::Task{
                       .period = std::chrono::microseconds{200'000},
                       .step = [&]() {
                           entered_step.store(true);
                           std::this_thread::sleep_for(40ms);
                       },
                   }},
                   running);

    while (!entered_step.load()) {
        std::this_thread::yield();
    }

    const auto stop_start = std::chrono::steady_clock::now();
    running.store(false);
    executor.stop();
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_start;

    return expect(stop_elapsed < 90ms, "stop latency should remain bounded by in-flight step execution");
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
    if (!testOverrunAccountingAndDeterministicScheduling()) {
        return EXIT_FAILURE;
    }
    if (!testStopLatencyBoundWithSlowTask()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
