#include "demo/serve_async.hpp"

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

int main() {
    using namespace minphys3d::demo;
    using namespace std::chrono_literals;

    AsyncPreviewCounters counters{};
    std::mutex mutex{};
    std::condition_variable cv{};
    bool first_frame_started = false;
    bool release_first_frame = false;
    std::vector<int> consumed_frames{};

    AsyncPreviewDispatcher dispatcher(
        [&](const PreviewFrameSnapshot& frame) {
            std::unique_lock<std::mutex> lock(mutex);
            consumed_frames.push_back(frame.frame_index);
            if (frame.frame_index == 1) {
                first_frame_started = true;
                cv.notify_all();
                cv.wait(lock, [&]() { return release_first_frame; });
            }
        },
        &counters);

    auto makeFrame = [](int frame_index) {
        PreviewFrameSnapshot frame{};
        frame.frame_index = frame_index;
        frame.sim_time_s = static_cast<float>(frame_index) / 60.0f;
        return frame;
    };

    if (!dispatcher.submit(makeFrame(1))) {
        std::cerr << "failed to enqueue frame 1\n";
        return 1;
    }

    {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, 500ms, [&]() { return first_frame_started; })) {
            std::cerr << "worker did not start first frame\n";
            return 2;
        }
    }

    if (!dispatcher.submit(makeFrame(2))) {
        std::cerr << "failed to enqueue frame 2\n";
        return 3;
    }

    auto submit_latest = std::async(std::launch::async, [&]() { return dispatcher.submit(makeFrame(3)); });
    if (submit_latest.wait_for(100ms) != std::future_status::ready) {
        std::cerr << "latest-only submit blocked behind the consumer\n";
        return 4;
    }
    if (!submit_latest.get()) {
        std::cerr << "failed to enqueue frame 3\n";
        return 5;
    }

    {
        std::lock_guard<std::mutex> lock(mutex);
        release_first_frame = true;
    }
    cv.notify_all();

    dispatcher.shutdown();

    if (consumed_frames.size() != 2u || consumed_frames[0] != 1 || consumed_frames[1] != 3) {
        std::cerr << "expected frames [1, 3], got";
        for (int frame_index : consumed_frames) {
            std::cerr << " " << frame_index;
        }
        std::cerr << "\n";
        return 6;
    }

    const std::uint64_t enqueued = counters.enqueued.load(std::memory_order_relaxed);
    const std::uint64_t sent = counters.sent.load(std::memory_order_relaxed);
    const std::uint64_t dropped = counters.dropped.load(std::memory_order_relaxed);
    if (enqueued != 3 || sent != 2 || dropped != 1) {
        std::cerr << "unexpected counters enqueued=" << enqueued
                  << " sent=" << sent
                  << " dropped=" << dropped << "\n";
        return 7;
    }

    std::cout << "test_serve_preview_async ok\n";
    return 0;
}
