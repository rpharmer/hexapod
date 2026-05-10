#pragma once

#include "demo/frame_sink.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace minphys3d::demo {

struct PreviewBodySnapshot {
    std::uint32_t id{0};
    Body body{};
};

struct PreviewFrameSnapshot {
    int frame_index{0};
    Real sim_time_s{0.0};
    TerrainPatchFrameSnapshot terrain_patch{};
    std::vector<PreviewBodySnapshot> bodies{};
};

struct AsyncPreviewCounters {
    std::atomic<std::uint64_t> enqueued{0};
    std::atomic<std::uint64_t> sent{0};
    std::atomic<std::uint64_t> dropped{0};
};

class AsyncPreviewDispatcher {
public:
    using Consumer = std::function<void(const PreviewFrameSnapshot&)>;

    explicit AsyncPreviewDispatcher(Consumer consumer, AsyncPreviewCounters* counters = nullptr)
        : consumer_(std::move(consumer)), counters_(counters) {
        worker_ = std::thread([this]() { run(); });
    }

    AsyncPreviewDispatcher(const AsyncPreviewDispatcher&) = delete;
    AsyncPreviewDispatcher& operator=(const AsyncPreviewDispatcher&) = delete;

    AsyncPreviewDispatcher(AsyncPreviewDispatcher&&) = delete;
    AsyncPreviewDispatcher& operator=(AsyncPreviewDispatcher&&) = delete;

    ~AsyncPreviewDispatcher() {
        shutdown();
    }

    bool submit(PreviewFrameSnapshot frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_requested_) {
            return false;
        }
        if (pending_frame_.has_value()) {
            incrementDropped();
        }
        pending_frame_ = std::move(frame);
        incrementEnqueued();
        cv_.notify_one();
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_requested_) {
                if (worker_.joinable()) {
                    worker_.join();
                }
                return;
            }
            shutdown_requested_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    void incrementEnqueued() const {
        if (counters_ != nullptr) {
            counters_->enqueued.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void incrementSent() const {
        if (counters_ != nullptr) {
            counters_->sent.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void incrementDropped() const {
        if (counters_ != nullptr) {
            counters_->dropped.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void run() {
        for (;;) {
            PreviewFrameSnapshot next{};
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&]() { return shutdown_requested_ || pending_frame_.has_value(); });
                if (!pending_frame_.has_value()) {
                    if (shutdown_requested_) {
                        break;
                    }
                    continue;
                }
                next = std::move(*pending_frame_);
                pending_frame_.reset();
            }

            if (consumer_) {
                consumer_(next);
            }
            incrementSent();
        }
    }

    Consumer consumer_{};
    AsyncPreviewCounters* counters_{nullptr};
    std::mutex mutex_{};
    std::condition_variable cv_{};
    std::optional<PreviewFrameSnapshot> pending_frame_{};
    bool shutdown_requested_{false};
    std::thread worker_{};
};

inline std::unique_ptr<AsyncPreviewDispatcher> MakeAsyncPreviewDispatcher(
    SinkKind sink_kind,
    const std::string& udp_host,
    int udp_port,
    AsyncPreviewCounters* counters = nullptr) {
    if (sink_kind != SinkKind::Udp) {
        return nullptr;
    }

    std::shared_ptr<FrameSink> sink = std::shared_ptr<FrameSink>(MakeFrameSink(sink_kind, udp_host, udp_port).release());
    return std::make_unique<AsyncPreviewDispatcher>(
        [sink = std::move(sink)](const PreviewFrameSnapshot& frame) {
            sink->begin_frame(frame.frame_index, frame.sim_time_s);
            sink->emit_terrain_patch_snapshot(frame.terrain_patch);
            for (const PreviewBodySnapshot& body : frame.bodies) {
                sink->emit_body(body.id, body.body);
            }
            sink->end_frame();
        },
        counters);
}

} // namespace minphys3d::demo
