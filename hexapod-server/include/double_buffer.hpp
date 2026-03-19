#pragma once

#include <atomic>
#include <mutex>

template <typename T>
class DoubleBuffer {
public:
    void write(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        const int next = 1 - index_.load(std::memory_order_relaxed);
        buffers_[next] = value;
        index_.store(next, std::memory_order_release);
    }

    T read() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const int idx = index_.load(std::memory_order_acquire);
        return buffers_[idx];
    }

private:
    T buffers_[2]{};
    std::atomic<int> index_{0};
    mutable std::mutex mutex_;
};
