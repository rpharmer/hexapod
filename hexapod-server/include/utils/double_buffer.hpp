#pragma once

#include <cstddef>
#include <mutex>
#include <utility>

template <typename T>
class DoubleBuffer {
public:
    void write(const T& value) {
        write_impl(value);
    }

    void write(T&& value) {
        write_impl(std::move(value));
    }

    [[nodiscard]] T read() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffers_[read_index_];
    }

private:
    template <typename U>
    void write_impl(U&& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::size_t next_index = read_index_ ^ 1U;
        buffers_[next_index] = std::forward<U>(value);
        read_index_ = next_index;
    }

    T buffers_[2]{};
    std::size_t read_index_{0};
    mutable std::mutex mutex_;
};
