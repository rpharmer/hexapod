#pragma once
#include <queue>
#include <mutex>
#include <optional>

template<typename T>
class EventQueue {
public:
    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mtx);
        q.push(item);
    }

    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (q.empty()) return std::nullopt;
        T item = q.front();
        q.pop();
        return item;
    }

private:
    std::queue<T> q;
    std::mutex mtx;
};
