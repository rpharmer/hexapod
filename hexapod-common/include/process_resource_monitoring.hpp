#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <optional>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#if defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#endif

#include <sys/resource.h>

namespace resource_monitoring {
namespace detail {

inline bool parseUnsignedToken(std::string_view text, std::uint64_t& out_value) {
    if (text.empty()) {
        return false;
    }

    const char* const begin = text.data();
    const char* const end = begin + text.size();
    std::uint64_t value = 0;
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return false;
    }

    out_value = value;
    return true;
}

inline bool parseNextToken(std::string_view& text, std::string_view& token) {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t' || text.front() == '\n' ||
                             text.front() == '\r' || text.front() == '\f' || text.front() == '\v')) {
        text.remove_prefix(1);
    }
    if (text.empty()) {
        token = {};
        return false;
    }

    const std::size_t end = text.find_first_of(" \t\n\r\f\v");
    if (end == std::string_view::npos) {
        token = text;
        text = {};
        return true;
    }

    token = text.substr(0, end);
    text.remove_prefix(end);
    return true;
}

inline std::uint64_t currentProcessCpuTimeUs() {
    struct rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0;
    }

    const std::uint64_t user_us = static_cast<std::uint64_t>(usage.ru_utime.tv_sec) * 1'000'000ULL +
                                  static_cast<std::uint64_t>(usage.ru_utime.tv_usec);
    const std::uint64_t system_us = static_cast<std::uint64_t>(usage.ru_stime.tv_sec) * 1'000'000ULL +
                                    static_cast<std::uint64_t>(usage.ru_stime.tv_usec);
    return user_us + system_us;
}

inline bool readProcessMemoryBytes(std::uint64_t& rss_bytes, std::uint64_t& vms_bytes) {
#if defined(__linux__)
    std::ifstream statm("/proc/self/statm");
    if (!statm.is_open()) {
        return false;
    }

    std::string line;
    if (!std::getline(statm, line)) {
        return false;
    }

    std::string_view view{line};
    std::string_view token{};
    std::uint64_t size_pages = 0;
    std::uint64_t resident_pages = 0;
    if (!parseNextToken(view, token) || !parseUnsignedToken(token, size_pages)) {
        return false;
    }
    if (!parseNextToken(view, token) || !parseUnsignedToken(token, resident_pages)) {
        return false;
    }

    const long page_size = ::sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        return false;
    }

    vms_bytes = size_pages * static_cast<std::uint64_t>(page_size);
    rss_bytes = resident_pages * static_cast<std::uint64_t>(page_size);
    return true;
#elif defined(__APPLE__)
    mach_task_basic_info_data_t info {};
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(),
                  MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info),
                  &count) != KERN_SUCCESS) {
        return false;
    }

    rss_bytes = static_cast<std::uint64_t>(info.resident_size);
    vms_bytes = static_cast<std::uint64_t>(info.virtual_size);
    return true;
#else
    rss_bytes = 0;
    vms_bytes = 0;
    return false;
#endif
}

} // namespace detail

template <typename T>
class CopyableAtomic {
public:
    CopyableAtomic() = default;
    explicit CopyableAtomic(T initial) : value_(initial) {}

    CopyableAtomic(const CopyableAtomic& other) noexcept : value_(other.value_.load(std::memory_order_relaxed)) {}
    CopyableAtomic& operator=(const CopyableAtomic& other) noexcept {
        if (this != &other) {
            value_.store(other.value_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        return *this;
    }

    CopyableAtomic(CopyableAtomic&& other) noexcept : value_(other.value_.load(std::memory_order_relaxed)) {}
    CopyableAtomic& operator=(CopyableAtomic&& other) noexcept {
        if (this != &other) {
            value_.store(other.value_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        return *this;
    }

    T load(std::memory_order order = std::memory_order_seq_cst) const noexcept { return value_.load(order); }
    void store(T desired, std::memory_order order = std::memory_order_seq_cst) noexcept { value_.store(desired, order); }
    T exchange(T desired, std::memory_order order = std::memory_order_seq_cst) noexcept { return value_.exchange(desired, order); }
    T fetch_add(T arg, std::memory_order order = std::memory_order_seq_cst) noexcept { return value_.fetch_add(arg, order); }
    bool compare_exchange_weak(T& expected,
                               T desired,
                               std::memory_order success,
                               std::memory_order failure) noexcept {
        return value_.compare_exchange_weak(expected, desired, success, failure);
    }

private:
    std::atomic<T> value_{};
};

struct ProcessResourceSnapshot {
    double cpu_percent{0.0};
    std::uint64_t cpu_window_us{0};
    std::uint64_t rss_bytes{0};
    std::uint64_t vms_bytes{0};
};

struct ResourceSectionSnapshot {
    const char* label{nullptr};
    std::uint64_t total_self_ns{0};
    std::uint64_t window_self_ns{0};
    std::uint64_t max_self_ns{0};
    std::uint64_t call_count{0};
};

template <std::size_t MaxSections>
struct ResourceSectionSummary {
    std::array<ResourceSectionSnapshot, MaxSections> sections{};
    std::size_t count{0};
};

template <std::size_t MaxSections, std::size_t MaxDepth = 64>
class SectionProfiler {
public:
    static_assert(MaxSections > 0, "SectionProfiler requires at least one section");
    static_assert(MaxDepth > 0, "SectionProfiler requires at least one stack slot");

    struct ScopeState {
        SectionProfiler* profiler{nullptr};
        std::size_t section_index{0};
        std::chrono::steady_clock::time_point start{};
        std::uint64_t child_elapsed_ns{0};
        bool active{false};
    };

    class Scope {
    public:
        Scope() = default;
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;

        Scope(Scope&& other) noexcept {
            state_ = other.state_;
            other.state_.active = false;
            other.state_.profiler = nullptr;
        }

        Scope& operator=(Scope&& other) noexcept {
            if (this != &other) {
                end();
                state_ = other.state_;
                other.state_.active = false;
                other.state_.profiler = nullptr;
            }
            return *this;
        }

        ~Scope() {
            end();
        }

    private:
        friend class SectionProfiler;
        explicit Scope(SectionProfiler& profiler, std::size_t section_index)
            : state_{} {
            state_.profiler = &profiler;
            state_.section_index = section_index;
            state_.start = std::chrono::steady_clock::now();
            state_.child_elapsed_ns = 0;
            state_.active = true;
            profiler.pushScope(state_);
        }

        void end() {
            if (!state_.active || state_.profiler == nullptr) {
                return;
            }
            state_.profiler->popScope(state_);
            state_.active = false;
            state_.profiler = nullptr;
        }

        ScopeState state_{};
    };

    explicit SectionProfiler(std::array<const char*, MaxSections> labels)
        : labels_(std::move(labels)) {}

    [[nodiscard]] Scope scope(std::size_t section_index) {
        return Scope(*this, section_index);
    }

    [[nodiscard]] ResourceSectionSnapshot snapshot(std::size_t section_index,
                                                   bool reset_window = true) const {
        ResourceSectionSnapshot snapshot{};
        if (section_index >= MaxSections) {
            return snapshot;
        }
        snapshot.label = labels_[section_index];
        snapshot.total_self_ns = totals_[section_index].total_self_ns.load(std::memory_order_relaxed);
        snapshot.max_self_ns = max_self_ns_[section_index].load(std::memory_order_relaxed);
        snapshot.call_count = call_counts_[section_index].load(std::memory_order_relaxed);
        if (reset_window) {
            snapshot.window_self_ns = window_self_ns_[section_index].exchange(0, std::memory_order_relaxed);
        } else {
            snapshot.window_self_ns = window_self_ns_[section_index].load(std::memory_order_relaxed);
        }
        return snapshot;
    }

    [[nodiscard]] ResourceSectionSummary<MaxSections> snapshotAll(bool reset_window = true) const {
        ResourceSectionSummary<MaxSections> summary{};
        summary.count = MaxSections;
        for (std::size_t index = 0; index < MaxSections; ++index) {
            summary.sections[index] = snapshot(index, reset_window);
        }
        return summary;
    }

    [[nodiscard]] ResourceSectionSummary<MaxSections> topSections(std::size_t limit,
                                                                  bool reset_window = true) const {
        ResourceSectionSummary<MaxSections> summary = snapshotAll(reset_window);
        if (limit >= summary.count) {
            sortSections(summary.sections.begin(), summary.sections.begin() + summary.count);
            return summary;
        }

        sortSections(summary.sections.begin(), summary.sections.begin() + summary.count);
        summary.count = limit;
        return summary;
    }

    [[nodiscard]] const char* label(std::size_t section_index) const {
        if (section_index >= MaxSections) {
            return nullptr;
        }
        return labels_[section_index];
    }

    void reset() {
        for (std::size_t index = 0; index < MaxSections; ++index) {
            totals_[index].total_self_ns.store(0, std::memory_order_relaxed);
            window_self_ns_[index].store(0, std::memory_order_relaxed);
            max_self_ns_[index].store(0, std::memory_order_relaxed);
            call_counts_[index].store(0, std::memory_order_relaxed);
        }
    }

private:
    struct SectionCounters {
        CopyableAtomic<std::uint64_t> total_self_ns{0};
    };

    static std::array<ScopeState, MaxDepth>& activeStack() {
        static thread_local std::array<ScopeState, MaxDepth> stack{};
        return stack;
    }

    static std::size_t& activeDepth() {
        static thread_local std::size_t depth = 0;
        return depth;
    }

    static std::uint64_t elapsedNs(const std::chrono::steady_clock::duration& duration) {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        return ns > 0 ? static_cast<std::uint64_t>(ns) : 0ULL;
    }

    void pushScope(const ScopeState& state) {
        auto& depth = activeDepth();
        auto& stack = activeStack();
        if (depth < MaxDepth) {
            stack[depth] = state;
            ++depth;
            return;
        }
        // Stack overflow is intentionally lossy but non-fatal: keep the scope active so we still
        // record self-time, but skip nesting propagation for this frame.
    }

    void popScope(ScopeState& state) {
        const auto now = std::chrono::steady_clock::now();
        const std::uint64_t elapsed_ns = elapsedNs(now - state.start);
        const std::uint64_t self_ns = elapsed_ns > state.child_elapsed_ns
                                          ? (elapsed_ns - state.child_elapsed_ns)
                                          : 0ULL;
        recordSection(state.section_index, self_ns);

        auto& depth = activeDepth();
        auto& stack = activeStack();
        if (depth > 0) {
            const std::size_t top_index = depth - 1;
            if (top_index < MaxDepth &&
                stack[top_index].profiler == state.profiler &&
                stack[top_index].section_index == state.section_index &&
                stack[top_index].start == state.start) {
                --depth;
            }
        }
        if (depth > 0) {
            stack[depth - 1].child_elapsed_ns += elapsed_ns;
        }
    }

    void recordSection(std::size_t section_index, std::uint64_t self_ns) {
        if (section_index >= MaxSections) {
            return;
        }

        totals_[section_index].total_self_ns.fetch_add(self_ns, std::memory_order_relaxed);
        window_self_ns_[section_index].fetch_add(self_ns, std::memory_order_relaxed);
        call_counts_[section_index].fetch_add(1, std::memory_order_relaxed);

        std::uint64_t current_max = max_self_ns_[section_index].load(std::memory_order_relaxed);
        while (self_ns > current_max &&
               !max_self_ns_[section_index].compare_exchange_weak(
                   current_max,
                   self_ns,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed)) {
        }
    }

    static void sortSections(typename std::array<ResourceSectionSnapshot, MaxSections>::iterator begin,
                             typename std::array<ResourceSectionSnapshot, MaxSections>::iterator end) {
        std::sort(begin, end, [](const ResourceSectionSnapshot& lhs, const ResourceSectionSnapshot& rhs) {
            if (lhs.window_self_ns != rhs.window_self_ns) {
                return lhs.window_self_ns > rhs.window_self_ns;
            }
            if (lhs.total_self_ns != rhs.total_self_ns) {
                return lhs.total_self_ns > rhs.total_self_ns;
            }
            const bool lhs_has_label = lhs.label != nullptr;
            const bool rhs_has_label = rhs.label != nullptr;
            if (lhs_has_label != rhs_has_label) {
                return lhs_has_label;
            }
            if (!lhs_has_label) {
                return false;
            }
            return std::string_view{lhs.label} < std::string_view{rhs.label};
        });
    }

    std::array<const char*, MaxSections> labels_{};
    std::array<SectionCounters, MaxSections> totals_{};
    mutable std::array<CopyableAtomic<std::uint64_t>, MaxSections> window_self_ns_{};
    std::array<CopyableAtomic<std::uint64_t>, MaxSections> max_self_ns_{};
    std::array<CopyableAtomic<std::uint64_t>, MaxSections> call_counts_{};
};

class ProcessResourceSampler {
public:
    ProcessResourceSnapshot sample() {
        ProcessResourceSnapshot snapshot{};
        const std::uint64_t current_cpu_time_us = detail::currentProcessCpuTimeUs();
        (void)detail::readProcessMemoryBytes(snapshot.rss_bytes, snapshot.vms_bytes);

        const auto now = std::chrono::steady_clock::now();
        if (has_last_sample_) {
            const auto wall_window = std::chrono::duration_cast<std::chrono::microseconds>(now - last_sample_time_);
            if (wall_window.count() > 0 && current_cpu_time_us >= last_cpu_time_us_) {
                snapshot.cpu_window_us = static_cast<std::uint64_t>(wall_window.count());
                const std::uint64_t cpu_delta_us = current_cpu_time_us - last_cpu_time_us_;
                snapshot.cpu_percent = 100.0 * static_cast<double>(cpu_delta_us) /
                                       static_cast<double>(snapshot.cpu_window_us);
            }
        }

        last_cpu_time_us_ = current_cpu_time_us;
        last_sample_time_ = now;
        has_last_sample_ = true;
        return snapshot;
    }

private:
    bool has_last_sample_{false};
    std::uint64_t last_cpu_time_us_{0};
    std::chrono::steady_clock::time_point last_sample_time_{};
};

inline bool parseProcStatmLine(std::string_view line,
                               std::uint64_t& size_pages,
                               std::uint64_t& resident_pages) {
    std::string_view view{line};
    std::string_view token{};
    if (!detail::parseNextToken(view, token) || !detail::parseUnsignedToken(token, size_pages)) {
        return false;
    }
    if (!detail::parseNextToken(view, token) || !detail::parseUnsignedToken(token, resident_pages)) {
        return false;
    }
    return true;
}

inline std::string formatProcessResourceSnapshot(const ProcessResourceSnapshot& snapshot) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1);
    out << "cpu_percent=" << snapshot.cpu_percent
        << " cpu_window_ms=" << (snapshot.cpu_window_us / 1000ULL)
        << " rss_bytes=" << snapshot.rss_bytes
        << " vms_bytes=" << snapshot.vms_bytes;
    return out.str();
}

template <std::size_t MaxSections>
inline std::string formatResourceSectionSummary(const ResourceSectionSummary<MaxSections>& summary,
                                                std::size_t max_sections_to_emit = MaxSections) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    const std::size_t count = std::min(summary.count, max_sections_to_emit);
    for (std::size_t index = 0; index < count; ++index) {
        if (index > 0) {
            out << " | ";
        }
        const ResourceSectionSnapshot& section = summary.sections[index];
        out << (section.label != nullptr ? section.label : "<unnamed>")
            << "{window_ms=" << (section.window_self_ns / 1'000'000ULL)
            << ",total_ms=" << (section.total_self_ns / 1'000'000ULL)
            << ",max_ms=" << (section.max_self_ns / 1'000'000ULL)
            << ",calls=" << section.call_count
            << '}';
    }
    return out.str();
}

} // namespace resource_monitoring
