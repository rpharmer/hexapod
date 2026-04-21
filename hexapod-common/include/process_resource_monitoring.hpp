#pragma once

#include <chrono>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

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

struct ProcessResourceSnapshot {
    double cpu_percent{0.0};
    std::uint64_t cpu_window_us{0};
    std::uint64_t rss_bytes{0};
    std::uint64_t vms_bytes{0};
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

} // namespace resource_monitoring
