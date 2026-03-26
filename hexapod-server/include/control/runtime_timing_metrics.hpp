#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>

struct LoopTimingRollingMetrics {
    uint64_t sample_count{0};
    uint64_t p50_control_dt_us{0};
    uint64_t p95_control_dt_us{0};
    uint64_t p99_control_dt_us{0};
    uint64_t overrun_events_total{0};
    uint64_t overrun_periods_total{0};
    uint64_t consecutive_overruns{0};
    uint64_t max_consecutive_overruns{0};
    uint64_t hard_overrun_escalation_crossings{0};
    uint64_t warning_consecutive_overrun_threshold{0};
    uint64_t hard_consecutive_overrun_threshold{0};
};

class RuntimeTimingMetrics {
public:
    static constexpr std::size_t kRollingWindowSize = 128;
    static constexpr uint64_t kWarningConsecutiveOverruns = 3;
    static constexpr uint64_t kHardConsecutiveOverruns = 5;

    RuntimeTimingMetrics(const control_config::ControlConfig& config,
                         std::atomic<uint64_t>& control_dt_sum_us,
                         std::atomic<uint64_t>& control_jitter_max_us);

    void reset();
    void update(TimePointUs now);
    uint64_t averageControlDtUs(uint64_t loops) const;
    [[nodiscard]] LoopTimingRollingMetrics rollingMetrics() const;

private:
    [[nodiscard]] uint64_t percentileLocked(double quantile) const;

    const control_config::ControlConfig& config_;
    std::atomic<uint64_t>& control_dt_sum_us_;
    std::atomic<uint64_t>& control_jitter_max_us_;
    mutable std::mutex mutex_{};
    TimePointUs last_control_step_us_{};
    std::array<uint64_t, kRollingWindowSize> control_dt_window_us_{};
    std::size_t control_dt_window_count_{0};
    std::size_t control_dt_window_write_index_{0};
    uint64_t overrun_events_total_{0};
    uint64_t overrun_periods_total_{0};
    uint64_t consecutive_overruns_{0};
    uint64_t max_consecutive_overruns_{0};
    uint64_t hard_overrun_escalation_crossings_{0};
};
