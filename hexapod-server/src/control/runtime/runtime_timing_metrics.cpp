#include "runtime_timing_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

RuntimeTimingMetrics::RuntimeTimingMetrics(const control_config::ControlConfig& config,
                                           std::atomic<uint64_t>& control_dt_sum_us,
                                           std::atomic<uint64_t>& control_jitter_max_us)
    : config_(config),
      control_dt_sum_us_(control_dt_sum_us),
      control_jitter_max_us_(control_jitter_max_us) {}

void RuntimeTimingMetrics::reset() {
    std::scoped_lock<std::mutex> lock(mutex_);
    last_control_step_us_ = TimePointUs{};
    control_dt_window_us_.fill(0);
    control_dt_window_count_ = 0;
    control_dt_window_write_index_ = 0;
    overrun_events_total_ = 0;
    overrun_periods_total_ = 0;
    consecutive_overruns_ = 0;
    max_consecutive_overruns_ = 0;
    hard_overrun_escalation_crossings_ = 0;
}

void RuntimeTimingMetrics::update(TimePointUs now) {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (last_control_step_us_.isZero()) {
        last_control_step_us_ = now;
        return;
    }

    const uint64_t dt_us = static_cast<uint64_t>((now - last_control_step_us_).value);
    control_dt_sum_us_.fetch_add(dt_us);
    const uint64_t target_us = static_cast<uint64_t>(config_.loop_timing.control_loop_period.count());
    const uint64_t jitter_us = (dt_us > target_us) ? (dt_us - target_us) : (target_us - dt_us);
    uint64_t current_max = control_jitter_max_us_.load();
    while (jitter_us > current_max &&
           !control_jitter_max_us_.compare_exchange_weak(current_max, jitter_us)) {
    }

    control_dt_window_us_[control_dt_window_write_index_] = dt_us;
    control_dt_window_write_index_ = (control_dt_window_write_index_ + 1) % kRollingWindowSize;
    if (control_dt_window_count_ < kRollingWindowSize) {
        ++control_dt_window_count_;
    }

    if (target_us > 0 && dt_us > target_us) {
        ++overrun_events_total_;
        overrun_periods_total_ += (dt_us / target_us);
        ++consecutive_overruns_;
        max_consecutive_overruns_ = std::max(max_consecutive_overruns_, consecutive_overruns_);
        if (consecutive_overruns_ == kHardConsecutiveOverruns) {
            ++hard_overrun_escalation_crossings_;
        }
    } else {
        consecutive_overruns_ = 0;
    }

    last_control_step_us_ = now;
}

uint64_t RuntimeTimingMetrics::averageControlDtUs(uint64_t loops) const {
    const uint64_t dt_sum_us = control_dt_sum_us_.load();
    return (loops > 1) ? (dt_sum_us / (loops - 1)) : 0;
}

uint64_t RuntimeTimingMetrics::percentileLocked(double quantile) const {
    if (control_dt_window_count_ == 0) {
        return 0;
    }

    std::vector<uint64_t> ordered(control_dt_window_count_);
    for (std::size_t idx = 0; idx < control_dt_window_count_; ++idx) {
        const std::size_t ring_idx =
            (control_dt_window_write_index_ + kRollingWindowSize - control_dt_window_count_ + idx) %
            kRollingWindowSize;
        ordered[idx] = control_dt_window_us_[ring_idx];
    }
    std::sort(ordered.begin(), ordered.end());

    const double scaled = std::ceil(quantile * static_cast<double>(ordered.size()));
    const std::size_t rank = std::clamp<std::size_t>(
        scaled > 1.0 ? static_cast<std::size_t>(scaled) : 1U,
        1U,
        ordered.size());
    return ordered[rank - 1U];
}

LoopTimingRollingMetrics RuntimeTimingMetrics::rollingMetrics() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    return LoopTimingRollingMetrics{
        .sample_count = static_cast<uint64_t>(control_dt_window_count_),
        .p50_control_dt_us = percentileLocked(0.50),
        .p95_control_dt_us = percentileLocked(0.95),
        .p99_control_dt_us = percentileLocked(0.99),
        .overrun_events_total = overrun_events_total_,
        .overrun_periods_total = overrun_periods_total_,
        .consecutive_overruns = consecutive_overruns_,
        .max_consecutive_overruns = max_consecutive_overruns_,
        .hard_overrun_escalation_crossings = hard_overrun_escalation_crossings_,
        .warning_consecutive_overrun_threshold = kWarningConsecutiveOverruns,
        .hard_consecutive_overrun_threshold = kHardConsecutiveOverruns,
    };
}
