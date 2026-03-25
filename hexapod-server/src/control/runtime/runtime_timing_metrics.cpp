#include "runtime_timing_metrics.hpp"

RuntimeTimingMetrics::RuntimeTimingMetrics(const control_config::ControlConfig& config,
                                           std::atomic<uint64_t>& control_dt_sum_us,
                                           std::atomic<uint64_t>& control_jitter_max_us)
    : config_(config),
      control_dt_sum_us_(control_dt_sum_us),
      control_jitter_max_us_(control_jitter_max_us) {}

void RuntimeTimingMetrics::reset() {
    last_control_step_us_ = TimePointUs{};
}

void RuntimeTimingMetrics::update(TimePointUs now) {
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
    last_control_step_us_ = now;
}

uint64_t RuntimeTimingMetrics::averageControlDtUs(uint64_t loops) const {
    const uint64_t dt_sum_us = control_dt_sum_us_.load();
    return (loops > 1) ? (dt_sum_us / (loops - 1)) : 0;
}
