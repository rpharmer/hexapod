#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <atomic>
#include <cstdint>

class RuntimeTimingMetrics {
public:
    RuntimeTimingMetrics(const control_config::ControlConfig& config,
                         std::atomic<uint64_t>& control_dt_sum_us,
                         std::atomic<uint64_t>& control_jitter_max_us);

    void reset();
    void update(TimePointUs now);
    uint64_t averageControlDtUs(uint64_t loops) const;

private:
    const control_config::ControlConfig& config_;
    std::atomic<uint64_t>& control_dt_sum_us_;
    std::atomic<uint64_t>& control_jitter_max_us_;
    TimePointUs last_control_step_us_{};
};
