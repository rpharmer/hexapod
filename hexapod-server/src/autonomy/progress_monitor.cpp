#include "autonomy/progress_monitor.hpp"

namespace autonomy {

ProgressMonitor::ProgressMonitor(uint64_t no_progress_timeout_ms)
    : no_progress_timeout_ms_(no_progress_timeout_ms) {}

void ProgressMonitor::reset() {
    initialized_ = false;
    last_progress_sample_ = {};
}

ProgressEvaluation ProgressMonitor::evaluate(const ProgressSample& sample) {
    if (!initialized_) {
        initialized_ = true;
        last_progress_sample_ = sample;
        return ProgressEvaluation{};
    }

    if (sample.completed_waypoints > last_progress_sample_.completed_waypoints) {
        last_progress_sample_ = sample;
        return ProgressEvaluation{};
    }

    const DurationMs stagnant_duration_ms =
        (sample.timestamp_ms >= last_progress_sample_.timestamp_ms)
            ? (sample.timestamp_ms - last_progress_sample_.timestamp_ms)
            : DurationMs{};

    return ProgressEvaluation{
        .no_progress = stagnant_duration_ms.value > no_progress_timeout_ms_,
        .stagnant_duration_ms = stagnant_duration_ms.value,
    };
}

} // namespace autonomy
