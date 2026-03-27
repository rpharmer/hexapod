#pragma once

#include <cstdint>

namespace autonomy {

struct ProgressSample {
    uint64_t timestamp_ms{0};
    uint64_t completed_waypoints{0};
};

struct ProgressEvaluation {
    bool no_progress{false};
    uint64_t stagnant_duration_ms{0};
};

class ProgressMonitor {
public:
    explicit ProgressMonitor(uint64_t no_progress_timeout_ms);

    void reset();
    ProgressEvaluation evaluate(const ProgressSample& sample);

private:
    uint64_t no_progress_timeout_ms_{0};
    bool initialized_{false};
    ProgressSample last_progress_sample_{};
};

} // namespace autonomy
