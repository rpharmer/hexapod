#pragma once

#include "types.hpp"

class GaitScheduler {
public:
    GaitScheduler() = default;

    GaitState update(const EstimatedState& est, const MotionIntent& intent, const SafetyState& safety);

private:
    double wrap01(double x) const;
    double phase_accum_{0.0};
    uint64_t last_update_us_{0};
};