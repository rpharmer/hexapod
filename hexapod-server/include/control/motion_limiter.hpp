#pragma once

#include "types.hpp"

struct MotionLimiterConfig {
    bool enabled{true};
    Vec3 linear_accel_max_mps2{0.8, 0.8, 0.8};
    Vec3 angular_accel_max_radps2{1.5, 1.5, 1.5};
};

class MotionLimiter {
public:
    explicit MotionLimiter(MotionLimiterConfig config = {});

    MotionIntent limit(const MotionIntent& intent, const TimePointUs& now);
    bool enabled() const;

private:
    static double clampAxisDelta(double previous, double commanded, double accel_limit, double dt_s);

    MotionLimiterConfig config_{};
    bool has_previous_command_{false};
    TimePointUs previous_update_timestamp_{};
    Vec3 previous_body_trans_mps_{};
    Vec3 previous_angular_velocity_radps_{};
};
