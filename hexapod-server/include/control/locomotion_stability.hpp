#pragma once

#include "types.hpp"

struct LocomotionStabilityConfig {
    /** Minimum clearance from projected COM to support boundary when allowing a liftoff (m). */
    double min_margin_required_m{0.008};
    /** Extra inward margin applied when testing support polygons (m). */
    double support_inset_m{0.002};
    /** Phase span (cycle fraction) before liftoff where support checks gate the swing. */
    double liftoff_gate_phase_span{0.07};
    /** Stride rates below this (Hz) use stricter margin requirements. */
    double slow_stride_hz_threshold{0.78};
    /** Multiplier on `min_margin_required_m` during slow gaits. */
    double slow_gait_margin_multiplier{0.85};
};

class LocomotionStability {
public:
    explicit LocomotionStability(LocomotionStabilityConfig config = {});

    /** Updates stance hold flags, per-leg liftoff clearance diagnostics, and the static support margin. */
    void apply(const RobotState& est, const MotionIntent& intent, GaitState& gait);

    void reset();

private:
    LocomotionStabilityConfig config_{};
};
