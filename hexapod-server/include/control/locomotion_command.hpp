#pragma once

#include "control_config.hpp"
#include "twist_field.hpp"
#include "types.hpp"

struct PlanarMotionCommand;

/**
 * Assembles the same body-frame locomotion twist previously implied by planar walk commands plus
 * `MotionIntent::twist` body rates (translation velocity overlay and roll/pitch/yaw rates).
 */
BodyTwist rawLocomotionTwistFromIntent(const MotionIntent& intent, const PlanarMotionCommand& planar);

BodyTwist clampLocomotionTwist(const BodyTwist& in, const control_config::LocomotionCommandConfig& cfg);

/**
 * Shapes locomotion BodyTwist each control tick.
 *
 * When control_config::LocomotionCommandConfig::enable_chassis_accel_limit is true (default),
 * output velocities slew toward the clamped intent with bounded linear/angular acceleration in
 * all modes (including stand / walk exit). The legacy first-order filter is not applied in that
 * mode. When accel limiting is disabled, behavior matches the historical path: non-walk snaps to
 * target, walk entry snaps, and optional exponential smoothing applies only while walking.
 */
class LocomotionCommandProcessor {
public:
    explicit LocomotionCommandProcessor(control_config::LocomotionCommandConfig config = {});

    void setConfig(control_config::LocomotionCommandConfig config) { config_ = config; }
    void reset();

    /** Returns clamped-and-shaped locomotion twist for this tick. */
    BodyTwist update(const MotionIntent& intent,
                     const PlanarMotionCommand& planar,
                     TimePointUs clock_tick_us);

private:
    control_config::LocomotionCommandConfig config_{};
    BodyTwist filtered_{};
    TimePointUs last_clock_{};
    bool have_clock_{false};
    bool prev_walking_{false};
};
