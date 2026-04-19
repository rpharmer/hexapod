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

/** First-order hold on the filtered locomotion twist between control ticks. */
class LocomotionCommandProcessor {
public:
    explicit LocomotionCommandProcessor(control_config::LocomotionCommandConfig config = {});

    void setConfig(control_config::LocomotionCommandConfig config) { config_ = config; }
    void reset();

    /**
     * Returns clamped-and-smoothed locomotion twist for this tick. When not walking, filtering
     * state tracks the raw command (no inertial smoothing) so walk entry starts from current intent.
     */
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
