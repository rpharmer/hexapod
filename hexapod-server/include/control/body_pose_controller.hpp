#pragma once

#include "motion_intent_utils.hpp"
#include "types.hpp"

struct BodyPoseSetpoint {
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};
    double body_height_m{0.05};
};

/** Base orientation and height from `MotionIntent`, plus motion-dependent lean scaled by stability margin. */
BodyPoseSetpoint computeBodyPoseSetpoint(const MotionIntent& intent,
                                         const PlanarMotionCommand& cmd,
                                         double static_stability_margin_m,
                                         double stride_phase_rate_hz);
