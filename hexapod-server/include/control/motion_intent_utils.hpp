#pragma once

#include "scenario_driver.hpp"
#include "twist_field.hpp"
#include "types.hpp"

struct PlanarMotionCommand {
    double vx_mps{};
    double vy_mps{};
    double yaw_rate_radps{};
};

PlanarMotionCommand planarMotionCommand(const MotionIntent& intent);

/** Planar slice of a processed locomotion twist (for gait scaling / body lean helpers). */
PlanarMotionCommand planarMotionFromCommandTwist(const BodyTwist& cmd);

MotionIntent makeMotionIntent(RobotMode mode, GaitType gait, double body_height_m);
MotionIntent makeMotionIntent(const ScenarioMotionIntent& motion);

/** Strict freshness requires nonzero monotonic `sample_id`; call before each `setMotionIntent`. */
void stampIntentStreamMotionFields(MotionIntent& intent);
