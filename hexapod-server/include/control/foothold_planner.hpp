#pragma once

#include "twist_field.hpp"

/**
 * Twist-integrated foothold helpers (roadmap stages 5–6): combine rigid twist prediction with
 * light stability shaping; exact foot motion and timing remain in `foot_planners` / gait.
 */

/** Body-frame XY delta (m) from integrating world-fixed contact motion over `horizon_s`. */
Vec3 twistIntegratedFootholdDeltaXY(const BodyTwist& body, const Vec3& foot_ref_body, double horizon_s);

/**
 * When static margin is tight, nudge touchdown slightly toward the body horizontal origin to
 * reduce overturning moment (small conservative bias, meters in body XY).
 */
Vec3 stabilityFootholdBiasXY(double static_stability_margin_m, const Vec3& foot_ref_body);

/** Limit extra XY foothold correction so it cannot dominate the nominal stride. */
Vec3 clampFootholdExtraXY(const Vec3& extra_xy, double max_hypot_m);
