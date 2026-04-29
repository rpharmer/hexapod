#pragma once

#include "motion_intent_utils.hpp"
#include "twist_field.hpp"
#include "types.hpp"

/** Merge processed locomotion twist with optional estimator twist when `RobotState` is valid. */
BodyVelocityCommand bodyVelocityForFootPlanning(const RobotState& est,
                                                const BodyTwist& cmd_twist,
                                                double foot_estimator_blend_01);

/** Body-frame velocity of a point fixed in the world at position `r_b` (small-body approximation). */
Vec3 supportFootVelocityAt(const Vec3& r_b, const BodyVelocityCommand& body);

// Stance: support-phase motion in the current body frame (foot moves opposite commanded body motion).
struct StanceFootInputs {
    Vec3 anchor{};
    Vec3 v_foot_body{};
    double phase{0.0};
    double f_hz{1.0};
};

// Stance: integrates twist-field support velocity (world-fixed foot in body frame). IMU / tilt
// leveling is applied in `BodyController` after `planStanceFoot`.
void planStanceFoot(const StanceFootInputs& in, Vec3& pos_body, Vec3& vel_body);

// Swing: planar cubic Bezier + optional time warp (`swing_trajectory`), vertical profile on same
// warped phase; touchdown XY is assembled as intent-only nominal stride plus one bounded capture
// correction, then this path decides how to move between liftoff and that foothold.
struct SwingFootInputs {
    Vec3 anchor{};
    Vec3 stance_end{};
    /** Foot velocity at liftoff (body frame), for C1 continuity with stance. */
    Vec3 v_liftoff_body{};
    double tau01{0.0};
    double swing_span{0.0};
    double f_hz{1.0};
    double step_length_m{0.0};
    double swing_height_m{0.0};
    double stride_ux{1.0};
    double stride_uy{0.0};
    /** Reserved feedforward command acceleration fields; swing capture no longer consumes them. */
    double cmd_accel_body_x_mps2{0.0};
    double cmd_accel_body_y_mps2{0.0};
    /** Extra horizon beyond `T_swing` for twist-based foothold prediction (e.g. fraction of stance). */
    double stance_lookahead_s{0.0};
    /** Static margin (m) for light stability bias on touchdown XY. */
    double static_stability_margin_m{0.0};
    /** From gait: S-curve amount on swing phase (see `swing_trajectory`). */
    double swing_time_ease_01{1.0};
};

/**
 * Decomposed swing foothold placement.
 * `nominal_body` is the intent-only stride target; `capture_raw_body` is the single corrective
 * channel before bounding; `capture_body` is the bounded correction actually applied.
 */
struct SwingFootPlanDecomposition {
    Vec3 nominal_body{};
    Vec3 capture_raw_body{};
    Vec3 capture_body{};
    Vec3 final_body{};
    double capture_limit_m{0.0};
};

/** Compute the nominal/capture swing foothold decomposition for tests and diagnostics. */
SwingFootPlanDecomposition computeSwingFootPlacement(const RobotState& est,
                                                     const BodyTwist& nominal_body,
                                                     const SwingFootInputs& in);

void planSwingFoot(const RobotState& est,
                   const BodyTwist& nominal_body,
                   const SwingFootInputs& in,
                   Vec3& pos_body,
                   Vec3& vel_body);
