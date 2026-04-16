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

// Swing: Hermite XY (C1 cubic, liftoff tangent match), vertical swing shape, Raibert + accel capture,
// plus twist-integrated foothold lookahead (`foothold_planner`) for curved / combined motion.
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
    /** Body XY commanded acceleration (m/s²) for touchdown shift. */
    double cmd_accel_body_x_mps2{0.0};
    double cmd_accel_body_y_mps2{0.0};
    /** Extra horizon beyond `T_swing` for twist-based foothold prediction (e.g. fraction of stance). */
    double stance_lookahead_s{0.0};
    /** Static margin (m) for light stability bias on touchdown XY. */
    double static_stability_margin_m{0.0};
};

void planSwingFoot(const BodyVelocityCommand& body, const SwingFootInputs& in, Vec3& pos_body, Vec3& vel_body);
