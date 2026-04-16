#pragma once

#include "types.hpp"

namespace foot_reachability {

/** Femur–tibia plane distance `d = hypot(rho, z)` in the leg frame (same convention as `LegIK`). */
double femurPlaneDistanceM(const LegGeometry& leg, const Vec3& foot_pos_body_m);

/**
 * If the foot lies outside the femur+tibia annulus (with inset), scale (rho, z) toward the coxa
 * along the same direction in the leg plane — same closure as `LegIK`, applied in body frame
 * before IK for predictable footholds.
 */
Vec3 clampFootPositionBody(const LegGeometry& leg, const Vec3& foot_pos_body_m, double inset_m = 0.004);

/**
 * After `clampFootPositionBody`, remove the velocity component that pushes back outside the clamp
 * (unit direction from clamped foot toward the pre-clamp command), to avoid boundary spikes.
 */
void clipVelocityForReachClamp(const Vec3& foot_before_body, const Vec3& foot_after_body, Vec3* vel_body_mps);

} // namespace foot_reachability
