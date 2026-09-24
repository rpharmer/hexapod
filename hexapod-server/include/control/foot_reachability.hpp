#pragma once

#include "types.hpp"

#include <optional>

namespace foot_reachability {

/** Femur–tibia plane distance `d = hypot(rho, z)` in the leg frame (same convention as `LegIK`). */
double femurPlaneDistanceM(const LegGeometry& leg, const Vec3& foot_pos_body_m);

/** True when `d = hypot(rho, z)` lies in the inset femur–tibia annulus. */
bool footInReachAnnulus(const LegGeometry& leg, const Vec3& foot_pos_body_m, double inset_m = 0.004);

/** Diagnostic query: continuous XY travel from an in-reach foot at fixed body-frame z,
 * along `direction_body_xy`, up to `max_travel_m`. Returns nullopt for an invalid
 * direction or an out-of-reach start. This is annulus reach only, not joint limits,
 * collision clearance, support-polygon margin, or dynamic tracking capacity. */
std::optional<double> planarTravelToReachBoundaryM(const LegGeometry& leg,
                                                    const Vec3& foot_pos_body_m,
                                                    const Vec3& direction_body_xy,
                                                    double max_travel_m = 0.25,
                                                    double inset_m = 0.004);

/**
 * If the foot lies outside the femur+tibia annulus (with inset), scale (rho, z) toward the coxa
 * along the same direction in the leg plane — same closure as `LegIK`, applied in body frame
 * before IK for predictable footholds.
 */
Vec3 clampFootPositionBody(const LegGeometry& leg, const Vec3& foot_pos_body_m, double inset_m = 0.004);

struct StrokeAlongStrokeResult {
    Vec3 pos_body_m{};
    bool planar_xy_hit{false};
    bool z_only_hit{false};
};

/**
 * Project `desired` onto the inset annulus without coxa-radial scaling when a reachable
 * previous foothold is available. Prefer keeping body XY (solve z onto the annulus);
 * otherwise intersect the segment `last_in_reach → desired`. `last_in_reach_body_m == nullptr`
 * or an out-of-reach last pose falls back to `clampFootPositionBody`.
 * `planar_xy_hit` is true when body XY changed; `z_only_hit` is true when only z changed.
 */
StrokeAlongStrokeResult clampFootPositionAlongStroke(const LegGeometry& leg,
                                                     const Vec3* last_in_reach_body_m,
                                                     const Vec3& desired_body_m,
                                                     double inset_m = 0.004);

/** Planted-foot variant: preserve requested height whenever the horizontal
 * reach slice exists, shortening XY instead of manufacturing a stance lift.
 * An unreachable height falls back to the general stroke projector. */
StrokeAlongStrokeResult clampPlantedFootPosition(const LegGeometry& leg,
                                                const Vec3* last_in_reach_body_m,
                                                const Vec3& desired_body_m,
                                                double inset_m = 0.004);

/**
 * After `clampFootPositionBody`, remove the velocity component that pushes back outside the clamp
 * (unit direction from clamped foot toward the pre-clamp command), to avoid boundary spikes.
 */
void clipVelocityForReachClamp(const Vec3& foot_before_body, const Vec3& foot_after_body, Vec3* vel_body_mps);

} // namespace foot_reachability
