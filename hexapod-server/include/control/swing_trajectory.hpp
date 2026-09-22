#pragma once

/**
 * Swing path generator: horizontal cubic Bezier in the plane (foothold decides endpoints),
 * optional S-curve time warping for softer acceleration, vertical profile uses the same warped
 * phase so XY and Z stay coherent (foothold = where, this module = how).
 */

namespace swing_trajectory {

/**
 * A swing plan with every live input already resolved: Bezier endpoints, end
 * tangents, clearance and timing. Holding this for the duration of one swing is
 * what makes the commanded foot path continuous.
 *
 * Re-resolving per control sample is what produced the measured discontinuities
 * (leftover §3.15): the foothold capture term integrates the *measured* body
 * twist over `T_swing + stance_lookahead_s` (~0.95 s) and `stance_end` adds a
 * further `duty / f_hz` (~0.5 s) lever. Both are bounded in magnitude but not in
 * rate, so ordinary estimator noise of 10-75 mm/s moved the whole Bezier by up
 * to 92 mm between consecutive 5 ms samples — a commanded foot speed of 18 m/s
 * that no actuator can track.
 *
 * Scalars only, so the controller can hold one per leg without depending on the
 * planner's own include graph.
 */
struct SwingPlanCommit {
    bool valid{false};
    double p0x{0.0};
    double p0y{0.0};
    double p3x{0.0};
    double p3y{0.0};
    double m0x{0.0};
    double m0y{0.0};
    double m1x{0.0};
    double m1y{0.0};
    double swing_height_m{0.0};
    double anchor_y{0.0};
    double anchor_z{0.0};
    double swing_span{1.0};
    double f_hz{1.0};
    double time_ease{1.0};
};

/** Endpoint-slope-preserving S warp; `ease01=0` → identity, `ease01=1` → full shaping. */
double timeWarp(double tau01, double ease01);
double timeWarpDeriv(double tau01, double ease01);

/**
 * Cubic Bezier from P0→P3 with Hermite-equivalent end tangents in u-space:
 * d/du|0 = m0, d/du|1 = m1  ⇒  P1 = P0 + m0/3, P2 = P3 − m1/3.
 * Returns position and d/dtau (chain to d/dt is multiply by `f_hz/swing_span` as before).
 */
void evalSwingPlanarBezier(double tau01,
                           double time_ease01,
                           double p0x,
                           double p0y,
                           double p3x,
                           double p3y,
                           double m0x,
                           double m0y,
                           double m1x,
                           double m1y,
                           double* px,
                           double* py,
                           double* dpx_dtau,
                           double* dpy_dtau);

} // namespace swing_trajectory
