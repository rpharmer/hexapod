#pragma once

/**
 * Swing path generator: horizontal cubic Bezier in the plane (foothold decides endpoints),
 * optional S-curve time warping for softer acceleration, vertical profile uses the same warped
 * phase so XY and Z stay coherent (foothold = where, this module = how).
 */

namespace swing_trajectory {

/** Smoothstep; `ease01=0` → identity, `ease01=1` → full cubic S on tau. */
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
