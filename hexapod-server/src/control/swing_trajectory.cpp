#include "swing_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace swing_trajectory {

double timeWarp(const double tau01, const double ease01) {
    const double k = std::clamp(ease01, 0.0, 1.0);
    const double t = std::clamp(tau01, 0.0, 1.0);
    const double s = t * t * (3.0 - 2.0 * t);
    return (1.0 - k) * t + k * s;
}

double timeWarpDeriv(const double tau01, const double ease01) {
    const double k = std::clamp(ease01, 0.0, 1.0);
    const double t = std::clamp(tau01, 0.0, 1.0);
    const double ds_dt = 6.0 * t * (1.0 - t);
    return (1.0 - k) + k * ds_dt;
}

namespace {

void cubicBezierEval(const double u,
                     const double* px,
                     const double* py,
                     double* out_x,
                     double* out_y) {
    const double om = 1.0 - u;
    const double o2 = om * om;
    const double u2 = u * u;
    *out_x = o2 * om * px[0] + 3.0 * o2 * u * px[1] + 3.0 * om * u2 * px[2] + u2 * u * px[3];
    *out_y = o2 * om * py[0] + 3.0 * o2 * u * py[1] + 3.0 * om * u2 * py[2] + u2 * u * py[3];
}

void cubicBezierDeriv(const double u,
                      const double* px,
                      const double* py,
                      double* dx_du,
                      double* dy_du) {
    const double om = 1.0 - u;
    *dx_du = 3.0 * om * om * (px[1] - px[0]) + 6.0 * om * u * (px[2] - px[1]) + 3.0 * u * u * (px[3] - px[2]);
    *dy_du = 3.0 * om * om * (py[1] - py[0]) + 6.0 * om * u * (py[2] - py[1]) + 3.0 * u * u * (py[3] - py[2]);
}

} // namespace

void evalSwingPlanarBezier(const double tau01,
                           const double time_ease01,
                           const double p0x,
                           const double p0y,
                           const double p3x,
                           const double p3y,
                           const double m0x,
                           const double m0y,
                           const double m1x,
                           const double m1y,
                           double* px,
                           double* py,
                           double* dpx_dtau,
                           double* dpy_dtau) {
    const double u = timeWarp(tau01, time_ease01);
    const double du_dtau = timeWarpDeriv(tau01, time_ease01);

    const double p1x = p0x + m0x / 3.0;
    const double p1y = p0y + m0y / 3.0;
    const double p2x = p3x - m1x / 3.0;
    const double p2y = p3y - m1y / 3.0;

    const double bx[4] = {p0x, p1x, p2x, p3x};
    const double by[4] = {p0y, p1y, p2y, p3y};

    double xu = 0.0;
    double yu = 0.0;
    double dx_du = 0.0;
    double dy_du = 0.0;
    cubicBezierEval(u, bx, by, &xu, &yu);
    cubicBezierDeriv(u, bx, by, &dx_du, &dy_du);

    *px = xu;
    *py = yu;
    *dpx_dtau = dx_du * du_dtau;
    *dpy_dtau = dy_du * du_dtau;
}

} // namespace swing_trajectory
