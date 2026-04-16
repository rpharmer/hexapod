#include "swing_trajectory.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-7) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    double px = 0.0;
    double py = 0.0;
    double dpx = 0.0;
    double dpy = 0.0;
    swing_trajectory::evalSwingPlanarBezier(0.0, 0.0, 1.0, 2.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, &px, &py, &dpx, &dpy);
    if (!nearlyEq(px, 1.0) || !nearlyEq(py, 2.0)) {
        std::cerr << "FAIL: Bezier start\n";
        return EXIT_FAILURE;
    }

    swing_trajectory::evalSwingPlanarBezier(1.0, 0.0, 1.0, 2.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, &px, &py, &dpx, &dpy);
    if (!nearlyEq(px, 4.0) || !nearlyEq(py, 5.0)) {
        std::cerr << "FAIL: Bezier end\n";
        return EXIT_FAILURE;
    }

    const double m0x = 0.3;
    const double m0y = 0.0;
    swing_trajectory::evalSwingPlanarBezier(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, m0x, m0y, 0.0, 0.0, &px, &py, &dpx, &dpy);
    if (!nearlyEq(dpx, m0x) || !nearlyEq(dpy, m0y)) {
        std::cerr << "FAIL: tangent at u=0 should match m0 when ease=0\n";
        return EXIT_FAILURE;
    }

    const double w0 = swing_trajectory::timeWarp(0.25, 1.0);
    const double w1 = swing_trajectory::timeWarp(0.25, 0.0);
    if (!(std::abs(w0 - 0.25) > 0.02) || !nearlyEq(w1, 0.25)) {
        std::cerr << "FAIL: time warp ease should change interior mapping\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
