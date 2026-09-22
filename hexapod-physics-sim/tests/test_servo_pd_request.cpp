#include "demo/servo_pd_request.hpp"
#include <cmath>
#include <iostream>

int main() {
    using minphys3d::demo::servoPdRequest;
    bool ok = true;
    for (double v : {-8., -1., 1., 8.}) {
        const double healthy = servoPdRequest(1., .08, 0., v, 1., 1.);
        const double preserved = servoPdRequest(1., .08, 0., v, .5, 1.);
        ok &= healthy == preserved && preserved*v < 0.;
        ok &= servoPdRequest(1., .08, .5, v, .5, 1.) == .25-.08*v;
    }
    // Work added by halving damping is positive irrespective of rotation sign.
    const double oldRetry = servoPdRequest(1., .08, .7, 8., .5, .5);
    const double safeRetry = servoPdRequest(1., .08, .7, 8., .5, 1.);
    ok &= oldRetry > 0. && safeRetry < 0.;
    if (!ok) std::cerr << "FAIL: reducing proportional demand must not remove damping\n";
    return ok ? 0 : 1;
}
