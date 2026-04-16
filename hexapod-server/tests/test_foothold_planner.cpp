#include "foothold_planner.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    BodyTwist body{};
    body.linear_mps = Vec3{0.1, 0.0, 0.0};
    body.angular_radps = Vec3{0.0, 0.0, 0.0};
    const Vec3 foot{0.0, 0.1, -0.05};
    const Vec3 d = twistIntegratedFootholdDeltaXY(body, foot, 0.2);
    if (!nearlyEq(d.z, 0.0) || d.x >= 0.0) {
        std::cerr << "FAIL: twist-integrated foothold should oppose forward motion in x\n";
        return EXIT_FAILURE;
    }

    const Vec3 c = clampFootholdExtraXY(Vec3{0.3, 0.4, 0.0}, 0.5);
    const double h = std::hypot(c.x, c.y);
    if (h > 0.5 + 1e-9) {
        std::cerr << "FAIL: clamp hypot\n";
        return EXIT_FAILURE;
    }

    const Vec3 b = stabilityFootholdBiasXY(0.0, Vec3{0.1, 0.0, 0.0});
    if (b.x >= 0.0) {
        std::cerr << "FAIL: low margin should bias inward (negative x for +x foot)\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
