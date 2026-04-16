#include "twist_field.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    BodyTwist twist{};
    twist.linear_mps = Vec3{0.2, 0.0, 0.0};
    twist.angular_radps = Vec3{0.0, 0.0, 0.5};
    const Vec3 p{0.0, 0.15, 0.0};

    const Vec3 v_point = TwistField::pointVelocity(twist, p);
    // v + w×p = (0.2,0,0) + (0,0,0.5)×(0,0.15,0) = (0.2,0,0) + (-0.075, 0, 0) = (0.125, 0, 0)
    if (!nearlyEq(v_point.x, 0.125) || !nearlyEq(v_point.y, 0.0) || !nearlyEq(v_point.z, 0.0)) {
        std::cerr << "FAIL: pointVelocity\n";
        return EXIT_FAILURE;
    }

    const Vec3 v_stance = TwistField::stanceFootVelocity(twist, p);
    if (!nearlyEq(v_stance.x, -0.125) || !nearlyEq(v_stance.y, 0.0) || !nearlyEq(v_stance.z, 0.0)) {
        std::cerr << "FAIL: stanceFootVelocity should negate point velocity\n";
        return EXIT_FAILURE;
    }

    const Vec3 delta = TwistField::worldFixedContactBodyDelta(twist, p, 0.5);
    if (!nearlyEq(delta.x, v_stance.x * 0.5) || !nearlyEq(delta.y, 0.0) || !nearlyEq(delta.z, 0.0)) {
        std::cerr << "FAIL: worldFixedContactBodyDelta\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
