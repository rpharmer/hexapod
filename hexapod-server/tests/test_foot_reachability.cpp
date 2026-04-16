#include "foot_reachability.hpp"
#include "geometry_config.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    const HexapodGeometry geo = geometry_config::buildDefaultHexapodGeometry();
    const LegGeometry& leg0 = geo.legGeometry[0];

    constexpr double inset = 0.004;
    const Vec3 coxa = leg0.bodyCoxaOffset;
    const Vec3 far = coxa + Vec3{0.35, 0.0, -0.12};
    const double d0 = foot_reachability::femurPlaneDistanceM(leg0, far);
    const Vec3 clamped = foot_reachability::clampFootPositionBody(leg0, far, inset);
    const double d1 = foot_reachability::femurPlaneDistanceM(leg0, clamped);

    if (!(d1 < d0 - 1e-6)) {
        std::cerr << "FAIL: clamp should reduce femur-plane distance\n";
        return EXIT_FAILURE;
    }

    const double max_r = leg0.femurLength.value + leg0.tibiaLength.value;
    if (d1 > max_r - inset + 1e-5) {
        std::cerr << "FAIL: clamped foot should respect inset inside max reach\n";
        return EXIT_FAILURE;
    }

    const Vec3 same = foot_reachability::clampFootPositionBody(leg0, clamped, 0.004);
    if (!nearlyEq(same.x, clamped.x) || !nearlyEq(same.y, clamped.y) || !nearlyEq(same.z, clamped.z)) {
        std::cerr << "FAIL: already-reachable foot should be unchanged\n";
        return EXIT_FAILURE;
    }

    Vec3 vel{0.5, 0.0, 0.0};
    foot_reachability::clipVelocityForReachClamp(far, clamped, &vel);
    const Vec3 outward = far - clamped;
    const double on = std::sqrt(outward.x * outward.x + outward.y * outward.y + outward.z * outward.z);
    const double v_along = (vel.x * outward.x + vel.y * outward.y + vel.z * outward.z) / std::max(on, 1e-12);
    if (v_along > 1e-4) {
        std::cerr << "FAIL: velocity should not push outward along clamp direction\n";
        return EXIT_FAILURE;
    }

    Vec3 vel2{0.01, 0.02, -0.03};
    const Vec3 vel2_copy = vel2;
    foot_reachability::clipVelocityForReachClamp(clamped, clamped, &vel2);
    if (!nearlyEq(vel2.x, vel2_copy.x) || !nearlyEq(vel2.y, vel2_copy.y) || !nearlyEq(vel2.z, vel2_copy.z)) {
        std::cerr << "FAIL: no clamp displacement should leave velocity unchanged\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
