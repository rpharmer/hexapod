#include "foot_reachability.hpp"
#include "geometry_config.hpp"
#include "body_controller.hpp"

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
    for (const auto& leg : geo.legGeometry) {
        for (int direction = 0; direction < 16; ++direction) {
            const double angle = direction * 2.0 * 3.141592653589793 / 16;
            const Vec3 requested = leg.bodyCoxaOffset + Vec3{.3 * std::cos(angle), .3 * std::sin(angle), -.13};
            const auto planted = foot_reachability::clampPlantedFootPosition(leg, nullptr, requested, inset);
            const auto repeated = foot_reachability::clampPlantedFootPosition(leg, &planted.pos_body_m, requested, inset);
            if (!nearlyEq(planted.pos_body_m.z, requested.z, 1e-9)
                || !nearlyEq(repeated.pos_body_m.z, requested.z, 1e-9)
                || !planted.planar_xy_hit
                || !foot_reachability::footInReachAnnulus(leg, planted.pos_body_m, inset)
                || !foot_reachability::footInReachAnnulus(leg, repeated.pos_body_m, inset)) {
                std::cerr << "FAIL: a reachable support height must survive planar reach limiting\n";
                return EXIT_FAILURE;
            }
        }
    }
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

    const Vec3 in_reach = clamped;
    const auto along_stroke_same =
        foot_reachability::clampFootPositionAlongStroke(leg0, &in_reach, in_reach, inset);
    if (!nearlyEq(along_stroke_same.pos_body_m.x, in_reach.x) ||
        !nearlyEq(along_stroke_same.pos_body_m.y, in_reach.y) ||
        !nearlyEq(along_stroke_same.pos_body_m.z, in_reach.z)) {
        std::cerr << "FAIL: in-reach desired should be unchanged by the stroke projector\n";
        return EXIT_FAILURE;
    }
    if (along_stroke_same.planar_xy_hit || along_stroke_same.z_only_hit) {
        std::cerr << "FAIL: in-reach desired should be a no-op, not a z-only or XY hit\n";
        return EXIT_FAILURE;
    }

    const auto missing_last = foot_reachability::clampFootPositionAlongStroke(leg0, nullptr, far, inset);
    if (!nearlyEq(missing_last.pos_body_m.x, clamped.x) || !nearlyEq(missing_last.pos_body_m.y, clamped.y) ||
        !nearlyEq(missing_last.pos_body_m.z, clamped.z)) {
        std::cerr << "FAIL: missing last in-reach should fall back to coxa-radial clamp\n";
        return EXIT_FAILURE;
    }
    if (!missing_last.planar_xy_hit) {
        std::cerr << "FAIL: radial fallback of an out-of-reach plant should be a planar-XY hit\n";
        return EXIT_FAILURE;
    }

    Vec3 z_only_desired = in_reach;
    z_only_desired.z -= 0.20;
    if (foot_reachability::footInReachAnnulus(leg0, z_only_desired, inset)) {
        std::cerr << "FAIL: z-only fixture desired should be outside the annulus\n";
        return EXIT_FAILURE;
    }
    const auto z_only =
        foot_reachability::clampFootPositionAlongStroke(leg0, &in_reach, z_only_desired, inset);
    if (!foot_reachability::footInReachAnnulus(leg0, z_only.pos_body_m, inset)) {
        std::cerr << "FAIL: z-only projector must land on the annulus\n";
        return EXIT_FAILURE;
    }
    if (z_only.planar_xy_hit || !z_only.z_only_hit) {
        std::cerr << "FAIL: keep-XY z projection must be z-only, not a planar-XY hit\n";
        return EXIT_FAILURE;
    }
    if (!nearlyEq(z_only.pos_body_m.x, z_only_desired.x, 1e-6) ||
        !nearlyEq(z_only.pos_body_m.y, z_only_desired.y, 1e-6)) {
        std::cerr << "FAIL: z-only projector should keep body XY\n";
        return EXIT_FAILURE;
    }

    const Vec3 last = coxa + (clamped - coxa) * 0.65;
    if (!foot_reachability::footInReachAnnulus(leg0, last, inset)) {
        std::cerr << "FAIL: stroke-projector last foothold should be inside the annulus\n";
        return EXIT_FAILURE;
    }
    const Vec3 desired = last + Vec3{0.12, 0.0, -0.08};
    if (foot_reachability::footInReachAnnulus(leg0, desired, inset)) {
        std::cerr << "FAIL: stroke-projector fixture desired should be outside the annulus\n";
        return EXIT_FAILURE;
    }
    const Vec3 radial_desired = foot_reachability::clampFootPositionBody(leg0, desired, inset);
    const auto stroke_desired =
        foot_reachability::clampFootPositionAlongStroke(leg0, &last, desired, inset);
    if (!foot_reachability::footInReachAnnulus(leg0, stroke_desired.pos_body_m, inset)) {
        std::cerr << "FAIL: stroke projector must land on the annulus\n";
        return EXIT_FAILURE;
    }
    const double stroke_xy_err =
        std::hypot(stroke_desired.pos_body_m.x - desired.x, stroke_desired.pos_body_m.y - desired.y);
    const double radial_xy_err = std::hypot(radial_desired.x - desired.x, radial_desired.y - desired.y);
    if (!(stroke_xy_err + 1e-3 < radial_xy_err)) {
        std::cerr << "FAIL: stroke projector should keep more planar stroke than coxa-radial clamp\n";
        return EXIT_FAILURE;
    }

    const Vec3 xy_desired = last + Vec3{0.30, 0.0, 0.0};
    const auto xy_hit = foot_reachability::clampFootPositionAlongStroke(leg0, &last, xy_desired, inset);
    if (!xy_hit.planar_xy_hit || xy_hit.z_only_hit) {
        std::cerr << "FAIL: out-of-reach planar stroke should be a planar-XY hit\n";
        return EXIT_FAILURE;
    }

    const auto nominal = computeNominalStance(geo, 0.14);
    constexpr double lean_pitch_rad = 0.13;
    const Mat3 body_rotation =
        (Mat3::rotZ(0.0) * Mat3::rotY(lean_pitch_rad) * Mat3::rotX(0.0)).transpose();
    auto planar_rho = [](const LegGeometry& leg, const Vec3& foot) {
        const Vec3 rel = foot - leg.bodyCoxaOffset;
        const Vec3 foot_leg = legFromBodyFrame(leg) * rel;
        return std::hypot(foot_leg.x, foot_leg.y) - leg.coxaLength.value;
    };
    bool found_origin_worse_d = false;
    bool found_origin_worse_rho = false;
    bool coxa_all_in = true;
    for (int i = 0; i < kNumLegs; ++i) {
        const LegGeometry& leg = geo.legGeometry[i];
        const Vec3 coxa_i = leg.bodyCoxaOffset;
        // 0.14 m nominal is z-limited onto d_max - 1 mm; inset so lean's d shift still fits.
        const Vec3 stance = coxa_i + (nominal[static_cast<std::size_t>(i)] - coxa_i) * 0.80;
        if (!foot_reachability::footInReachAnnulus(leg, stance, inset)) {
            std::cerr << "FAIL: inset 0.14 m stance should start in the annulus\n";
            return EXIT_FAILURE;
        }
        const Vec3 origin_rot = body_rotation * stance;
        const Vec3 coxa_rot = coxa_i + (body_rotation * (stance - coxa_i));
        const double d_origin = foot_reachability::femurPlaneDistanceM(leg, origin_rot);
        const double d_coxa = foot_reachability::femurPlaneDistanceM(leg, coxa_rot);
        if (d_origin > d_coxa + 1e-4) {
            found_origin_worse_d = true;
        }
        if (std::abs(planar_rho(leg, origin_rot)) > std::abs(planar_rho(leg, coxa_rot)) + 1e-4) {
            found_origin_worse_rho = true;
        }
        coxa_all_in = coxa_all_in && foot_reachability::footInReachAnnulus(leg, coxa_rot, inset);
    }
    if (!found_origin_worse_d && !found_origin_worse_rho) {
        std::cerr << "FAIL: origin rotation should consume more femur-plane reach than coxa rotation\n";
        return EXIT_FAILURE;
    }
    if (!coxa_all_in) {
        std::cerr << "FAIL: coxa-centered lean should keep stance feet in the annulus\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
