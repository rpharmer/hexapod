#include "foothold_planner.hpp"

#include <cmath>

namespace {

constexpr double kTwistHorizonGain = 0.20;
constexpr double kMarginSoftM = 0.022;
constexpr double kMarginHardM = 0.004;
constexpr double kStabilityInwardM = 0.010;

} // namespace

Vec3 twistIntegratedFootholdDeltaXY(const BodyTwist& body, const Vec3& foot_ref_body, const double horizon_s) {
    const Vec3 d = TwistField::worldFixedContactBodyDelta(body, foot_ref_body, horizon_s);
    return Vec3{d.x * kTwistHorizonGain, d.y * kTwistHorizonGain, 0.0};
}

Vec3 stabilityFootholdBiasXY(const double static_stability_margin_m, const Vec3& foot_ref_body) {
    const double u =
        1.0 - std::clamp((static_stability_margin_m - kMarginHardM) / std::max(kMarginSoftM - kMarginHardM, 1e-6),
                          0.0,
                          1.0);
    if (u <= 1e-9) {
        return Vec3{0.0, 0.0, 0.0};
    }
    const double r = std::hypot(foot_ref_body.x, foot_ref_body.y);
    if (r < 1e-6) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{-foot_ref_body.x / r * kStabilityInwardM * u,
                -foot_ref_body.y / r * kStabilityInwardM * u,
                0.0};
}

Vec3 clampFootholdExtraXY(const Vec3& extra_xy, const double max_hypot_m) {
    const double h = std::hypot(extra_xy.x, extra_xy.y);
    if (h <= max_hypot_m || h < 1e-12) {
        return extra_xy;
    }
    const double s = max_hypot_m / h;
    return Vec3{extra_xy.x * s, extra_xy.y * s, 0.0};
}
