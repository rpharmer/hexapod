#include "foot_reachability.hpp"

#include <cmath>

namespace foot_reachability {

double femurPlaneDistanceM(const LegGeometry& leg, const Vec3& foot_pos_body_m) {
    const Vec3 relative_to_coxa = foot_pos_body_m - leg.bodyCoxaOffset;
    const Mat3 r_leg = Mat3::rotZ(-leg.mountAngle.value);
    const Vec3 foot_leg = r_leg * relative_to_coxa;
    const double r = std::hypot(foot_leg.x, foot_leg.y);
    const double rho = r - leg.coxaLength.value;
    return std::hypot(rho, foot_leg.z);
}

Vec3 clampFootPositionBody(const LegGeometry& leg, const Vec3& foot_pos_body_m, const double inset_m) {
    const double m = std::max(0.0, inset_m);
    const Vec3 relative_to_coxa = foot_pos_body_m - leg.bodyCoxaOffset;
    const Mat3 r_leg = Mat3::rotZ(-leg.mountAngle.value);
    const Vec3 foot_leg = r_leg * relative_to_coxa;

    const double x = foot_leg.x;
    const double y = foot_leg.y;
    const double z = foot_leg.z;
    const double q1 = std::atan2(y, x);
    const double r = std::hypot(x, y);
    const double rho = r - leg.coxaLength.value;
    const double d = std::hypot(rho, z);

    const double min_reach = std::fabs(leg.femurLength.value - leg.tibiaLength.value);
    const double max_reach = leg.femurLength.value + leg.tibiaLength.value;
    const double d_min = min_reach + m;
    const double d_max = std::max(d_min + 1e-6, max_reach - m);

    if (d <= 1e-12) {
        return foot_pos_body_m;
    }

    double scale = 1.0;
    if (d > d_max) {
        scale = d_max / d;
    } else if (d < d_min) {
        scale = d_min / d;
    }

    if (std::abs(scale - 1.0) < 1e-12) {
        return foot_pos_body_m;
    }

    const double rho_s = rho * scale;
    const double z_s = z * scale;
    const double r_s = rho_s + leg.coxaLength.value;
    const Vec3 foot_leg_s{r_s * std::cos(q1), r_s * std::sin(q1), z_s};
    const Mat3 r_body = Mat3::rotZ(leg.mountAngle.value);
    return leg.bodyCoxaOffset + (r_body * foot_leg_s);
}

void clipVelocityForReachClamp(const Vec3& foot_before_body,
                               const Vec3& foot_after_body,
                               Vec3* vel_body_mps) {
    if (!vel_body_mps) {
        return;
    }
    const Vec3 outward = foot_before_body - foot_after_body;
    const double on = vecNorm(outward);
    if (on < 1e-9) {
        return;
    }
    const Vec3 outward_hat = outward * (1.0 / on);
    const double v_out = outward_hat.x * vel_body_mps->x + outward_hat.y * vel_body_mps->y +
                         outward_hat.z * vel_body_mps->z;
    if (v_out > 0.0) {
        *vel_body_mps = *vel_body_mps - outward_hat * v_out;
    }
}

} // namespace foot_reachability
