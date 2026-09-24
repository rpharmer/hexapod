#include "foot_reachability.hpp"

#include <algorithm>
#include <cmath>

namespace foot_reachability {
namespace {

struct AnnulusLimits {
    double d_min{0.0};
    double d_max{0.0};
};

struct FemurPlaneCoords {
    double q1{0.0};
    double rho{0.0};
    double z{0.0};
};

AnnulusLimits annulusLimits(const LegGeometry& leg, const double inset_m) {
    const double m = std::max(0.0, inset_m);
    const double min_reach = std::fabs(leg.femurLength.value - leg.tibiaLength.value);
    const double max_reach = leg.femurLength.value + leg.tibiaLength.value;
    AnnulusLimits out{};
    out.d_min = min_reach + m;
    out.d_max = std::max(out.d_min + 1e-6, max_reach - m);
    return out;
}

FemurPlaneCoords toFemurPlane(const LegGeometry& leg, const Vec3& foot_pos_body_m) {
    const Vec3 relative_to_coxa = foot_pos_body_m - leg.bodyCoxaOffset;
    const Mat3 r_leg = legFromBodyFrame(leg);
    const Vec3 foot_leg = r_leg * relative_to_coxa;
    FemurPlaneCoords out{};
    out.q1 = std::atan2(foot_leg.y, foot_leg.x);
    const double r = std::hypot(foot_leg.x, foot_leg.y);
    out.rho = r - leg.coxaLength.value;
    out.z = foot_leg.z;
    return out;
}

Vec3 fromFemurPlane(const LegGeometry& leg, const double q1, const double rho, const double z) {
    const double r = rho + leg.coxaLength.value;
    const Vec3 foot_leg{r * std::cos(q1), r * std::sin(q1), z};
    const Mat3 r_body = bodyFromLegFrame(leg);
    return leg.bodyCoxaOffset + (r_body * foot_leg);
}

bool inAnnulus(const LegGeometry& leg, const Vec3& foot_pos_body_m, const double inset_m) {
    const AnnulusLimits lim = annulusLimits(leg, inset_m);
    const double d = femurPlaneDistanceM(leg, foot_pos_body_m);
    return d + 1e-12 >= lim.d_min && d - 1e-12 <= lim.d_max;
}

bool projectZKeepingPlanar(const LegGeometry& leg,
                           const Vec3& desired_body_m,
                           const double inset_m,
                           Vec3* out_body_m) {
    if (!out_body_m) {
        return false;
    }
    const AnnulusLimits lim = annulusLimits(leg, inset_m);
    const FemurPlaneCoords f = toFemurPlane(leg, desired_body_m);
    const double d = std::hypot(f.rho, f.z);
    if (d + 1e-12 >= lim.d_min && d - 1e-12 <= lim.d_max) {
        *out_body_m = desired_body_m;
        return true;
    }
    if (std::abs(f.rho) > lim.d_max) {
        return false;
    }

    double z_new = f.z;
    if (d > lim.d_max) {
        const double z_lim2 = lim.d_max * lim.d_max - f.rho * f.rho;
        if (z_lim2 < 0.0) {
            return false;
        }
        const double z_lim = std::sqrt(z_lim2);
        z_new = std::clamp(f.z, -z_lim, z_lim);
    } else {
        const double z_need2 = lim.d_min * lim.d_min - f.rho * f.rho;
        if (z_need2 <= 0.0) {
            *out_body_m = desired_body_m;
            return true;
        }
        const double z_need = std::sqrt(z_need2);
        z_new = (f.z >= 0.0) ? z_need : -z_need;
        if (std::hypot(f.rho, z_new) > lim.d_max + 1e-12) {
            return false;
        }
    }
    *out_body_m = fromFemurPlane(leg, f.q1, f.rho, z_new);
    return inAnnulus(leg, *out_body_m, inset_m);
}

Vec3 intersectSegmentWithAnnulus(const LegGeometry& leg,
                                 const Vec3& last_in_reach_body_m,
                                 const Vec3& desired_body_m,
                                 const double inset_m) {
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (lo + hi);
        const Vec3 p = last_in_reach_body_m + (desired_body_m - last_in_reach_body_m) * mid;
        if (inAnnulus(leg, p, inset_m)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return last_in_reach_body_m + (desired_body_m - last_in_reach_body_m) * lo;
}

} // namespace

double femurPlaneDistanceM(const LegGeometry& leg, const Vec3& foot_pos_body_m) {
    const FemurPlaneCoords f = toFemurPlane(leg, foot_pos_body_m);
    return std::hypot(f.rho, f.z);
}

bool footInReachAnnulus(const LegGeometry& leg, const Vec3& foot_pos_body_m, const double inset_m) {
    return inAnnulus(leg, foot_pos_body_m, inset_m);
}

std::optional<double> planarTravelToReachBoundaryM(const LegGeometry& leg,
                                                    const Vec3& foot_pos_body_m,
                                                    const Vec3& direction_body_xy,
                                                    const double max_travel_m,
                                                    const double inset_m) {
    const double direction_norm = std::hypot(direction_body_xy.x, direction_body_xy.y);
    if (!std::isfinite(direction_norm) || direction_norm <= 1e-12
        || !std::isfinite(max_travel_m) || max_travel_m <= 0.0 || max_travel_m > 1.0
        || !std::isfinite(foot_pos_body_m.x) || !std::isfinite(foot_pos_body_m.y)
        || !std::isfinite(foot_pos_body_m.z)
        || !inAnnulus(leg, foot_pos_body_m, inset_m)) {
        return std::nullopt;
    }
    const Vec3 direction{direction_body_xy.x / direction_norm,
                         direction_body_xy.y / direction_norm, 0.0};
    // Scan before bisecting: along a radial ray the annulus may have an inner
    // unreachable interval followed by a second reachable interval. We want
    // the first boundary of the continuous stroke, not the final reachable point.
    constexpr double kProbeSpacingM = 0.001;
    const int probes = static_cast<int>(std::ceil(max_travel_m / kProbeSpacingM));
    double last_inside_m = 0.0;
    for (int i = 1; i <= probes; ++i) {
        const double distance_m = std::min(max_travel_m, i * kProbeSpacingM);
        if (!inAnnulus(leg, foot_pos_body_m + direction * distance_m, inset_m)) {
            double lo = last_inside_m;
            double hi = distance_m;
            for (int iteration = 0; iteration < 20; ++iteration) {
                const double mid = 0.5 * (lo + hi);
                if (inAnnulus(leg, foot_pos_body_m + direction * mid, inset_m)) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            return lo;
        }
        last_inside_m = distance_m;
    }
    return max_travel_m;
}

Vec3 clampFootPositionBody(const LegGeometry& leg, const Vec3& foot_pos_body_m, const double inset_m) {
    const AnnulusLimits lim = annulusLimits(leg, inset_m);
    const FemurPlaneCoords f = toFemurPlane(leg, foot_pos_body_m);
    const double d = std::hypot(f.rho, f.z);

    if (d <= 1e-12) {
        return foot_pos_body_m;
    }

    double scale = 1.0;
    if (d > lim.d_max) {
        scale = lim.d_max / d;
    } else if (d < lim.d_min) {
        scale = lim.d_min / d;
    }

    if (std::abs(scale - 1.0) < 1e-12) {
        return foot_pos_body_m;
    }

    return fromFemurPlane(leg, f.q1, f.rho * scale, f.z * scale);
}

StrokeAlongStrokeResult classifyAlongStroke(const Vec3& desired_body_m, const Vec3& out_body_m) {
    StrokeAlongStrokeResult out{};
    out.pos_body_m = out_body_m;
    const double xy = std::hypot(out_body_m.x - desired_body_m.x, out_body_m.y - desired_body_m.y);
    const double dz = std::abs(out_body_m.z - desired_body_m.z);
    if (xy > 1e-9) {
        out.planar_xy_hit = true;
    } else if (dz > 1e-9) {
        out.z_only_hit = true;
    }
    return out;
}

StrokeAlongStrokeResult clampFootPositionAlongStroke(const LegGeometry& leg,
                                                     const Vec3* last_in_reach_body_m,
                                                     const Vec3& desired_body_m,
                                                     const double inset_m) {
    if (inAnnulus(leg, desired_body_m, inset_m)) {
        return classifyAlongStroke(desired_body_m, desired_body_m);
    }

    Vec3 z_projected{};
    if (projectZKeepingPlanar(leg, desired_body_m, inset_m, &z_projected)) {
        return classifyAlongStroke(desired_body_m, z_projected);
    }

    if (last_in_reach_body_m != nullptr && inAnnulus(leg, *last_in_reach_body_m, inset_m)) {
        return classifyAlongStroke(
            desired_body_m,
            intersectSegmentWithAnnulus(leg, *last_in_reach_body_m, desired_body_m, inset_m));
    }

    return classifyAlongStroke(desired_body_m, clampFootPositionBody(leg, desired_body_m, inset_m));
}

StrokeAlongStrokeResult clampPlantedFootPosition(const LegGeometry& leg,
                                                const Vec3* last_in_reach_body_m,
                                                const Vec3& desired_body_m,
                                                const double inset_m) {
    if (inAnnulus(leg, desired_body_m, inset_m))
        return classifyAlongStroke(desired_body_m, desired_body_m);
    const auto lim = annulusLimits(leg, inset_m);
    const auto f = toFemurPlane(leg, desired_body_m);
    if (std::abs(f.z) <= lim.d_max) {
        if (last_in_reach_body_m) {
            Vec3 start = *last_in_reach_body_m;
            start.z = desired_body_m.z;
            if (inAnnulus(leg, start, inset_m)) {
                return classifyAlongStroke(desired_body_m,
                    intersectSegmentWithAnnulus(leg, start, desired_body_m, inset_m));
            }
        }
        const double rho_max = std::sqrt(std::max(0.0, lim.d_max * lim.d_max - f.z * f.z));
        const double rho_min = std::sqrt(std::max(0.0, lim.d_min * lim.d_min - f.z * f.z));
        const double rho = std::copysign(std::clamp(std::abs(f.rho), rho_min, rho_max), f.rho);
        return classifyAlongStroke(desired_body_m, fromFemurPlane(leg, f.q1, rho, f.z));
    }
    return clampFootPositionAlongStroke(leg, last_in_reach_body_m, desired_body_m, inset_m);
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
