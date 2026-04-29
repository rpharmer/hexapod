#include "locomotion_stability.hpp"

#include "config/geometry_config.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;
constexpr double kComFromBodyXYScale = 0.28;

struct Pt2 {
    double x{0.0};
    double y{0.0};
};

double planarBodyRateRadps(const RobotState& est) {
    if (!est.has_imu || !est.imu.valid) {
        return 0.0;
    }
    return std::hypot(est.imu.gyro_radps.x, est.imu.gyro_radps.y);
}

double cross2(const Pt2& O, const Pt2& A, const Pt2& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

double dot2(const Pt2& a, const Pt2& b) {
    return a.x * b.x + a.y * b.y;
}

double len2(const Pt2& a) {
    return std::hypot(a.x, a.y);
}

Pt2 operator-(const Pt2& a, const Pt2& b) {
    return Pt2{a.x - b.x, a.y - b.y};
}

Pt2 operator+(const Pt2& a, const Pt2& b) {
    return Pt2{a.x + b.x, a.y + b.y};
}

Pt2 operator*(const Pt2& a, const double s) {
    return Pt2{a.x * s, a.y * s};
}

double distPointSegment(const Pt2& p, const Pt2& a, const Pt2& b) {
    const Pt2 ab = b - a;
    const Pt2 ap = p - a;
    const double ab2 = dot2(ab, ab);
    if (ab2 < 1e-18) {
        return len2(ap);
    }
    const double t = std::clamp(dot2(ap, ab) / ab2, 0.0, 1.0);
    const Pt2 closest = a + ab * t;
    return len2(p - closest);
}

std::vector<Pt2> convexHull(std::vector<Pt2> pts) {
    const int n = static_cast<int>(pts.size());
    if (n <= 1) {
        return pts;
    }
    std::sort(pts.begin(), pts.end(), [](const Pt2& a, const Pt2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    std::vector<Pt2> lower;
    lower.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        while (static_cast<int>(lower.size()) >= 2 &&
               cross2(lower[lower.size() - 2], lower.back(), pts[static_cast<std::size_t>(i)]) <= 0.0) {
            lower.pop_back();
        }
        lower.push_back(pts[static_cast<std::size_t>(i)]);
    }
    std::vector<Pt2> upper;
    upper.reserve(static_cast<std::size_t>(n));
    for (int i = n - 1; i >= 0; --i) {
        while (static_cast<int>(upper.size()) >= 2 &&
               cross2(upper[upper.size() - 2], upper.back(), pts[static_cast<std::size_t>(i)]) <= 0.0) {
            upper.pop_back();
        }
        upper.push_back(pts[static_cast<std::size_t>(i)]);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

bool pointInConvexPolygon(const Pt2& p, const std::vector<Pt2>& v) {
    const int n = static_cast<int>(v.size());
    if (n == 0) {
        return false;
    }
    if (n == 1) {
        return len2(p - v[0]) < 1e-6;
    }
    if (n == 2) {
        const double d = distPointSegment(p, v[0], v[1]);
        const Pt2 ab = v[1] - v[0];
        const Pt2 ap = p - v[0];
        const double ab2 = dot2(ab, ab);
        if (ab2 < 1e-18) {
            return d < 1e-6;
        }
        const double t = dot2(ap, ab) / ab2;
        return t >= -1e-9 && t <= 1.0 + 1e-9 && d < 1e-3;
    }
    bool pos = false;
    bool neg = false;
    for (int i = 0; i < n; ++i) {
        const double cr = cross2(v[static_cast<std::size_t>(i)], v[static_cast<std::size_t>((i + 1) % n)], p);
        if (cr > 1e-9) {
            pos = true;
        }
        if (cr < -1e-9) {
            neg = true;
        }
    }
    return !(pos && neg);
}

double minDistanceToHullBoundary(const Pt2& p, const std::vector<Pt2>& hull) {
    const int n = static_cast<int>(hull.size());
    if (n == 0) {
        return 0.0;
    }
    if (n == 1) {
        return len2(p - hull[0]);
    }
    double dmin = 1e9;
    for (int i = 0; i < n; ++i) {
        const Pt2& a = hull[static_cast<std::size_t>(i)];
        const Pt2& b = hull[static_cast<std::size_t>((i + 1) % n)];
        dmin = std::min(dmin, distPointSegment(p, a, b));
    }
    return dmin;
}

double staticStabilityMargin(const Pt2& com, const std::vector<Pt2>& stance_feet) {
    if (stance_feet.empty()) {
        return -1.0;
    }
    if (stance_feet.size() == 1U) {
        return len2(com - stance_feet[0]);
    }
    if (stance_feet.size() == 2U) {
        const double d_edge = distPointSegment(com, stance_feet[0], stance_feet[1]);
        const bool on_seg = pointInConvexPolygon(com, stance_feet);
        return on_seg ? d_edge : -d_edge;
    }
    const std::vector<Pt2> hull = convexHull(stance_feet);
    const bool inside = pointInConvexPolygon(com, hull);
    const double d_edge = minDistanceToHullBoundary(com, hull);
    return inside ? d_edge : -d_edge;
}

double supportPolygonClearance(const Pt2& com,
                               const std::vector<Pt2>& stance_feet,
                               const double margin_required,
                               const double inset) {
    const double need = std::max(0.0, margin_required + inset);
    if (stance_feet.empty()) {
        return -need;
    }
    if (stance_feet.size() == 1U) {
        return len2(com - stance_feet[0]) - need;
    }
    if (stance_feet.size() == 2U) {
        if (!pointInConvexPolygon(com, stance_feet)) {
            return -distPointSegment(com, stance_feet[0], stance_feet[1]) - need;
        }
        return distPointSegment(com, stance_feet[0], stance_feet[1]) - need;
    }
    const std::vector<Pt2> hull = convexHull(stance_feet);
    if (!pointInConvexPolygon(com, hull)) {
        return -minDistanceToHullBoundary(com, hull) - need;
    }
    return minDistanceToHullBoundary(com, hull) - need;
}

std::array<Vec3, kNumLegs> nominalStancePositions(const HexapodGeometry& geometry, const double body_height_m) {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const double femur_tibia_reach =
            std::max(0.0, leg_geo.femurLength.value + leg_geo.tibiaLength.value - kReachMarginM);
        const double desired_rho = kNominalReachFraction * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);
        const double desired_foot_z_body = -body_height_m;
        double foot_z_in_leg_frame = desired_foot_z_body - leg_geo.bodyCoxaOffset.z;
        if (std::abs(foot_z_in_leg_frame) > femur_tibia_reach) {
            foot_z_in_leg_frame = std::copysign(femur_tibia_reach, foot_z_in_leg_frame);
        }
        const double max_rho =
            std::sqrt(std::max(0.0, femur_tibia_reach * femur_tibia_reach - foot_z_in_leg_frame * foot_z_in_leg_frame));
        const double rho = std::min(desired_rho, max_rho);
        const Vec3 neutral_leg_frame{leg_geo.coxaLength.value + rho, 0.0, foot_z_in_leg_frame};
        const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
        nominal[leg] = leg_geo.bodyCoxaOffset + (body_from_leg * neutral_leg_frame);
    }
    return nominal;
}

} // namespace

LocomotionStability::LocomotionStability(LocomotionStabilityConfig config)
    : config_(config) {}

void LocomotionStability::reset() {
}

void LocomotionStability::apply(const RobotState& est, const MotionIntent& intent, GaitState& gait) {
    gait.stability_hold_stance.fill(false);
    gait.support_liftoff_clearance_m.fill(0.0);
    gait.support_liftoff_safe_to_lift.fill(false);
    gait.static_stability_margin_m = 0.0;

    const bool walking = (intent.requested_mode == RobotMode::WALK);
    if (!walking) {
        return;
    }

    double body_h = intent.twist.body_trans_m.z;
    if (body_h <= 1e-6) {
        body_h = 0.14;
    }
    const Vec3 planar_off{intent.twist.body_trans_m.x, intent.twist.body_trans_m.y, 0.0};
    const HexapodGeometry geo = geometry_config::activeHexapodGeometry();
    const std::array<Vec3, kNumLegs> nominal = nominalStancePositions(geo, body_h);

    std::array<Pt2, kNumLegs> foot_xy{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 p = nominal[static_cast<std::size_t>(leg)] - planar_off;
        foot_xy[static_cast<std::size_t>(leg)] = Pt2{p.x, p.y};
    }

    const Pt2 com{intent.twist.body_trans_m.x * kComFromBodyXYScale,
                  intent.twist.body_trans_m.y * kComFromBodyXYScale};

    std::vector<Pt2> stance_now;
    stance_now.reserve(static_cast<std::size_t>(kNumLegs));
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (gait.in_stance[leg]) {
            stance_now.push_back(foot_xy[static_cast<std::size_t>(leg)]);
        }
    }

    gait.static_stability_margin_m = staticStabilityMargin(com, stance_now);

    double margin_need = config_.min_margin_required_m;
    bool emergency_tilt_hold = false;
    double tilt_scale = 1.0;
    if (est.has_body_twist_state) {
        const double roll = est.body_twist_state.twist_pos_rad.x;
        const double pitch = est.body_twist_state.twist_pos_rad.y;
        if (std::isfinite(roll) && std::isfinite(pitch)) {
            const double tilt_mag = std::hypot(roll, pitch);
            // When the chassis is already leaning, require extra support margin before allowing
            // another leg to leave the ground. This keeps the gait from chasing a growing tilt
            // with the same lift pattern that caused it.
            margin_need += std::clamp(0.04 * tilt_mag, 0.0, 0.05);
            const double tilt_over = std::max(0.0, tilt_mag - 0.12);
            tilt_scale = std::clamp(1.0 - 0.75 * tilt_over, 0.45, 1.0);
            emergency_tilt_hold = tilt_mag > 0.30;
        }
    }
    if (gait.stride_phase_rate_hz.value < config_.slow_stride_hz_threshold) {
        margin_need *= config_.slow_gait_margin_multiplier;
    }

    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const double commanded_planar_speed_mps = std::hypot(cmd.vx_mps, cmd.vy_mps);
    const double commanded_yaw_rate_radps = std::abs(cmd.yaw_rate_radps);
    const bool high_activity_command = commanded_planar_speed_mps >= 0.18 || commanded_yaw_rate_radps >= 0.35;

    double body_rate_scale = 1.0;
    if (high_activity_command && est.has_imu && est.imu.valid) {
        const double body_rate = planarBodyRateRadps(est);
        if (std::isfinite(body_rate)) {
            constexpr double kBodyRateSoftStartRadps = 0.60;
            constexpr double kBodyRateScaleSlope = 0.45;
            const double body_rate_over = std::max(0.0, body_rate - kBodyRateSoftStartRadps);
            body_rate_scale = std::clamp(1.0 - kBodyRateScaleSlope * body_rate_over, 0.45, 1.0);
        }
    }

    const double gait_activity_scale = std::min(tilt_scale, body_rate_scale);
    gait.step_length_m *= gait_activity_scale;
    gait.stride_phase_rate_hz.value *= std::clamp(0.75 + 0.25 * gait_activity_scale, 0.55, 1.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        std::vector<Pt2> others;
        others.reserve(static_cast<std::size_t>(kNumLegs - 1));
        for (int j = 0; j < kNumLegs; ++j) {
            if (j == leg) {
                continue;
            }
            if (gait.in_stance[j]) {
                others.push_back(foot_xy[static_cast<std::size_t>(j)]);
            }
        }

        const double lift_clearance_m =
            supportPolygonClearance(com, others, margin_need, config_.support_inset_m);
        const bool can_lift = lift_clearance_m >= 0.0 && !emergency_tilt_hold;
        gait.support_liftoff_clearance_m[static_cast<std::size_t>(leg)] = lift_clearance_m;
        gait.support_liftoff_safe_to_lift[static_cast<std::size_t>(leg)] = can_lift;
        gait.stability_hold_stance[static_cast<std::size_t>(leg)] = !can_lift;
    }
}
