#include "support_assessment.hpp"

#include "leg_fk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr double kNominalReachFraction = 0.55;
constexpr double kReachMarginM = 0.005;
constexpr double kComFromBodyXYScale = 0.28;

double cross2(const SupportPoint2& origin, const SupportPoint2& a, const SupportPoint2& b) {
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

double dot2(const SupportPoint2& a, const SupportPoint2& b) {
    return a.x * b.x + a.y * b.y;
}

double len2(const SupportPoint2& a) {
    return std::hypot(a.x, a.y);
}

SupportPoint2 operator-(const SupportPoint2& a, const SupportPoint2& b) {
    return SupportPoint2{a.x - b.x, a.y - b.y};
}

SupportPoint2 operator+(const SupportPoint2& a, const SupportPoint2& b) {
    return SupportPoint2{a.x + b.x, a.y + b.y};
}

SupportPoint2 operator*(const SupportPoint2& a, const double scale) {
    return SupportPoint2{a.x * scale, a.y * scale};
}

double distPointSegment(const SupportPoint2& p, const SupportPoint2& a, const SupportPoint2& b) {
    const SupportPoint2 ab = b - a;
    const SupportPoint2 ap = p - a;
    const double ab2 = dot2(ab, ab);
    if (ab2 < 1e-18) {
        return len2(ap);
    }
    const double t = std::clamp(dot2(ap, ab) / ab2, 0.0, 1.0);
    const SupportPoint2 closest = a + ab * t;
    return len2(p - closest);
}

std::vector<SupportPoint2> convexHull(std::vector<SupportPoint2> points) {
    const int n = static_cast<int>(points.size());
    if (n <= 1) {
        return points;
    }
    std::sort(points.begin(), points.end(), [](const SupportPoint2& lhs, const SupportPoint2& rhs) {
        return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    });

    std::vector<SupportPoint2> lower;
    lower.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        while (static_cast<int>(lower.size()) >= 2 &&
               cross2(lower[lower.size() - 2], lower.back(), points[static_cast<std::size_t>(i)]) <= 0.0) {
            lower.pop_back();
        }
        lower.push_back(points[static_cast<std::size_t>(i)]);
    }

    std::vector<SupportPoint2> upper;
    upper.reserve(static_cast<std::size_t>(n));
    for (int i = n - 1; i >= 0; --i) {
        while (static_cast<int>(upper.size()) >= 2 &&
               cross2(upper[upper.size() - 2], upper.back(), points[static_cast<std::size_t>(i)]) <= 0.0) {
            upper.pop_back();
        }
        upper.push_back(points[static_cast<std::size_t>(i)]);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

bool pointInConvexPolygon(const SupportPoint2& p, const std::vector<SupportPoint2>& polygon) {
    const int n = static_cast<int>(polygon.size());
    if (n == 0) {
        return false;
    }
    if (n == 1) {
        return len2(p - polygon[0]) < 1e-6;
    }
    if (n == 2) {
        const double d = distPointSegment(p, polygon[0], polygon[1]);
        const SupportPoint2 ab = polygon[1] - polygon[0];
        const SupportPoint2 ap = p - polygon[0];
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
        const double cr =
            cross2(polygon[static_cast<std::size_t>(i)], polygon[static_cast<std::size_t>((i + 1) % n)], p);
        if (cr > 1e-9) {
            pos = true;
        }
        if (cr < -1e-9) {
            neg = true;
        }
    }
    return !(pos && neg);
}

double minDistanceToHullBoundary(const SupportPoint2& p, const std::vector<SupportPoint2>& hull) {
    const int n = static_cast<int>(hull.size());
    if (n == 0) {
        return 0.0;
    }
    if (n == 1) {
        return len2(p - hull[0]);
    }
    double min_distance = 1e9;
    for (int i = 0; i < n; ++i) {
        const SupportPoint2& a = hull[static_cast<std::size_t>(i)];
        const SupportPoint2& b = hull[static_cast<std::size_t>((i + 1) % n)];
        min_distance = std::min(min_distance, distPointSegment(p, a, b));
    }
    return min_distance;
}

double staticStabilityMargin(const SupportPoint2& com, const std::vector<SupportPoint2>& stance_feet) {
    if (stance_feet.empty()) {
        return -1.0;
    }
    if (stance_feet.size() == 1U) {
        return len2(com - stance_feet[0]);
    }
    if (stance_feet.size() == 2U) {
        const double edge_distance = distPointSegment(com, stance_feet[0], stance_feet[1]);
        return pointInConvexPolygon(com, stance_feet) ? edge_distance : -edge_distance;
    }
    const std::vector<SupportPoint2> hull = convexHull(stance_feet);
    const bool inside = pointInConvexPolygon(com, hull);
    const double edge_distance = minDistanceToHullBoundary(com, hull);
    return inside ? edge_distance : -edge_distance;
}

double supportPolygonClearance(const SupportPoint2& com,
                               const std::vector<SupportPoint2>& stance_feet,
                               const double margin_required_m,
                               const double inset_m) {
    const double required = std::max(0.0, margin_required_m + inset_m);
    if (stance_feet.empty()) {
        return -required;
    }
    if (stance_feet.size() == 1U) {
        return len2(com - stance_feet[0]) - required;
    }
    if (stance_feet.size() == 2U) {
        const double edge_distance = distPointSegment(com, stance_feet[0], stance_feet[1]);
        return pointInConvexPolygon(com, stance_feet) ? edge_distance - required : -edge_distance - required;
    }
    const std::vector<SupportPoint2> hull = convexHull(stance_feet);
    const double edge_distance = minDistanceToHullBoundary(com, hull);
    return pointInConvexPolygon(com, hull) ? edge_distance - required : -edge_distance - required;
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
        const double max_rho = std::sqrt(
            std::max(0.0, femur_tibia_reach * femur_tibia_reach - foot_z_in_leg_frame * foot_z_in_leg_frame));
        const double rho = std::min(desired_rho, max_rho);
        const Vec3 neutral_leg_frame{leg_geo.coxaLength.value + rho, 0.0, foot_z_in_leg_frame};
        const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
        nominal[static_cast<std::size_t>(leg)] = leg_geo.bodyCoxaOffset + (body_from_leg * neutral_leg_frame);
    }
    return nominal;
}

bool supportPhaseCanCarryLoad(const ContactPhase phase, const bool planned_stance) {
    switch (phase) {
        case ContactPhase::ConfirmedStance:
            return true;
        case ContactPhase::LostCandidate:
            return planned_stance;
        case ContactPhase::ContactCandidate:
        case ContactPhase::ExpectedTouchdown:
        case ContactPhase::Swing:
        case ContactPhase::Search:
        default:
            return false;
    }
}

bool hasSupportObservation(const RobotState& est) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        if (est.foot_contacts[leg]) {
            return true;
        }
        const ContactPhase phase = est.foot_contact_fusion[leg].phase;
        if (phase == ContactPhase::ConfirmedStance || phase == ContactPhase::LostCandidate) {
            return true;
        }
    }
    return false;
}

bool jointQualitySupportsFk(const JointStateQuality& quality) {
    if (!quality.position_valid) {
        return false;
    }
    return quality.source == JointStateSource::Measured ||
           quality.source == JointStateSource::Simulated ||
           (quality.source == JointStateSource::ObserverEstimate && quality.confidence >= 0.45);
}

} // namespace

SupportAssessment assessSupportState(const RobotState& est,
                                     const MotionIntent& intent,
                                     const GaitState& gait,
                                     const HexapodGeometry& geometry) {
    SupportAssessment out{};

    double body_height_m = intent.twist.body_trans_m.z;
    if (body_height_m <= 1e-6) {
        body_height_m = 0.14;
    }
    const Vec3 planar_offset{intent.twist.body_trans_m.x, intent.twist.body_trans_m.y, 0.0};
    const std::array<Vec3, kNumLegs> nominal = nominalStancePositions(geometry, body_height_m);
    LegFK fk{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 nominal_point = nominal[leg] - planar_offset;
        out.nominal_foot_xy_body_m[leg] = SupportPoint2{nominal_point.x, nominal_point.y};
        out.actual_foot_xy_body_m[leg] = out.nominal_foot_xy_body_m[leg];
        out.support_point_source[leg] = SupportPointSource::Nominal;
        if (jointQualitySupportsFk(est.joint_state_quality[leg])) {
            const FootTarget actual = fk.footInBodyFrame(est.leg_states[leg], geometry.legGeometry[leg]);
            out.actual_foot_xy_body_m[leg] = SupportPoint2{actual.pos_body_m.x, actual.pos_body_m.y};
            out.support_point_source[leg] = SupportPointSource::JointFk;
        }
        out.foot_xy_body_m[leg] = out.actual_foot_xy_body_m[leg];
    }

    out.projected_com_xy_m = SupportPoint2{
        intent.twist.body_trans_m.x * kComFromBodyXYScale,
        intent.twist.body_trans_m.y * kComFromBodyXYScale,
    };

    const bool use_support_observation = hasSupportObservation(est);
    std::vector<SupportPoint2> stance_now;
    std::vector<SupportPoint2> nominal_stance_now;
    stance_now.reserve(kNumLegs);
    nominal_stance_now.reserve(kNumLegs);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const bool planned_stance = gait.in_stance[leg];
        const bool contact_support = est.foot_contacts[leg];
        const ContactPhase phase = est.foot_contact_fusion[leg].phase;
        const bool phase_support = supportPhaseCanCarryLoad(phase, planned_stance);
        const bool support = use_support_observation ? (contact_support || phase_support) : planned_stance;
        const bool confirmed_support = use_support_observation &&
                                       support &&
                                       (contact_support || phase == ContactPhase::ConfirmedStance) &&
                                       phase != ContactPhase::Search &&
                                       phase != ContactPhase::ExpectedTouchdown &&
                                       phase != ContactPhase::LostCandidate;
        out.effective_support[leg] = support;
        if (use_support_observation) {
            if (phase == ContactPhase::Search) {
                ++out.search_leg_count;
            }
            if (phase == ContactPhase::LostCandidate) {
                ++out.lost_candidate_count;
            }
            if (phase == ContactPhase::ExpectedTouchdown) {
                ++out.expected_touchdown_count;
            }
        }
        if (support) {
            ++out.support_count;
            if (confirmed_support) {
                ++out.confirmed_support_count;
            }
            stance_now.push_back(out.foot_xy_body_m[leg]);
            nominal_stance_now.push_back(out.nominal_foot_xy_body_m[leg]);
        }
    }

    out.uncertain_support_count = out.support_count - out.confirmed_support_count;
    out.all_support_confirmed = out.support_count > 0 && out.uncertain_support_count == 0;
    out.nominal_static_margin_m = staticStabilityMargin(out.projected_com_xy_m, nominal_stance_now);
    out.actual_static_margin_m = staticStabilityMargin(out.projected_com_xy_m, stance_now);
    // Keep existing control behaviour on the nominal support metric until the actual-margin
    // controller path is explicitly enabled and retuned. The actual margin is still published
    // for diagnostics and A/B validation.
    out.static_margin_m = out.nominal_static_margin_m;
    out.sparse_support = out.support_count <= 3;
    out.degraded_support = out.sparse_support || out.static_margin_m < 0.0;
    return out;
}

double supportPolygonClearanceExcludingLeg(const SupportAssessment& support,
                                           const int excluded_leg,
                                           const double margin_required_m,
                                           const double inset_m) {
    std::vector<SupportPoint2> others;
    others.reserve(static_cast<std::size_t>(kNumLegs - 1));
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (leg == excluded_leg) {
            continue;
        }
        if (support.effective_support[static_cast<std::size_t>(leg)]) {
            others.push_back(support.foot_xy_body_m[static_cast<std::size_t>(leg)]);
        }
    }
    return supportPolygonClearance(support.projected_com_xy_m, others, margin_required_m, inset_m);
}
