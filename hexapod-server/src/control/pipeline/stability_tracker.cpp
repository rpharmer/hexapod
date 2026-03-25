#include "stability_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_config.hpp"

namespace {

struct Point2 {
    double x{0.0};
    double y{0.0};
};

double cross(const Point2& o, const Point2& a, const Point2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double edgeSignedDistance(const Point2& a, const Point2& b, const Point2& p) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len <= 1e-9) {
        return 0.0;
    }
    return (dx * (p.y - a.y) - dy * (p.x - a.x)) / len;
}

Point2 footPointInBodyFrame(const LegState& leg_state, const LegGeometry& geometry) {
    const auto& joints = leg_state.joint_state;
    const double q1 = joints[0].pos_rad.value;
    const double q2 = joints[1].pos_rad.value;
    const double q3 = joints[2].pos_rad.value;

    const double rho =
        geometry.femurLength.value * std::cos(q2) +
        geometry.tibiaLength.value * std::cos(q2 + q3);
    const double radial = geometry.coxaLength.value + rho;

    const double local_x = radial * std::cos(q1);
    const double local_y = radial * std::sin(q1);

    const double mount_c = std::cos(geometry.mountAngle.value);
    const double mount_s = std::sin(geometry.mountAngle.value);

    const double body_x = geometry.bodyCoxaOffset.x + mount_c * local_x - mount_s * local_y;
    const double body_y = geometry.bodyCoxaOffset.y + mount_s * local_x + mount_c * local_y;
    return Point2{body_x, body_y};
}


double polygonSignedArea(const std::vector<Point2>& polygon) {
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point2& a = polygon[i];
        const Point2& b = polygon[(i + 1) % polygon.size()];
        area += (a.x * b.y) - (b.x * a.y);
    }
    return 0.5 * area;
}

std::vector<Point2> convexHull(std::vector<Point2> points) {
    if (points.size() <= 1) {
        return points;
    }

    std::sort(points.begin(), points.end(), [](const Point2& lhs, const Point2& rhs) {
        if (lhs.x == rhs.x) {
            return lhs.y < rhs.y;
        }
        return lhs.x < rhs.x;
    });

    std::vector<Point2> hull;
    hull.reserve(points.size() * 2);

    for (const Point2& p : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    const std::size_t lower_size = hull.size();
    for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
        while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull[hull.size() - 1], *it) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(*it);
    }

    if (!hull.empty()) {
        hull.pop_back();
    }

    return hull;
}

} // namespace

StabilityAssessment assessStability(const RobotState& state) {
    StabilityAssessment assessment{};

    const HexapodGeometry geometry = defaultHexapodGeometry();

    std::vector<Point2> support_points;
    support_points.reserve(kNumLegs);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!state.foot_contacts[leg]) {
            continue;
        }

        support_points.push_back(footPointInBodyFrame(state.leg_states[leg], geometry.legGeometry[leg]));
    }

    assessment.support_contact_count = static_cast<int>(support_points.size());
    if (assessment.support_contact_count < 3) {
        return assessment;
    }

    const std::vector<Point2> support_polygon = convexHull(support_points);
    if (support_polygon.size() < 3) {
        return assessment;
    }

    assessment.has_support_polygon = true;

    const Point2 com_projection{0.0, 0.0};
    const double orientation_sign = (polygonSignedArea(support_polygon) >= 0.0) ? 1.0 : -1.0;

    double min_signed_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < support_polygon.size(); ++i) {
        const Point2& a = support_polygon[i];
        const Point2& b = support_polygon[(i + 1) % support_polygon.size()];
        const double signed_distance = orientation_sign * edgeSignedDistance(a, b, com_projection);
        min_signed_distance = std::min(min_signed_distance, signed_distance);
    }

    assessment.stability_margin_m = min_signed_distance;
    assessment.com_inside_support_polygon = (min_signed_distance >= 0.0);
    return assessment;
}
