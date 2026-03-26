#include "stability_tracker.hpp"

#include <array>
#include <limits>
#include <vector>

#include "geometry_config.hpp"
#include "leg_kinematics_utils.hpp"

namespace {

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

        support_points.push_back(kinematics::footPointInBodyPlane(
            state.leg_states[leg], geometry.legGeometry[leg]));
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
