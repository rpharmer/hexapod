#pragma once

#include "types.hpp"

#include <cmath>

namespace kinematics {

struct LegReachEnvelope {
    double min_reach_m{0.0};
    double max_reach_m{0.0};
};

inline LegReachEnvelope legReachEnvelope(const LegGeometry& leg) {
    return LegReachEnvelope{
        std::fabs(leg.femurLength.value - leg.tibiaLength.value),
        leg.femurLength.value + leg.tibiaLength.value};
}

inline double legReachUtilization(const Vec3& foot_leg_frame, const LegGeometry& leg) {
    const double r = std::hypot(foot_leg_frame.x, foot_leg_frame.y);
    const double rho = r - leg.coxaLength.value;
    const double d = std::hypot(rho, foot_leg_frame.z);
    const double max_reach = legReachEnvelope(leg).max_reach_m;
    if (max_reach <= 1e-9) {
        return 0.0;
    }
    return d / max_reach;
}

inline Vec3 clampFootToReachEnvelope(const Vec3& foot_leg_frame, const LegGeometry& leg) {
    const LegReachEnvelope envelope = legReachEnvelope(leg);

    const double x = foot_leg_frame.x;
    const double y = foot_leg_frame.y;
    const double z = foot_leg_frame.z;

    const double q1 = std::atan2(y, x);
    const double r = std::hypot(x, y);
    const double rho = r - leg.coxaLength.value;
    const double d = std::hypot(rho, z);

    const double clamped_d = clamp(d, envelope.min_reach_m, envelope.max_reach_m);
    const double scale = (d > 1e-9) ? (clamped_d / d) : 1.0;

    const double rho_solved = rho * scale;
    const double z_solved = z * scale;
    const double r_solved = leg.coxaLength.value + rho_solved;

    return Vec3{r_solved * std::cos(q1), r_solved * std::sin(q1), z_solved};
}

} // namespace kinematics
