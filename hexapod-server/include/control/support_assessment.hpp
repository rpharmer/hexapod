#pragma once

#include "types.hpp"

#include <array>

struct SupportPoint2 {
    double x{0.0};
    double y{0.0};
};

struct SupportAssessment {
    std::array<bool, kNumLegs> effective_support{};
    std::array<SupportPoint2, kNumLegs> foot_xy_body_m{};
    SupportPoint2 projected_com_xy_m{};
    double static_margin_m{0.0};
    int support_count{0};
    int confirmed_support_count{0};
    int uncertain_support_count{0};
    int search_leg_count{0};
    int lost_candidate_count{0};
    int expected_touchdown_count{0};
    bool all_support_confirmed{false};
    bool sparse_support{false};
    bool degraded_support{false};
};

SupportAssessment assessSupportState(const RobotState& est,
                                     const MotionIntent& intent,
                                     const GaitState& gait,
                                     const HexapodGeometry& geometry);

double supportPolygonClearanceExcludingLeg(const SupportAssessment& support,
                                           int excluded_leg,
                                           double margin_required_m,
                                           double inset_m);
