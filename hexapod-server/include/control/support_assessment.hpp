#pragma once

#include "types.hpp"

#include <array>

struct SupportPoint2 {
    double x{0.0};
    double y{0.0};
};

enum class SupportPointSource : std::uint8_t {
    Nominal = 0,
    JointFk = 1,
    AnchorFallback = 2,
};

struct SupportAssessment {
    std::array<bool, kNumLegs> effective_support{};
    /** Active support point set used by control. */
    std::array<SupportPoint2, kNumLegs> foot_xy_body_m{};
    std::array<SupportPoint2, kNumLegs> nominal_foot_xy_body_m{};
    std::array<SupportPoint2, kNumLegs> actual_foot_xy_body_m{};
    std::array<SupportPointSource, kNumLegs> support_point_source{};
    SupportPoint2 projected_com_xy_m{};
    double static_margin_m{0.0};
    double nominal_static_margin_m{0.0};
    double actual_static_margin_m{0.0};
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
