#pragma once

#include "types.hpp"

struct StabilityAssessment {
    bool has_support_polygon{false};
    bool com_inside_support_polygon{false};
    int support_contact_count{0};
    double stability_margin_m{0.0};
};

StabilityAssessment assessStability(const RobotState& state);
