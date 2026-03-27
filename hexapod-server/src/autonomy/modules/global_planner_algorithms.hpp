#pragma once

#include "autonomy/common_types.hpp"
#include "autonomy/navigation_types.hpp"

#include <vector>

namespace autonomy::global_planner::algorithms {

std::vector<Waypoint> buildConstraintAwareRoute(const Waypoint& target,
                                                double traversability_cost,
                                                double traversability_risk);

} // namespace autonomy::global_planner::algorithms
