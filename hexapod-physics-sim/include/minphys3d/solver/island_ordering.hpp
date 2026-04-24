#pragma once

#include <cstddef>
#include <vector>

#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/solver/types.hpp"

namespace minphys3d::solver_internal {

IslandOrderResult ComputeIslandOrder(const Island& island,
                                     const std::vector<Body>& bodies,
                                     const std::vector<Manifold>& manifolds,
                                     const ContactSolverConfig& config);

} // namespace minphys3d::solver_internal
