#include "minphys3d/solver/island_ordering.hpp"

#include <algorithm>

namespace minphys3d::solver_internal {

IslandOrderResult ComputeIslandOrder(const Island& island,
                                     const std::vector<Body>& bodies,
                                     const std::vector<Manifold>& manifolds,
                                     const ContactSolverConfig& config) {
    IslandOrderResult result{};
    result.manifoldOrder = island.manifolds;

    if (config.enableDeterministicOrdering) {
        std::stable_sort(result.manifoldOrder.begin(), result.manifoldOrder.end(), [&](std::size_t lhs, std::size_t rhs) {
            const Manifold& ml = manifolds[lhs];
            const Manifold& mr = manifolds[rhs];
            if (ml.pairKey() != mr.pairKey()) {
                return ml.pairKey() < mr.pairKey();
            }
            if (ml.manifoldType != mr.manifoldType) {
                return ml.manifoldType < mr.manifoldType;
            }
            return lhs < rhs;
        });
    }

    result.orderingUsed = config.islandSolveOrdering;
    if (result.orderingUsed == IslandSolveOrdering::Insertion && config.enableSupportDepthOrdering) {
        result.orderingUsed = IslandSolveOrdering::SupportDepth;
    }

    if (result.orderingUsed == IslandSolveOrdering::SupportDepth || result.orderingUsed == IslandSolveOrdering::ShockPropagation) {
        // Use a flat vector indexed by body ID — O(1) lookup, no heap allocation from hash map.
        // Body IDs are contiguous indices; non-island bodies remain at 0 (static depth), which is
        // harmless because the sort only touches manifolds belonging to this island.
        std::vector<std::uint32_t> supportDepth(bodies.size(), 0u);
        for (std::uint32_t id : island.bodies) {
            supportDepth[id] = (bodies[id].invMass == 0.0f) ? 0u : 1u;
        }
        for (std::uint32_t iter = 0; iter < config.ordering.support_depth_relaxation_passes; ++iter) {
            for (std::size_t mi : island.manifolds) {
                const Manifold& m = manifolds[mi];
                const std::uint32_t da = supportDepth[m.a];
                const std::uint32_t db = supportDepth[m.b];
                if (bodies[m.a].invMass == 0.0f && bodies[m.b].invMass > 0.0f) {
                    supportDepth[m.b] = std::max(supportDepth[m.b], da + 1);
                } else if (bodies[m.b].invMass == 0.0f && bodies[m.a].invMass > 0.0f) {
                    supportDepth[m.a] = std::max(supportDepth[m.a], db + 1);
                }
            }
        }
        std::sort(result.manifoldOrder.begin(), result.manifoldOrder.end(), [&](std::size_t lhs, std::size_t rhs) {
            const Manifold& ml = manifolds[lhs];
            const Manifold& mr = manifolds[rhs];
            return std::min(supportDepth[ml.a], supportDepth[ml.b]) < std::min(supportDepth[mr.a], supportDepth[mr.b]);
        });
        result.supportDepthApplied = true;
    }

    return result;
}

} // namespace minphys3d::solver_internal
