#include "island_ordering.hpp"

#include <algorithm>
#include <unordered_map>

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
        std::unordered_map<std::uint32_t, std::uint32_t> supportDepth;
        for (std::uint32_t id : island.bodies) {
            supportDepth[id] = (bodies[id].invMass == 0.0f) ? 0u : 1u;
        }
        for (int iter = 0; iter < 4; ++iter) {
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
