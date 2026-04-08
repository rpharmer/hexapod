#include <cassert>
#include <vector>

#include "../src/minphys3d/solver/island_ordering.hpp"

int main() {
    using namespace minphys3d;
    using namespace minphys3d::solver_internal;

    std::vector<Body> bodies(3);
    bodies[0].invMass = 0.0f;
    bodies[1].invMass = 1.0f;
    bodies[2].invMass = 1.0f;

    std::vector<Manifold> manifolds(2);
    manifolds[0].a = 1;
    manifolds[0].b = 2;
    manifolds[0].manifoldType = 5;
    manifolds[1].a = 0;
    manifolds[1].b = 1;
    manifolds[1].manifoldType = 1;

    Island island{};
    island.bodies = {0, 1, 2};
    island.manifolds = {0, 1};

    ContactSolverConfig cfg{};
    cfg.enableDeterministicOrdering = true;
    cfg.enableSupportDepthOrdering = true;
    cfg.islandSolveOrdering = IslandSolveOrdering::Insertion;

    const IslandOrderResult first = ComputeIslandOrder(island, bodies, manifolds, cfg);
    const IslandOrderResult second = ComputeIslandOrder(island, bodies, manifolds, cfg);
    assert(first.manifoldOrder == second.manifoldOrder);
    assert(first.supportDepthApplied);

    return 0;
}
