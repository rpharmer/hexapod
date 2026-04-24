#include "minphys3d/core/constraint_solver.hpp"
#include "minphys3d/solver/island_ordering.hpp"

namespace minphys3d::core_internal {

void ConstraintSolver::SolveIslands(const ConstraintSolverContext& context) const {
    ContactSolver solver;

    for (std::size_t islandIdx = 0; islandIdx < context.islands.size(); ++islandIdx) {
        const Island& island = context.islands[islandIdx];

        // Use pre-computed ordering when the caller has already done the work for this substep.
        // Fall back to computing on-demand (e.g. tests that call SolveIslands directly).
        IslandOrderResult onDemandOrder;
        const IslandOrderResult* orderPtr = nullptr;
        if (context.precomputedIslandOrders && islandIdx < context.precomputedIslandOrders->size()) {
            orderPtr = &(*context.precomputedIslandOrders)[islandIdx];
        } else {
            onDemandOrder = solver_internal::ComputeIslandOrder(
                island, context.bodies, context.manifolds, context.contactSolverConfig);
            orderPtr = &onDemandOrder;
        }
        const IslandOrderResult& islandOrder = *orderPtr;

        const std::vector<std::size_t>& manifoldOrder = islandOrder.manifoldOrder;
        const IslandSolveOrdering ordering = islandOrder.orderingUsed;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        if (context.solverTelemetry) {
            if (islandOrder.supportDepthApplied) {
                ++context.solverTelemetry->supportDepthOrderApplied;
            } else {
                ++context.solverTelemetry->supportDepthOrderBypassed;
            }
        }
#endif

        auto solveOrder = [&](const std::vector<std::size_t>& order) {
            for (std::size_t mi : order) {
                context.solveContactsInManifold(solver, context.contactContext, context.manifolds[mi]);
            }
        };
        solveOrder(manifoldOrder);
        if (ordering == IslandSolveOrdering::ShockPropagation && manifoldOrder.size() > 1) {
            for (auto it = manifoldOrder.rbegin(); it != manifoldOrder.rend(); ++it) {
                context.solveContactsInManifold(solver, context.contactContext, context.manifolds[*it]);
            }
        }

        for (std::size_t ji : island.joints) {
            context.solveDistanceJoint(context.joints[ji]);
        }
        for (std::size_t hi : island.hinges) {
            context.solveHingeJoint(context.hingeJoints[hi]);
        }
        for (std::size_t bi : island.ballSockets) {
            context.solveBallSocketJoint(context.ballSocketJoints[bi]);
        }
        for (std::size_t fi : island.fixeds) {
            context.solveFixedJoint(context.fixedJoints[fi]);
        }
        for (std::size_t pi : island.prismatics) {
            context.solvePrismaticJoint(context.prismaticJoints[pi]);
        }
        for (std::size_t si : island.servos) {
            context.solveServoJoint(context.servoJoints[si]);
        }
    }
}

} // namespace minphys3d::core_internal
