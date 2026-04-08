#include "constraint_solver.hpp"
#include "../solver/island_ordering.hpp"

namespace minphys3d::core_internal {

void ConstraintSolver::SolveIslands(const ConstraintSolverContext& context) const {
    ContactSolver solver;

    for (const Island& island : context.islands) {
        const solver_internal::IslandOrderResult islandOrder = solver_internal::ComputeIslandOrder(
            island,
            context.bodies,
            context.manifolds,
            context.contactSolverConfig);
        const std::vector<std::size_t>& manifoldOrder = islandOrder.manifoldOrder;
        const IslandSolveOrdering ordering = islandOrder.orderingUsed;
#ifndef NDEBUG
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
            std::vector<std::size_t> reverseOrder = manifoldOrder;
            std::reverse(reverseOrder.begin(), reverseOrder.end());
            solveOrder(reverseOrder);
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
