#include "minphys3d/core/constraint_solver.hpp"
#include "minphys3d/core/world_resource_monitoring.hpp"
#include "minphys3d/solver/island_ordering.hpp"

namespace minphys3d::core_internal {

void ConstraintSolver::SolveIslands(const ConstraintSolverContext& context) const {
    ContactSolver solver;
    using world_resource_monitoring::Section;
    using world_resource_monitoring::toIndex;

    for (std::size_t islandIdx = 0; islandIdx < context.islands.size(); ++islandIdx) {
        const Island& island = context.islands[islandIdx];

        IslandOrderResult onDemandOrder;
        const IslandOrderResult* orderPtr = nullptr;
        if (context.precomputedIslandOrders && islandIdx < context.precomputedIslandOrders->size()) {
            orderPtr = &(*context.precomputedIslandOrders)[islandIdx];
        } else {
            if (context.worldResourceProfiler) {
                const auto _scope = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsIslandOrder));
                onDemandOrder = solver_internal::ComputeIslandOrder(
                    island, context.bodies, context.manifolds, context.contactSolverConfig);
            } else {
                onDemandOrder = solver_internal::ComputeIslandOrder(
                    island, context.bodies, context.manifolds, context.contactSolverConfig);
            }
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

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsContactsForward));
                solveOrder(manifoldOrder);
            }
        } else {
            solveOrder(manifoldOrder);
        }

        if (ordering == IslandSolveOrdering::ShockPropagation && manifoldOrder.size() > 1) {
            if (context.worldResourceProfiler) {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsContactsShock));
                for (auto it = manifoldOrder.rbegin(); it != manifoldOrder.rend(); ++it) {
                    context.solveContactsInManifold(
                        solver, context.contactContext, context.manifolds[*it]);
                }
            } else {
                for (auto it = manifoldOrder.rbegin(); it != manifoldOrder.rend(); ++it) {
                    context.solveContactsInManifold(
                        solver, context.contactContext, context.manifolds[*it]);
                }
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsDistanceJoints));
                for (std::size_t ji : island.joints) {
                    context.solveDistanceJoint(context.joints[ji]);
                }
            }
        } else {
            for (std::size_t ji : island.joints) {
                context.solveDistanceJoint(context.joints[ji]);
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsHingeJoints));
                for (std::size_t hi : island.hinges) {
                    context.solveHingeJoint(context.hingeJoints[hi]);
                }
            }
        } else {
            for (std::size_t hi : island.hinges) {
                context.solveHingeJoint(context.hingeJoints[hi]);
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsBallSocketJoints));
                for (std::size_t bi : island.ballSockets) {
                    context.solveBallSocketJoint(context.ballSocketJoints[bi]);
                }
            }
        } else {
            for (std::size_t bi : island.ballSockets) {
                context.solveBallSocketJoint(context.ballSocketJoints[bi]);
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsFixedJoints));
                for (std::size_t fi : island.fixeds) {
                    context.solveFixedJoint(context.fixedJoints[fi]);
                }
            }
        } else {
            for (std::size_t fi : island.fixeds) {
                context.solveFixedJoint(context.fixedJoints[fi]);
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsPrismaticJoints));
                for (std::size_t pi : island.prismatics) {
                    context.solvePrismaticJoint(context.prismaticJoints[pi]);
                }
            }
        } else {
            for (std::size_t pi : island.prismatics) {
                context.solvePrismaticJoint(context.prismaticJoints[pi]);
            }
        }

        if (context.worldResourceProfiler) {
            {
                const auto _s = context.worldResourceProfiler->scope(
                    toIndex(Section::SolveIslandsServoJoints));
                for (std::size_t si : island.servos) {
                    context.solveServoJoint(context.servoJoints[si]);
                }
            }
        } else {
            for (std::size_t si : island.servos) {
                context.solveServoJoint(context.servoJoints[si]);
            }
        }
    }
}

} // namespace minphys3d::core_internal
