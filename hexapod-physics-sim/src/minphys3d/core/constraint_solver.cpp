#include "constraint_solver.hpp"

namespace minphys3d::core_internal {

void ConstraintSolver::SolveIslands(const ConstraintSolverContext& context) const {
    ContactSolver solver;

    for (const Island& island : context.islands) {
        std::vector<std::size_t> manifoldOrder = island.manifolds;
        if (context.contactSolverConfig.enableDeterministicOrdering) {
            std::stable_sort(manifoldOrder.begin(), manifoldOrder.end(), [&](std::size_t lhs, std::size_t rhs) {
                const Manifold& ml = context.manifolds[lhs];
                const Manifold& mr = context.manifolds[rhs];
                if (ml.pairKey() != mr.pairKey()) {
                    return ml.pairKey() < mr.pairKey();
                }
                if (ml.manifoldType != mr.manifoldType) {
                    return ml.manifoldType < mr.manifoldType;
                }
                return lhs < rhs;
            });
        }

        IslandSolveOrdering ordering = context.contactSolverConfig.islandSolveOrdering;
        if (ordering == IslandSolveOrdering::Insertion && context.contactSolverConfig.enableSupportDepthOrdering) {
            ordering = IslandSolveOrdering::SupportDepth;
        }
        if (ordering == IslandSolveOrdering::SupportDepth || ordering == IslandSolveOrdering::ShockPropagation) {
            std::unordered_map<std::uint32_t, std::uint32_t> supportDepth;
            for (std::uint32_t id : island.bodies) {
                supportDepth[id] = (context.bodies[id].invMass == 0.0f) ? 0u : 1u;
            }
            for (int iter = 0; iter < 4; ++iter) {
                for (std::size_t mi : island.manifolds) {
                    const Manifold& m = context.manifolds[mi];
                    const std::uint32_t da = supportDepth[m.a];
                    const std::uint32_t db = supportDepth[m.b];
                    if (context.bodies[m.a].invMass == 0.0f && context.bodies[m.b].invMass > 0.0f) {
                        supportDepth[m.b] = std::max(supportDepth[m.b], da + 1);
                    } else if (context.bodies[m.b].invMass == 0.0f && context.bodies[m.a].invMass > 0.0f) {
                        supportDepth[m.a] = std::max(supportDepth[m.a], db + 1);
                    }
                }
            }
            std::sort(manifoldOrder.begin(), manifoldOrder.end(), [&](std::size_t lhs, std::size_t rhs) {
                const Manifold& ml = context.manifolds[lhs];
                const Manifold& mr = context.manifolds[rhs];
                const std::uint32_t dl = std::min(supportDepth[ml.a], supportDepth[ml.b]);
                const std::uint32_t dr = std::min(supportDepth[mr.a], supportDepth[mr.b]);
                return dl < dr;
            });
#ifndef NDEBUG
            if (context.solverTelemetry) {
                ++context.solverTelemetry->supportDepthOrderApplied;
            }
#endif
        } else {
#ifndef NDEBUG
            if (context.solverTelemetry) {
                ++context.solverTelemetry->supportDepthOrderBypassed;
            }
#endif
        }

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
