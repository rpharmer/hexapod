#pragma once

#include <functional>
#include <vector>

#include "minphys3d/core/subsystems.hpp"
#include "minphys3d/core/world.hpp"

namespace minphys3d::core_internal {

struct ConstraintSolverContext {
    std::vector<Body>& bodies;
    std::vector<Manifold>& manifolds;
    const std::vector<Island>& islands;
    std::vector<DistanceJoint>& joints;
    std::vector<HingeJoint>& hingeJoints;
    std::vector<BallSocketJoint>& ballSocketJoints;
    std::vector<FixedJoint>& fixedJoints;
    std::vector<PrismaticJoint>& prismaticJoints;
    std::vector<ServoJoint>& servoJoints;
    const ContactSolverContext& contactContext;
    const ContactSolverConfig& contactSolverConfig;
    // When non-null, island ordering is read from this cache instead of being recomputed each call.
    const std::vector<IslandOrderResult>* precomputedIslandOrders = nullptr;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    World::SolverTelemetry* solverTelemetry = nullptr;
#endif

    std::function<void(ContactSolver&, const ContactSolverContext&, Manifold&)> solveContactsInManifold;
    std::function<void(DistanceJoint&)> solveDistanceJoint;
    std::function<void(HingeJoint&)> solveHingeJoint;
    std::function<void(BallSocketJoint&)> solveBallSocketJoint;
    std::function<void(FixedJoint&)> solveFixedJoint;
    std::function<void(PrismaticJoint&)> solvePrismaticJoint;
    std::function<void(ServoJoint&)> solveServoJoint;
};

class ConstraintSolver {
public:
    void SolveIslands(const ConstraintSolverContext& context) const;
};

} // namespace minphys3d::core_internal
