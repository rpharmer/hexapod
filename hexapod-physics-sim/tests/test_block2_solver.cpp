#include <cassert>
#include <cmath>

#include "minphys3d/solver/block2_solver.hpp"

int main() {
    using namespace minphys3d;
    using namespace minphys3d::solver_internal;

    Block2SolveInput good{};
    good.invIA = Mat3::Identity();
    good.invIB = Mat3::Identity();
    good.invMassSum = 2.0;
    good.blockDiagonalMinimum = 1e-6;
    good.blockDeterminantEpsilon = 1e-8;
    good.blockConditionEstimateMax = 100.0;
    good.contacts[0] = {Normalize(Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, -1.0}, 0.0, 1.0};
    good.contacts[1] = {Normalize(Vec3{0.0, 1.0, 0.0}), Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, -1.0}, 0.0, 1.0};
    const Block2SolveResult goodResult = SolveBlock2NormalLcp(good);
    assert(goodResult.success);
    assert(std::isfinite(goodResult.conditionEstimate));
    assert(goodResult.solvedImpulses[0] >= 0.0);
    assert(goodResult.solvedImpulses[1] >= 0.0);

    Block2SolveInput conditioned = good;
    conditioned.blockConditionEstimateMax = 2.0;
    conditioned.blockDeterminantEpsilon = 0.0;
    conditioned.contacts[1].normal = Normalize(Vec3{1.0, 1e-3, 0.0});
    const Block2SolveResult conditionedResult = SolveBlock2NormalLcp(conditioned);
    assert(!conditionedResult.success);
    assert(conditionedResult.fallbackReason == BlockSolveFallbackCode::ConditionEstimateExceeded);

    return 0;
}
