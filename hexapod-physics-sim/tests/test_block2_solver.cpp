#include <cassert>
#include <cmath>

#include "minphys3d/solver/block2_solver.hpp"

int main() {
    using namespace minphys3d;
    using namespace minphys3d::solver_internal;

    Block2SolveInput good{};
    good.invIA = Mat3::Identity();
    good.invIB = Mat3::Identity();
    good.invMassSum = 2.0f;
    good.blockDiagonalMinimum = 1e-6f;
    good.blockDeterminantEpsilon = 1e-8f;
    good.blockConditionEstimateMax = 100.0f;
    good.contacts[0] = {Normalize(Vec3{1.0f, 0.0f, 0.0f}), Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 0.0f, -1.0f}, 0.0f, 1.0f};
    good.contacts[1] = {Normalize(Vec3{0.0f, 1.0f, 0.0f}), Vec3{0.0f, 0.0f, 1.0f}, Vec3{0.0f, 0.0f, -1.0f}, 0.0f, 1.0f};
    const Block2SolveResult goodResult = SolveBlock2NormalLcp(good);
    assert(goodResult.success);
    assert(std::isfinite(goodResult.conditionEstimate));
    assert(goodResult.solvedImpulses[0] >= 0.0f);
    assert(goodResult.solvedImpulses[1] >= 0.0f);

    Block2SolveInput conditioned = good;
    conditioned.blockConditionEstimateMax = 2.0f;
    conditioned.blockDeterminantEpsilon = 0.0f;
    conditioned.contacts[1].normal = Normalize(Vec3{1.0f, 1e-3f, 0.0f});
    const Block2SolveResult conditionedResult = SolveBlock2NormalLcp(conditioned);
    assert(!conditionedResult.success);
    assert(conditionedResult.fallbackReason == BlockSolveFallbackCode::ConditionEstimateExceeded);

    return 0;
}
