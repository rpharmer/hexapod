#include <cassert>

#include "minphys3d/solver/block4_solver.hpp"

int main() {
    using namespace minphys3d;
    using namespace minphys3d::solver_internal;

    Block4SolveInput input{};
    input.invIA = Mat3::Identity();
    input.invIB = Mat3::Identity();
    input.invMassSum = 2.0;
    input.blockDiagonalMinimum = 1e-6;
    input.blockConditionEstimateMax = 0.0;
    input.face4ConditionEstimateMax = 0.0;
    input.face4Iterations = 8;
    input.face4ProjectedGaussSeidelEpsilon = 1e-5;
    input.face4MinSpreadSq = 1e-6;
    input.face4MinArea = 1e-6;

    for (int i = 0; i < 4; ++i) {
        input.contacts[i].point = Vec3{0.0, 0.0, 0.0};
        input.contacts[i].normal = Normalize(Vec3{0.0, 1.0, 0.0});
        input.contacts[i].ra = input.contacts[i].point;
        input.contacts[i].rb = input.contacts[i].point;
        input.contacts[i].rhs = 1.0;
    }

    const Block4SolveResult qualityFail = SolveBlock4ProjectedGaussSeidel(input);
    assert(!qualityFail.success);
    assert(qualityFail.fallbackReason == BlockSolveFallbackCode::QualityGate);

    input.contacts[0].point = Vec3{-1.0, 0.0, -1.0};
    input.contacts[1].point = Vec3{ 1.0, 0.0, -1.0};
    input.contacts[2].point = Vec3{ 1.0, 0.0,  1.0};
    input.contacts[3].point = Vec3{-1.0, 0.0,  1.0};
    for (int i = 0; i < 4; ++i) {
        input.contacts[i].ra = input.contacts[i].point;
        input.contacts[i].rb = input.contacts[i].point;
    }

    input.blockConditionEstimateMax = 1.0;
    input.face4ConditionEstimateMax = 1.0;
    const Block4SolveResult conditioned = SolveBlock4ProjectedGaussSeidel(input);
    assert(!conditioned.success);
    assert(conditioned.fallbackReason != BlockSolveFallbackCode::None);

    Block4SolveInput legacyInput = input;
    legacyInput.symmetryTolerance = 2e-3;
    const Block4SolveResult defaultTolerance = SolveBlock4ProjectedGaussSeidel(input);
    const Block4SolveResult legacyTolerance = SolveBlock4ProjectedGaussSeidel(legacyInput);
    assert(defaultTolerance.success == legacyTolerance.success);
    assert(defaultTolerance.fallbackReason == legacyTolerance.fallbackReason);


    return 0;
}
