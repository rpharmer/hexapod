#include <cassert>

#include "../src/minphys3d/solver/block4_solver.hpp"

int main() {
    using namespace minphys3d;
    using namespace minphys3d::solver_internal;

    Block4SolveInput input{};
    input.invIA = Mat3::Identity();
    input.invIB = Mat3::Identity();
    input.invMassSum = 2.0f;
    input.blockDiagonalMinimum = 1e-6f;
    input.blockConditionEstimateMax = 0.0f;
    input.face4ConditionEstimateMax = 0.0f;
    input.face4Iterations = 8;
    input.face4ProjectedGaussSeidelEpsilon = 1e-5f;
    input.face4MinSpreadSq = 1e-6f;
    input.face4MinArea = 1e-6f;

    for (int i = 0; i < 4; ++i) {
        input.contacts[i].point = Vec3{0.0f, 0.0f, 0.0f};
        input.contacts[i].normal = Normalize(Vec3{0.0f, 1.0f, 0.0f});
        input.contacts[i].ra = input.contacts[i].point;
        input.contacts[i].rb = input.contacts[i].point;
        input.contacts[i].rhs = 1.0f;
    }

    const Block4SolveResult qualityFail = SolveBlock4ProjectedGaussSeidel(input);
    assert(!qualityFail.success);
    assert(qualityFail.fallbackReason == BlockSolveFallbackCode::QualityGate);

    input.contacts[0].point = Vec3{-1.0f, 0.0f, -1.0f};
    input.contacts[1].point = Vec3{ 1.0f, 0.0f, -1.0f};
    input.contacts[2].point = Vec3{ 1.0f, 0.0f,  1.0f};
    input.contacts[3].point = Vec3{-1.0f, 0.0f,  1.0f};
    for (int i = 0; i < 4; ++i) {
        input.contacts[i].ra = input.contacts[i].point;
        input.contacts[i].rb = input.contacts[i].point;
    }

    input.blockConditionEstimateMax = 1.0f;
    input.face4ConditionEstimateMax = 1.0f;
    const Block4SolveResult conditioned = SolveBlock4ProjectedGaussSeidel(input);
    assert(!conditioned.success);
    assert(conditioned.fallbackReason != BlockSolveFallbackCode::None);

    return 0;
}
