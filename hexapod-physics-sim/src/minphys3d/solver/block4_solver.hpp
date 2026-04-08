#pragma once

#include <array>

#include "block2_solver.hpp"
#include "minphys3d/math/mat3.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d::solver_internal {

struct Block4ContactInput {
    Vec3 point{};
    Vec3 normal{};
    Vec3 ra{};
    Vec3 rb{};
    float oldImpulse = 0.0f;
    float rhs = 0.0f;
};

struct Block4SolveInput {
    Mat3 invIA{};
    Mat3 invIB{};
    float invMassSum = 0.0f;
    float blockDiagonalMinimum = 0.0f;
    float blockConditionEstimateMax = 0.0f;
    float face4ConditionEstimateMax = 0.0f;
    int face4Iterations = 1;
    float face4ProjectedGaussSeidelEpsilon = 1e-5f;
    float face4MinSpreadSq = 0.0f;
    float face4MinArea = 0.0f;
    std::array<Block4ContactInput, 4> contacts{};
};

struct Block4SolveResult {
    bool success = false;
    BlockSolveFallbackCode fallbackReason = BlockSolveFallbackCode::LcpFailure;
    std::array<float, 4> impulseDeltas{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> solvedImpulses{0.0f, 0.0f, 0.0f, 0.0f};
    float conditionEstimate = 0.0f;
};

Block4SolveResult SolveBlock4ProjectedGaussSeidel(const Block4SolveInput& input);

} // namespace minphys3d::solver_internal
