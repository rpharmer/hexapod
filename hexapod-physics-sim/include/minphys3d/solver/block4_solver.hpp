#pragma once

#include <array>

#include "minphys3d/solver/block2_solver.hpp"
#include "minphys3d/math/mat3.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d::solver_internal {

struct Block4ContactInput {
    Vec3 point{};
    Vec3 normal{};
    Vec3 ra{};
    Vec3 rb{};
    Real oldImpulse = 0.0;
    Real rhs = 0.0;
};

struct Block4SolveInput {
    Mat3 invIA{};
    Mat3 invIB{};
    Real invMassSum = 0.0;
    Real blockDiagonalMinimum = 0.0;
    Real blockConditionEstimateMax = 0.0;
    Real face4ConditionEstimateMax = 0.0;
    int face4Iterations = 1;
    Real face4ProjectedGaussSeidelEpsilon = 1e-5;
    Real face4MinSpreadSq = 0.0;
    Real face4MinArea = 0.0;
    Real symmetryTolerance = 2e-3;
    std::array<Block4ContactInput, 4> contacts{};
};

struct Block4SolveResult {
    bool success = false;
    BlockSolveFallbackCode fallbackReason = BlockSolveFallbackCode::LcpFailure;
    std::array<Real, 4> impulseDeltas{0.0, 0.0, 0.0, 0.0};
    std::array<Real, 4> solvedImpulses{0.0, 0.0, 0.0, 0.0};
    Real conditionEstimate = 0.0;
};

Block4SolveResult SolveBlock4ProjectedGaussSeidel(const Block4SolveInput& input);

} // namespace minphys3d::solver_internal
