#pragma once

#include <array>

#include "minphys3d/math/mat3.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d::solver_internal {

enum class BlockSolveFallbackCode {
    None,
    Ineligible,
    TypePolicy,
    QualityGate,
    PersistenceGate,
    InvalidManifoldNormal,
    ContactNormalMismatch,
    MissingBlockSlots,
    DegenerateMassMatrix,
    ConditionEstimateExceeded,
    LcpFailure,
    NonFiniteResult,
};

struct Block2ContactInput {
    Vec3 normal{};
    Vec3 ra{};
    Vec3 rb{};
    Real oldImpulse = 0.0;
    Real rhs = 0.0;
};

struct Block2SolveInput {
    Mat3 invIA{};
    Mat3 invIB{};
    Real invMassSum = 0.0;
    Real blockDiagonalMinimum = 0.0;
    Real blockDeterminantEpsilon = 0.0;
    Real blockConditionEstimateMax = 0.0;
    std::array<Block2ContactInput, 2> contacts{};
};

struct Block2SolveResult {
    bool success = false;
    BlockSolveFallbackCode fallbackReason = BlockSolveFallbackCode::LcpFailure;
    std::array<Real, 2> impulseDeltas{0.0, 0.0};
    std::array<Real, 2> solvedImpulses{0.0, 0.0};
    Real conditionEstimate = 0.0;
};

Block2SolveResult SolveBlock2NormalLcp(const Block2SolveInput& input);

} // namespace minphys3d::solver_internal
