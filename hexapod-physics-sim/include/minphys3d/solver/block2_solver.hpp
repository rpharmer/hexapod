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
    float oldImpulse = 0.0f;
    float rhs = 0.0f;
};

struct Block2SolveInput {
    Mat3 invIA{};
    Mat3 invIB{};
    float invMassSum = 0.0f;
    float blockDiagonalMinimum = 0.0f;
    float blockDeterminantEpsilon = 0.0f;
    float blockConditionEstimateMax = 0.0f;
    std::array<Block2ContactInput, 2> contacts{};
};

struct Block2SolveResult {
    bool success = false;
    BlockSolveFallbackCode fallbackReason = BlockSolveFallbackCode::LcpFailure;
    std::array<float, 2> impulseDeltas{0.0f, 0.0f};
    std::array<float, 2> solvedImpulses{0.0f, 0.0f};
    float conditionEstimate = 0.0f;
};

Block2SolveResult SolveBlock2NormalLcp(const Block2SolveInput& input);

} // namespace minphys3d::solver_internal
