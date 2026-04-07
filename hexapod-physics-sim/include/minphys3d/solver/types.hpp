#pragma once

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

constexpr float kSleepLinearThreshold = 0.05f;
constexpr float kSleepAngularThreshold = 0.05f;
constexpr int kSleepFramesThreshold = 120;
constexpr float kMaxSubstepDistanceFactor = 0.5f;

struct ContactSolverConfig {
    float bounceVelocityThreshold = 0.0f;
    float restitutionSuppressionSpeed = 0.0f;
    float restitutionVelocityCutoff = 0.0f;
    float staticFrictionSpeedThreshold = 0.0f;
    float staticToDynamicTransitionSpeed = 0.0f;
    float penetrationSlop = 0.01f;
    float penetrationBiasFactor = 0.0f;
    float positionalCorrectionPercent = 0.8f;
    bool useSplitImpulse = false;
    float splitImpulseCorrectionFactor = 0.8f;
    float blockDeterminantEpsilon = 1e-8f;
    float blockDiagonalMinimum = 1e-6f;
    float blockConditionEstimateMax = 0.0f;
    bool useBlockSolver = true;
    std::uint32_t blockManifoldTypeMask = (1u << 7u) | (1u << 9u);
    std::array<std::uint8_t, 256> blockMinPersistenceByType{};
    bool useFace4PointNormalBlock = false;
    std::uint16_t face4MinPersistenceAge = 1;
    float face4MinSpreadSq = 1e-6f;
    float face4MinArea = 1e-6f;
    float face4ConditionEstimateMax = 0.0f;
    std::uint8_t face4Iterations = 8;
    float face4ProjectedGaussSeidelEpsilon = 1e-5f;
};

struct Contact {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    Vec3 point{};
    float penetration = 0.0f;
    float normalImpulseSum = 0.0f;
    float tangentImpulseSum = 0.0f;
    std::uint64_t key = 0;
    std::uint64_t featureKey = 0;
    std::uint16_t persistenceAge = 0;
};

struct Manifold {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    std::uint8_t manifoldType = 0;
    std::vector<Contact> contacts;
    std::array<float, 2> blockNormalImpulseSum{0.0f, 0.0f};
    std::array<std::uint64_t, 2> blockContactKeys{0u, 0u};
    std::array<bool, 2> blockSlotValid{false, false};
    std::array<int, 2> selectedBlockContactIndices{-1, -1};
    std::array<std::uint64_t, 2> selectedBlockContactKeys{0u, 0u};
    Vec3 t0{};
    Vec3 t1{};
    bool tangentBasisValid = false;
    std::array<float, 2> manifoldTangentImpulseSum{0.0f, 0.0f};
    bool manifoldTangentImpulseValid = false;
    std::unordered_map<std::uint64_t, std::array<float, 2>> cachedImpulseByContactKey{};
    bool selectedBlockPairPersistent = false;
    bool selectedBlockPairQualityPass = false;
    bool lowQuality = false;
    bool blockSolveEligible = false;
    bool usedBlockSolve = false;
#ifndef NDEBUG
    struct BlockSolveDebugCounters {
        std::array<float, 2> selectedPreNormalImpulses{0.0f, 0.0f};
        std::array<float, 2> selectedPostNormalImpulses{0.0f, 0.0f};
        float selectedPairPenetrationStep = 0.0f;
        std::uint32_t blockSolveUsedCount = 0;
        std::uint32_t scalarFallbackIneligibleCount = 0;
        std::uint32_t scalarFallbackPersistenceGateCount = 0;
        std::uint32_t scalarFallbackInvalidNormalCount = 0;
        std::uint32_t scalarFallbackNormalMismatchCount = 0;
        std::uint32_t scalarFallbackMissingSlotsCount = 0;
        std::uint32_t scalarFallbackDegenerateMassCount = 0;
        std::uint32_t scalarFallbackConditionEstimateCount = 0;
        std::uint32_t scalarFallbackLcpFailureCount = 0;
        std::uint32_t scalarFallbackNonFiniteCount = 0;
    } blockSolveDebug{};
#endif

    std::uint64_t pairKey() const {
        const std::uint32_t lo = std::min(a, b);
        const std::uint32_t hi = std::max(a, b);
        return (static_cast<std::uint64_t>(lo) << 32) | hi;
    }
};

struct Island {
    std::vector<std::uint32_t> bodies;
    std::vector<std::size_t> manifolds;
    std::vector<std::size_t> joints;
    std::vector<std::size_t> hinges;
};

} // namespace minphys3d
