#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

enum class FrictionBudgetNormalSupportSource : std::uint8_t {
    SelectedBlockPairOnly = 0,
    AllManifoldContacts = 1,
    BlendedSelectedPairAndManifold = 2,
};

enum class IslandSolveOrdering : std::uint8_t {
    Insertion = 0,
    SupportDepth = 1,
    ShockPropagation = 2,
};

constexpr float kSleepLinearThreshold = 0.05f;
constexpr float kSleepAngularThreshold = 0.05f;
constexpr int kSleepFramesThreshold = 120;
constexpr float kMaxSubstepDistanceFactor = 0.5f;

struct ToiSolverConfig {
    int max_iterations = 8;
    float min_time_step = 1e-6f;
};

struct Block4MatrixConfig {
    float symmetry_tolerance = 2e-3f;
};

struct OrderingConfig {
    std::uint8_t support_depth_relaxation_passes = 4;
};

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
    bool enableRelaxationPass = false;
    std::uint8_t relaxationIterations = 0;
    float splitImpulseCorrectionFactor = 0.8f;
    float highMassRatioSplitImpulseBoost = 0.35f;
    float highMassRatioBiasBoost = 0.30f;
    float highMassRatioThreshold = 8.0f;
    float penetrationBiasMaxSpeed = 3.0f;
    bool enableSupportDepthOrdering = false;
    bool enableDeterministicOrdering = true;
    IslandSolveOrdering islandSolveOrdering = IslandSolveOrdering::Insertion;
    std::uint16_t manifoldAnchorReuseMinAge = 2;
    float manifoldAnchorReuseMaxSeparationDelta = 0.03f;
    std::uint32_t manifoldAnchorReuseMaxFallbackPerStep = 64;
    std::uint16_t softContactMinAge = 4;
    float softContactMaxNormalSpeed = 0.2f;
    float softContactBiasRate = 0.0f;
    float softContactCompliance = 0.0f;
    float blockDeterminantEpsilon = 1e-8f;
    float blockDiagonalMinimum = 1e-6f;
    float blockConditionEstimateMax = 0.0f;
    bool useBlockSolver = true;
    std::uint32_t blockManifoldTypeMask =
        (1u << 7u) | (1u << 9u) | (1u << 15u) | (1u << 19u) | (1u << 20u) | (1u << 21u);
    std::array<std::uint8_t, 256> blockMinPersistenceByType{};
    bool useFace4PointNormalBlock = false;
    std::uint16_t face4MinPersistenceAge = 1;
    float face4MinSpreadSq = 1e-6f;
    float face4MinArea = 1e-6f;
    float face4ConditionEstimateMax = 0.0f;
    std::uint8_t face4Iterations = 8;
    float face4ProjectedGaussSeidelEpsilon = 1e-5f;
    // Conservative default: keep 4-point projected normal solve off until friction coherence
    // telemetry indicates stable basis reuse/churn in target scenarios.
    bool face4RequireFrictionCoherence = true;
    std::uint16_t face4MinCoherentPersistenceAge = 2;
    std::uint16_t face4MinStableContactCount = 4;
    float face4MaxBasisChurnRatio = 0.5f;

    ToiSolverConfig toi{};
    Block4MatrixConfig block4{};
    OrderingConfig ordering{};

    // Staged rollout controls for manifold-level 2D friction budgeting.
    bool enableTwoAxisFrictionSolve = true;
    bool enableManifoldFrictionBudget = true;
    bool frictionBudgetUseRadialClamp = true;
    float manifoldFrictionBudgetScale = 1.0f;
    // Defaults preserve prior behavior: manifold budget normal support from all manifold contacts.
    FrictionBudgetNormalSupportSource frictionBudgetNormalSupportSource =
        FrictionBudgetNormalSupportSource::AllManifoldContacts;
    // Used only when support source is BlendedSelectedPairAndManifold.
    float frictionBudgetSelectedPairBlendWeight = 2.0f;
    bool enablePersistentStickConstraints = true;
    float stickVelocityThreshold = 0.08f;
    std::uint16_t stickMinPersistenceAge = 2;
    float stickImpulseRetention = 0.92f;
};

struct JointSolverConfig {
    bool useBlockSolver = false;
    float blockDeterminantEpsilon = 1e-8f;
    float blockDiagonalMinimum = 1e-6f;
    float blockConditionEstimateMax = 0.0f;
    float hingeAnchorBiasFactor = 0.2f;
    float hingeAnchorDampingFactor = 0.1f;
    /// Post-integration servo anchor + hinge-angle stabilization passes (0 disables that block).
    std::uint8_t servoPositionPasses = 4;
};

struct BroadphaseConfig {
    float baseFatAabbMargin = 0.1f;
    float linearVelocityMarginScale = 1.0f;
    float angularVelocityMarginScale = 0.5f;
    float minSweptMargin = 0.05f;
    float maxSweptMargin = 1.0f;

    float partialRebuildAreaRatioThreshold = 2.5f;
    float partialRebuildAvgDepthThreshold = 24.0f;
    float partialRebuildMovedProxyRatioThreshold = 0.20f;

    float fullRebuildAreaRatioThreshold = 4.0f;
    float fullRebuildMaxDepthThreshold = 96.0f;
    float fullRebuildMovedProxyRatioThreshold = 0.50f;
    float fullRebuildQueryNodeVisitsPerProxyThreshold = 256.0f;
    bool enableMovedSetOnlyUpdates = true;
    bool enablePairCacheReuseForQuasiStatic = true;

    std::uint32_t minStepsBetweenRebuilds = 8;
};

struct BroadphaseMetrics {
    float areaRatio = 0.0f;
    float maxDepth = 0.0f;
    float avgDepth = 0.0f;
    float movedProxyRatio = 0.0f;
    float queryNodeVisitsPerProxy = 0.0f;
    float pairGenerationMs = 0.0f;
    float pairCacheHitRate = 0.0f;
    std::uint32_t queryNodeVisits = 0;
    std::uint32_t pairCacheQueries = 0;
    std::uint32_t pairCacheHits = 0;
    std::uint32_t validProxyCount = 0;
    bool usedMovedSetOnlyUpdate = false;
    bool partialRebuildTriggered = false;
    bool fullRebuildTriggered = false;
};

struct Contact {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    Vec3 point{};
    float penetration = 0.0f;
    float normalImpulseSum = 0.0f;
    // Legacy scalar tangent accumulator retained for compatibility/migration.
    float tangentImpulseSum = 0.0f;
    float tangentImpulseSum0 = 0.0f;
    float tangentImpulseSum1 = 0.0f;
    std::uint8_t manifoldType = 0;
    std::uint64_t key = 0;
    std::uint64_t featureKey = 0;
    std::uint16_t persistenceAge = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    float referenceSeparation = 0.0f;
    bool anchorsValid = false;
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
    bool stickConstraintActive = false;
    std::uint16_t stickConstraintAge = 0;
    // Cached per-contact impulse state: [normal, tangent0, tangent1].
    std::unordered_map<std::uint64_t, std::array<float, 3>> cachedImpulseByContactKey{};
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
    std::vector<std::size_t> ballSockets;
    std::vector<std::size_t> fixeds;
    std::vector<std::size_t> prismatics;
    std::vector<std::size_t> servos;
};


inline bool ValidateContactSolverConfig(const ContactSolverConfig& config) {
    constexpr float kMinSaneToiStep = 1e-9f;
    if (config.toi.max_iterations < 1) {
        return false;
    }
    if (!std::isfinite(config.toi.min_time_step) || config.toi.min_time_step < kMinSaneToiStep) {
        return false;
    }
    if (!std::isfinite(config.block4.symmetry_tolerance) || config.block4.symmetry_tolerance < 0.0f) {
        return false;
    }
    return true;
}

inline ContactSolverConfig SanitizeContactSolverConfig(const ContactSolverConfig& config) {
    ContactSolverConfig sanitized = config;
    const ContactSolverConfig defaults{};

    if (sanitized.toi.max_iterations < 1) {
        sanitized.toi.max_iterations = defaults.toi.max_iterations;
    }
    if (!std::isfinite(sanitized.toi.min_time_step) || sanitized.toi.min_time_step < 1e-9f) {
        sanitized.toi.min_time_step = defaults.toi.min_time_step;
    }
    if (!std::isfinite(sanitized.block4.symmetry_tolerance) || sanitized.block4.symmetry_tolerance < 0.0f) {
        sanitized.block4.symmetry_tolerance = defaults.block4.symmetry_tolerance;
    }

    return sanitized;
}

} // namespace minphys3d
