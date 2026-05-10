#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "minphys3d/math/vec3.hpp"

#ifndef MINPHYS3D_SOLVER_TELEMETRY_ENABLED
#if !defined(NDEBUG) || defined(MINPHYS3D_ENABLE_SOLVER_TELEMETRY)
#define MINPHYS3D_SOLVER_TELEMETRY_ENABLED 1
#else
#define MINPHYS3D_SOLVER_TELEMETRY_ENABLED 0
#endif
#endif

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

constexpr Real kSleepLinearThreshold = 0.05;
constexpr Real kSleepAngularThreshold = 0.05;
constexpr int kSleepFramesThreshold = 120;
constexpr Real kMaxSubstepDistanceFactor = 0.5;

struct ToiSolverConfig {
    int max_iterations = 8;
    Real min_time_step = 1e-6;
};

struct Block4MatrixConfig {
    Real symmetry_tolerance = 2e-3;
};

struct OrderingConfig {
    std::uint8_t support_depth_relaxation_passes = 4;
};

struct ContactSolverConfig {
    Real bounceVelocityThreshold = 0.0;
    Real restitutionSuppressionSpeed = 0.0;
    Real restitutionVelocityCutoff = 0.0;
    Real staticFrictionSpeedThreshold = 0.0;
    Real staticToDynamicTransitionSpeed = 0.0;
    Real penetrationSlop = 0.01;
    Real penetrationBiasFactor = 0.0;
    Real positionalCorrectionPercent = 0.8;
    bool useSplitImpulse = false;
    bool enableRelaxationPass = false;
    std::uint8_t relaxationIterations = 0;
    Real splitImpulseCorrectionFactor = 0.8;
    Real highMassRatioSplitImpulseBoost = 0.35;
    Real highMassRatioBiasBoost = 0.30;
    Real highMassRatioThreshold = 8.0;
    Real penetrationBiasMaxSpeed = 3.0;
    bool enableSupportDepthOrdering = false;
    bool enableDeterministicOrdering = true;
    IslandSolveOrdering islandSolveOrdering = IslandSolveOrdering::Insertion;
    std::uint16_t manifoldAnchorReuseMinAge = 2;
    Real manifoldAnchorReuseMaxSeparationDelta = 0.03;
    std::uint32_t manifoldAnchorReuseMaxFallbackPerStep = 64;
    std::uint16_t softContactMinAge = 4;
    Real softContactMaxNormalSpeed = 0.2;
    Real softContactBiasRate = 0.0;
    Real softContactCompliance = 0.0;
    Real blockDeterminantEpsilon = 1e-8;
    Real blockDiagonalMinimum = 1e-6;
    Real blockConditionEstimateMax = 0.0;
    bool useBlockSolver = true;
    std::uint32_t blockManifoldTypeMask =
        (1u << 7u) | (1u << 9u) | (1u << 15u) | (1u << 19u) | (1u << 20u) | (1u << 21u);
    std::array<std::uint8_t, 256> blockMinPersistenceByType{};
    bool useFace4PointNormalBlock = false;
    std::uint16_t face4MinPersistenceAge = 1;
    Real face4MinSpreadSq = 1e-6;
    Real face4MinArea = 1e-6;
    Real face4ConditionEstimateMax = 0.0;
    std::uint8_t face4Iterations = 8;
    Real face4ProjectedGaussSeidelEpsilon = 1e-5;
    // Conservative default: keep 4-point projected normal solve off until friction coherence
    // telemetry indicates stable basis reuse/churn in target scenarios.
    bool face4RequireFrictionCoherence = true;
    std::uint16_t face4MinCoherentPersistenceAge = 2;
    std::uint16_t face4MinStableContactCount = 4;
    Real face4MaxBasisChurnRatio = 0.5;

    ToiSolverConfig toi{};
    Block4MatrixConfig block4{};
    OrderingConfig ordering{};

    // Staged rollout controls for manifold-level 2D friction budgeting.
    bool enableTwoAxisFrictionSolve = true;
    bool enableManifoldFrictionBudget = true;
    bool frictionBudgetUseRadialClamp = true;
    Real manifoldFrictionBudgetScale = 1.0;
    // Defaults preserve prior behavior: manifold budget normal support from all manifold contacts.
    FrictionBudgetNormalSupportSource frictionBudgetNormalSupportSource =
        FrictionBudgetNormalSupportSource::AllManifoldContacts;
    // Used only when support source is BlendedSelectedPairAndManifold.
    Real frictionBudgetSelectedPairBlendWeight = 2.0;
    bool enablePersistentStickConstraints = true;
    Real stickVelocityThreshold = 0.08;
    std::uint16_t stickMinPersistenceAge = 2;
    Real stickImpulseRetention = 0.92;
};

struct JointSolverConfig {
    bool useBlockSolver = false;
    Real blockDeterminantEpsilon = 1e-8;
    Real blockDiagonalMinimum = 1e-6;
    Real blockConditionEstimateMax = 0.0;
    Real hingeAnchorBiasFactor = 0.2;
    Real hingeAnchorDampingFactor = 0.1;
    /// Maximum position error magnitude (meters) fed into the anchor bias term. Errors beyond
    /// this are capped so a large joint deformation (hard landing, pathological state) does not
    /// generate an explosive corrective impulse. 0 = disabled (no cap, legacy behaviour).
    Real hingeAnchorBiasMaxErrorM = 0.0;
    /// Post-integration servo position passes. Anchor corrections run every pass (Gauss-Seidel).
    /// Angular corrections (axis alignment + hinge snap) use direct position snap for simple
    /// chains, or Jacobi-averaged velocity biases for systems containing a star-topology hub
    /// (any body with > 2 servo joints). Use 0 to disable the block entirely.
    std::uint8_t servoPositionPasses = 4;
    /// Run `SolveJointPositions` every Nth substep (1 = every substep). Useful for CPU/stability tradeoffs
    /// without changing velocity-solve PGS iteration count.
    std::uint8_t servoPositionSolveStride = 1;
    /// Skip expensive servo anchor 3x3 block solve when both anchor error and relative anchor
    /// speed are already tiny; keeps axis/angle servo rows active.
    bool enableServoAnchorEarlyOut = true;
    /// Anchor error threshold (meters) for `enableServoAnchorEarlyOut`.
    Real servoAnchorEarlyOutError = 1.0e-4;
    /// Relative anchor speed threshold (m/s) for `enableServoAnchorEarlyOut`.
    Real servoAnchorEarlyOutSpeed = 0.02;
    /// Enable angular-axis alignment row early-out when axis error/omega are tiny.
    bool enableServoAngularEarlyOut = false;
    /// Axis misalignment threshold (radians, approximated by |axisA x axisB|) for angular early-out.
    Real servoAngularEarlyOutError = 1.0e-3;
    /// Relative angular speed threshold (rad/s) for angular early-out.
    Real servoAngularEarlyOutSpeed = 0.05;
    /// Enable hinge-axis row early-out when angle error/axis speed are tiny.
    bool enableServoHingeEarlyOut = false;
    /// Hinge angle error threshold (rad) for hinge-row early-out.
    Real servoHingeEarlyOutError = 1.0e-3;
    /// Hinge axis angular speed threshold (rad/s) for hinge-row early-out.
    Real servoHingeEarlyOutSpeed = 0.05;
    /// Exit threshold multiplier (>1) used for early-out hysteresis.
    Real servoEarlyOutResumeScale = 1.5;
    /// Prevent early-out while row impulse magnitude is above this fraction of max torque.
    Real servoEarlyOutImpulseGuardFraction = 0.2;
    /// Maximum position error magnitude (meters) fed into the servo anchor position correction
    /// each pass. Caps the per-joint displacement so a pathological state (free-Real, hard
    /// landing) cannot accumulate non-physical upward energy across passes and leg chains.
    /// 0 = disabled (no cap, legacy behaviour).
    Real servoAnchorCorrectionMaxM = 0.0;
    /// Maximum relative anchor velocity magnitude (m/s) fed into the damping term of the anchor
    /// constraint rhs (rhs = dampingFactor * relVel + anchorBias). Without a cap, a fast contact
    /// event can make dampingFactor*relVel arbitrarily large for light-body joints, launching the
    /// limb. 0 = disabled (no cap, legacy behaviour).
    Real hingeAnchorDampingMaxRelVelMs = 0.0;

    /// When true, the hinge servo PD constraint is solved as TWO separate PGS rows — one
    /// pure-stiffness row driving toward the target angle and one pure-damping row driving
    /// toward zero relative angular velocity. The legacy single-row formulation packs both
    /// into a single Catto soft-constraint row whose `bias` and softness `γ` are coupled,
    /// so the user-facing `dampingGain` (ζ) ends up reducing the effective position
    /// correction at large `dt × positionGain`. See test_servo_classical_pd_invariant for
    /// a focused regression that fails on the legacy path and passes on the decoupled path.
    ///
    /// Currently OPT-IN. The decoupled formulation has different effective dynamics than
    /// the legacy one for the same `(positionGain, dampingGain)` pair — the hexapod scene
    /// is tuned to the legacy formulation, and switching the default would silently change
    /// chassis behaviour. Tests that want classical-PD semantics enable the flag explicitly.
    /// We will revisit the default once the hexapod servo profile has been re-tuned for the
    /// decoupled formulation and the diagnostic suite (stand quiescence, substep convergence)
    /// confirms equivalence or improvement.
    bool enableServoStiffnessDampingDecoupling = false;
};

struct BroadphaseConfig {
    Real baseFatAabbMargin = 0.1;
    Real linearVelocityMarginScale = 1.0;
    Real angularVelocityMarginScale = 0.5;
    Real minSweptMargin = 0.05;
    Real maxSweptMargin = 1.0;

    Real partialRebuildAreaRatioThreshold = 2.5;
    Real partialRebuildAvgDepthThreshold = 24.0;
    Real partialRebuildMovedProxyRatioThreshold = 0.20;

    Real fullRebuildAreaRatioThreshold = 4.0;
    Real fullRebuildMaxDepthThreshold = 96.0;
    Real fullRebuildMovedProxyRatioThreshold = 0.50;
    Real fullRebuildQueryNodeVisitsPerProxyThreshold = 256.0;
    bool enableMovedSetOnlyUpdates = true;
    bool enablePairCacheReuseForQuasiStatic = true;

    std::uint32_t minStepsBetweenRebuilds = 8;
};

struct BroadphaseMetrics {
    Real areaRatio = 0.0;
    Real maxDepth = 0.0;
    Real avgDepth = 0.0;
    Real movedProxyRatio = 0.0;
    Real queryNodeVisitsPerProxy = 0.0;
    Real pairGenerationMs = 0.0;
    Real pairCacheHitRate = 0.0;
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
    Real penetration = 0.0;
    Real normalImpulseSum = 0.0;
    // Legacy scalar tangent accumulator retained for compatibility/migration.
    Real tangentImpulseSum = 0.0;
    Real tangentImpulseSum0 = 0.0;
    Real tangentImpulseSum1 = 0.0;
    std::uint8_t manifoldType = 0;
    std::uint64_t key = 0;
    std::uint64_t featureKey = 0;
    std::uint16_t persistenceAge = 0;
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Real referenceSeparation = 0.0;
    bool anchorsValid = false;
};

struct Manifold {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    std::uint8_t manifoldType = 0;
    std::vector<Contact> contacts;
    std::array<Real, 2> blockNormalImpulseSum{0.0, 0.0};
    std::array<std::uint64_t, 2> blockContactKeys{0u, 0u};
    std::array<bool, 2> blockSlotValid{false, false};
    std::array<int, 2> selectedBlockContactIndices{-1, -1};
    std::array<std::uint64_t, 2> selectedBlockContactKeys{0u, 0u};
    Vec3 t0{};
    Vec3 t1{};
    bool tangentBasisValid = false;
    std::array<Real, 2> manifoldTangentImpulseSum{0.0, 0.0};
    bool manifoldTangentImpulseValid = false;
    bool stickConstraintActive = false;
    std::uint16_t stickConstraintAge = 0;
    // Cached per-contact impulse state: [normal, tangent0, tangent1].
    std::unordered_map<std::uint64_t, std::array<Real, 3>> cachedImpulseByContactKey{};
    bool selectedBlockPairPersistent = false;
    bool selectedBlockPairQualityPass = false;
    bool lowQuality = false;
    bool blockSolveEligible = false;
    bool usedBlockSolve = false;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    struct BlockSolveDebugCounters {
        std::array<Real, 2> selectedPreNormalImpulses{0.0, 0.0};
        std::array<Real, 2> selectedPostNormalImpulses{0.0, 0.0};
        Real selectedPairPenetrationStep = 0.0;
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

struct IslandOrderResult {
    std::vector<std::size_t> manifoldOrder;
    IslandSolveOrdering orderingUsed = IslandSolveOrdering::Insertion;
    bool supportDepthApplied = false;
};


inline bool ValidateContactSolverConfig(const ContactSolverConfig& config) {
    constexpr Real kMinSaneToiStep = 1e-9;
    if (config.toi.max_iterations < 1) {
        return false;
    }
    if (!std::isfinite(config.toi.min_time_step) || config.toi.min_time_step < kMinSaneToiStep) {
        return false;
    }
    if (!std::isfinite(config.block4.symmetry_tolerance) || config.block4.symmetry_tolerance < 0.0) {
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
    if (!std::isfinite(sanitized.toi.min_time_step) || sanitized.toi.min_time_step < 1e-9) {
        sanitized.toi.min_time_step = defaults.toi.min_time_step;
    }
    if (!std::isfinite(sanitized.block4.symmetry_tolerance) || sanitized.block4.symmetry_tolerance < 0.0) {
        sanitized.block4.symmetry_tolerance = defaults.block4.symmetry_tolerance;
    }

    return sanitized;
}

} // namespace minphys3d
