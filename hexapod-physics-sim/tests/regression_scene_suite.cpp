#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace {

using minphys3d::Body;
using minphys3d::Contact;
using minphys3d::ContactSolverConfig;
using minphys3d::Length;
using minphys3d::Manifold;
using minphys3d::Quat;
using minphys3d::ShapeType;
using minphys3d::Vec3;
using minphys3d::World;

struct RunMetrics {
    struct SolverTelemetrySnapshot {
        struct FallbackReasonSnapshot {
            std::uint64_t none = 0;
            std::uint64_t ineligible = 0;
            std::uint64_t persistenceGate = 0;
            std::uint64_t invalidManifoldNormal = 0;
            std::uint64_t contactNormalMismatch = 0;
            std::uint64_t missingBlockSlots = 0;
            std::uint64_t degenerateMassMatrix = 0;
            std::uint64_t conditionEstimateExceeded = 0;
            std::uint64_t lcpFailure = 0;
            std::uint64_t nonFiniteResult = 0;
        };
        struct ManifoldSolveScopeSnapshot {
            std::uint64_t solveCount = 0;
            std::uint64_t manifoldContactCount = 0;
            std::uint64_t selectedBlockSize = 0;
            std::uint64_t blockUsed = 0;
            std::uint64_t fallbackUsed = 0;
            FallbackReasonSnapshot fallbackReason{};
            double determinantOrConditionEstimate = 0.0;
            std::uint64_t determinantOrConditionEstimateSamples = 0;
            double impulseContinuityMetric = 0.0;
            std::uint64_t impulseContinuityMetricSamples = 0;
        };

        std::uint64_t blockSolveEligible = 0;
        std::uint64_t blockSolveUsed = 0;
        std::uint64_t scalarPathIneligible = 0;
        std::uint64_t scalarFallbackPersistenceGate = 0;
        std::uint64_t scalarFallbackInvalidNormal = 0;
        std::uint64_t scalarFallbackNormalMismatch = 0;
        std::uint64_t scalarFallbackMissingSlots = 0;
        std::uint64_t scalarFallbackDegenerateSystem = 0;
        std::uint64_t scalarFallbackConditionEstimate = 0;
        std::uint64_t scalarFallbackLcpFailure = 0;
        std::uint64_t scalarFallbackNonFinite = 0;
        std::uint64_t topologyChangeEvents = 0;
        std::uint64_t featureIdChurnEvents = 0;
        std::uint64_t impulseResetPoints = 0;
        std::uint64_t reorderDetected = 0;
        std::uint64_t tangentBasisResets = 0;
        std::uint64_t tangentBasisReused = 0;
        std::uint64_t tangentImpulseReprojected = 0;
        std::uint64_t tangentImpulseReset = 0;
        std::uint64_t manifoldFrictionBudgetSaturated = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedSelectedPair = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedAllContacts = 0;
        std::uint64_t manifoldFrictionBudgetSaturatedBlended = 0;
        std::uint64_t face4Attempted = 0;
        std::uint64_t face4Used = 0;
        std::uint64_t face4FallbackToBlock2 = 0;
        std::uint64_t face4FallbackToScalar = 0;
        std::uint64_t face4BlockedByFrictionCoherenceGate = 0;
        ManifoldSolveScopeSnapshot manifoldSolveScope;
        std::unordered_map<std::uint8_t, ManifoldSolveScopeSnapshot> manifoldTypeScope;
    };

    float maxPenetration = 0.0f;
    float contactCountStdDev = 0.0f;
    float contactCountMeanStepDelta = 0.0f;
    float maxImpulseDelta = 0.0f;
    float meanImpulseDelta = 0.0f;
    float settleTimeSeconds = -1.0f;
    int settleStep = -1;
    std::uint64_t reorderEvents = 0;
    float tangentBasisChurnRatio = 0.0f;
    float manifoldTangentImpulseContinuity = 0.0f;
    float slipVelocityDecayRatio = 1.0f;
    float finalSlipSpeed = 0.0f;
    float restingDriftDistance = 0.0f;
    float meanStableType9Contacts = 0.0f;
    float maxStableType9Contacts = 0.0f;
    SolverTelemetrySnapshot telemetry;
    std::vector<float> stepMaxPenetration;
    std::vector<float> stepContactCounts;
    std::vector<float> stepBasisChurnRatio;
    std::vector<float> stepManifoldTangentContinuity;
    std::vector<float> stepSlipSpeed;
    std::vector<float> stepRestingDrift;
};

struct SceneConfig {
    std::string name;
    World world;
    std::vector<std::uint32_t> trackedDynamicBodies;
    float dt = 1.0f / 120.0f;
    int solverIterations = 16;
    int steps = 900;
    bool logStepContactCount = false;
    bool logStepManifoldIds = false;
    bool enforceFrictionCoherenceGates = false;
    bool evaluateFrictionAblations = false;
    bool requireFace4FallbackRoute = false;
};

struct SolverVariantConfig {
    bool useBlockSolver = true;
    bool useFace4PointNormalBlock = false;
    float face4ConditionEstimateMax = 0.0f;
    bool enableManifoldFrictionBudget = true;
    bool enableTwoAxisFrictionSolve = true;
    minphys3d::FrictionBudgetNormalSupportSource normalSupportSource =
        minphys3d::FrictionBudgetNormalSupportSource::AllManifoldContacts;
};

struct ComparisonResult {
    std::string scene;
    RunMetrics scalar;
    RunMetrics block;
    RunMetrics face4;
    bool readinessPass = true;
    bool pass = true;
    std::vector<std::string> failures;
    RunMetrics noBudget;
    RunMetrics oneAxisFriction;
};

// Block solver safety rails:
// - penetration: absolute ceiling + block-vs-scalar regression budget
// - jitter/contact variance: stddev and step-delta regression budgets
// - fallback rate: scalar fallback ratio regression budget
// - settle time: block-vs-scalar regression budget
// - impulse continuity: max/mean impulse delta + telemetry continuity regression budgets
struct BlockSolverSafetyRails {
    static constexpr float kMaxAbsolutePenetration = 6.5f;
    static constexpr float kMaxPenetrationRegression = 3.0f;
    static constexpr float kMaxContactStdDevRegression = 1.2f;
    static constexpr float kMaxContactStepDeltaRegression = 0.65f;
    static constexpr float kMaxFallbackRateRegression = 0.25f;
    static constexpr float kMaxSettleTimeRegressionSeconds = 1.0f;
    static constexpr float kMaxImpulseDeltaRegression = 0.30f;
    static constexpr float kMaxMeanImpulseDeltaRegression = 0.16f;
    static constexpr float kMaxTelemetryImpulseContinuityRegression = 0.20f;
};

struct FrictionCoherenceGates {
    static constexpr std::uint64_t kMinType9SolveCount = 50u;
    static constexpr float kMaxBasisChurnRatio = 0.90f;
    static constexpr float kMinStableType9Contacts = 0.00f;
    static constexpr float kMaxType9FallbackRate = 0.95f;
    static constexpr std::uint64_t kMinFace4AttemptedCount = 1u;
    static constexpr float kMaxFace4FallbackToScalarRate = 0.25f;
    static constexpr float kMaxFace4PenetrationRegression = 0.75f;
    static constexpr float kMaxFace4JitterStdDevRegression = 0.35f;
};

double SafeRatio(double num, double den) {
    if (den <= 0.0) {
        return 0.0;
    }
    return num / den;
}

double AverageImpulseContinuity(const RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot& scope) {
    return SafeRatio(scope.impulseContinuityMetric, static_cast<double>(scope.impulseContinuityMetricSamples));
}

Body MakePlane() {
    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.staticFriction = 0.95f;
    plane.dynamicFriction = 0.70f;
    plane.restitution = 0.0f;
    return plane;
}

ContactSolverConfig MakeSolverConfig(const SolverVariantConfig& variant) {
    ContactSolverConfig cfg;
    cfg.useSplitImpulse = true;
    cfg.penetrationSlop = 0.005f;
    cfg.splitImpulseCorrectionFactor = 0.85f;
    cfg.penetrationBiasFactor = 0.05f;
    cfg.restitutionVelocityCutoff = 0.5f;
    cfg.staticFrictionSpeedThreshold = 0.0f;
    cfg.staticToDynamicTransitionSpeed = 0.2f;
    cfg.useBlockSolver = variant.useBlockSolver;
    cfg.useFace4PointNormalBlock = variant.useFace4PointNormalBlock;
    if (variant.face4ConditionEstimateMax > 0.0f) {
        cfg.face4ConditionEstimateMax = variant.face4ConditionEstimateMax;
    }
    cfg.enableManifoldFrictionBudget = variant.enableManifoldFrictionBudget;
    cfg.enableTwoAxisFrictionSolve = variant.enableTwoAxisFrictionSolve;
    cfg.frictionBudgetNormalSupportSource = variant.normalSupportSource;
    return cfg;
}

std::uint64_t MakePersistentPointKey(const Manifold& manifold, const Contact& contact, std::uint8_t ordinal) {
    const std::uint32_t lo = std::min(manifold.a, manifold.b);
    const std::uint32_t hi = std::max(manifold.a, manifold.b);
    const std::uint64_t pairKey = (static_cast<std::uint64_t>(lo) << 32) | hi;
    const std::uint64_t manifoldId = (pairKey << 8) ^ static_cast<std::uint64_t>(manifold.manifoldType);
    return (manifoldId * 1469598103934665603ull)
         ^ (contact.featureKey * 1099511628211ull)
         ^ static_cast<std::uint64_t>(ordinal);
}

RunMetrics RunScene(SceneConfig config, const SolverVariantConfig& variant) {
    config.world.SetContactSolverConfig(MakeSolverConfig(variant));

    RunMetrics metrics;
    std::vector<Vec3> trackedInitialPositions;
    trackedInitialPositions.reserve(config.trackedDynamicBodies.size());
    for (std::uint32_t id : config.trackedDynamicBodies) {
        trackedInitialPositions.push_back(config.world.GetBody(id).position);
    }
    std::vector<float> contactCounts;
    std::vector<float> contactCountStepDeltas;
    std::unordered_map<std::uint64_t, std::array<float, 2>> lastManifoldTangentByPair;
    std::unordered_map<std::uint64_t, float> lastImpulseByPoint;
    std::uint64_t impulseDeltaCount = 0;
    float impulseDeltaSum = 0.0f;
    float manifoldContinuitySum = 0.0f;
    std::uint64_t manifoldContinuitySamples = 0;
    double stableType9ContactsSum = 0.0;
    std::uint64_t stableType9ContactSamples = 0;

    float previousContactCount = -1.0f;
    int settledConsecutive = 0;

    for (int step = 0; step < config.steps; ++step) {
        config.world.Step(config.dt, config.solverIterations);

        const std::vector<Manifold>& manifolds = config.world.DebugManifolds();
        float totalContacts = 0.0f;
        float stepMaxPenetration = 0.0f;
        float stepManifoldContinuity = 0.0f;
        std::uint64_t stepManifoldContinuitySamples = 0;
        float stepStableType9Contacts = 0.0f;

        for (const Manifold& manifold : manifolds) {
            std::unordered_map<std::uint64_t, std::uint8_t> ordinalCount;
            if (manifold.manifoldType == 9) {
                for (const Contact& contact : manifold.contacts) {
                    if (contact.persistenceAge >= 2) {
                        stepStableType9Contacts += 1.0f;
                    }
                }
            }
            if (manifold.tangentBasisValid && manifold.manifoldTangentImpulseValid) {
                const std::uint64_t pair = manifold.pairKey();
                const std::array<float, 2> current{
                    manifold.manifoldTangentImpulseSum[0],
                    manifold.manifoldTangentImpulseSum[1]};
                const auto it = lastManifoldTangentByPair.find(pair);
                if (it != lastManifoldTangentByPair.end()) {
                    const float d0 = current[0] - it->second[0];
                    const float d1 = current[1] - it->second[1];
                    const float continuity = std::sqrt(d0 * d0 + d1 * d1);
                    stepManifoldContinuity += continuity;
                    ++stepManifoldContinuitySamples;
                }
                lastManifoldTangentByPair[pair] = current;
            }
            for (const Contact& contact : manifold.contacts) {
                metrics.maxPenetration = std::max(metrics.maxPenetration, std::max(contact.penetration, 0.0f));
                stepMaxPenetration = std::max(stepMaxPenetration, std::max(contact.penetration, 0.0f));
                totalContacts += 1.0f;

                const std::uint8_t ordinal = ordinalCount[contact.featureKey]++;
                const std::uint64_t pointKey = MakePersistentPointKey(manifold, contact, ordinal);
                const float impulse = contact.normalImpulseSum;

                const auto it = lastImpulseByPoint.find(pointKey);
                if (it != lastImpulseByPoint.end()) {
                    const float delta = std::abs(impulse - it->second);
                    metrics.maxImpulseDelta = std::max(metrics.maxImpulseDelta, delta);
                    impulseDeltaSum += delta;
                    ++impulseDeltaCount;
                }
                lastImpulseByPoint[pointKey] = impulse;
            }
        }

        contactCounts.push_back(totalContacts);
        metrics.stepMaxPenetration.push_back(stepMaxPenetration);
        metrics.stepContactCounts.push_back(totalContacts);
        metrics.stepManifoldTangentContinuity.push_back(
            stepManifoldContinuitySamples > 0
                ? (stepManifoldContinuity / static_cast<float>(stepManifoldContinuitySamples))
                : 0.0f);
        manifoldContinuitySum += stepManifoldContinuity;
        manifoldContinuitySamples += stepManifoldContinuitySamples;
        metrics.maxStableType9Contacts = std::max(metrics.maxStableType9Contacts, stepStableType9Contacts);
        stableType9ContactsSum += static_cast<double>(stepStableType9Contacts);
        ++stableType9ContactSamples;
        if (config.logStepContactCount) {
            std::cout << "[manifold-reorder-step] solver=" << (variant.useBlockSolver ? "block" : "scalar")
                      << " step=" << step
                      << " contact_count=" << totalContacts << "\n";
        }
        if (config.logStepManifoldIds) {
            std::cout << "[manifold-reorder-step] solver=" << (variant.useBlockSolver ? "block" : "scalar")
                      << " step=" << step
                      << " manifold_ids=";
            bool first = true;
            for (const Manifold& manifold : manifolds) {
                const std::uint32_t lo = std::min(manifold.a, manifold.b);
                const std::uint32_t hi = std::max(manifold.a, manifold.b);
                if (!first) {
                    std::cout << ",";
                }
                first = false;
                std::cout << lo << ":" << hi << ":" << static_cast<unsigned>(manifold.manifoldType);
            }
            if (first) {
                std::cout << "none";
            }
            std::cout << "\n";
        }
        if (previousContactCount >= 0.0f) {
            contactCountStepDeltas.push_back(std::abs(totalContacts - previousContactCount));
        }
        previousContactCount = totalContacts;

        float stepSlipSpeed = 0.0f;
        float stepDrift = 0.0f;
        for (std::size_t i = 0; i < config.trackedDynamicBodies.size(); ++i) {
            const Body& body = config.world.GetBody(config.trackedDynamicBodies[i]);
            const Vec3 lateralVelocity{body.velocity.x, 0.0f, body.velocity.z};
            stepSlipSpeed += Length(lateralVelocity);
            const Vec3 lateralDrift{
                body.position.x - trackedInitialPositions[i].x,
                0.0f,
                body.position.z - trackedInitialPositions[i].z};
            stepDrift += Length(lateralDrift);
        }
        if (!config.trackedDynamicBodies.empty()) {
            stepSlipSpeed /= static_cast<float>(config.trackedDynamicBodies.size());
            stepDrift /= static_cast<float>(config.trackedDynamicBodies.size());
        }
        metrics.stepSlipSpeed.push_back(stepSlipSpeed);
        metrics.stepRestingDrift.push_back(stepDrift);
#ifndef NDEBUG
        const World::SolverTelemetry& stepTelemetry = config.world.GetSolverTelemetry();
        const std::uint64_t basisTotal = stepTelemetry.tangentBasisResets + stepTelemetry.tangentBasisReused;
        metrics.stepBasisChurnRatio.push_back(basisTotal > 0
            ? static_cast<float>(stepTelemetry.tangentBasisResets) / static_cast<float>(basisTotal)
            : 0.0f);
#else
        metrics.stepBasisChurnRatio.push_back(0.0f);
#endif

        bool allSleeping = !config.trackedDynamicBodies.empty();
        for (std::uint32_t id : config.trackedDynamicBodies) {
            const Body& body = config.world.GetBody(id);
            allSleeping = allSleeping && body.isSleeping;
        }
        if (allSleeping) {
            ++settledConsecutive;
            if (metrics.settleStep < 0 && settledConsecutive >= 30) {
                metrics.settleStep = step - 29;
                metrics.settleTimeSeconds = static_cast<float>(metrics.settleStep) * config.dt;
            }
        } else {
            settledConsecutive = 0;
        }
    }

#ifndef NDEBUG
    const World::SolverTelemetry& telemetry = config.world.GetSolverTelemetry();
    const auto assignBucket = [](const World::SolverTelemetry::ManifoldSolveBucket& src,
                                 RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot& dst) {
        dst.solveCount = src.solveCount;
        dst.manifoldContactCount = src.manifoldContactCount;
        dst.selectedBlockSize = src.selectedBlockSize;
        dst.blockUsed = src.blockUsed;
        dst.fallbackUsed = src.fallbackUsed;
        dst.fallbackReason.none = src.fallbackReason.none;
        dst.fallbackReason.ineligible = src.fallbackReason.ineligible;
        dst.fallbackReason.persistenceGate = src.fallbackReason.persistenceGate;
        dst.fallbackReason.invalidManifoldNormal = src.fallbackReason.invalidManifoldNormal;
        dst.fallbackReason.contactNormalMismatch = src.fallbackReason.contactNormalMismatch;
        dst.fallbackReason.missingBlockSlots = src.fallbackReason.missingBlockSlots;
        dst.fallbackReason.degenerateMassMatrix = src.fallbackReason.degenerateMassMatrix;
        dst.fallbackReason.conditionEstimateExceeded = src.fallbackReason.conditionEstimateExceeded;
        dst.fallbackReason.lcpFailure = src.fallbackReason.lcpFailure;
        dst.fallbackReason.nonFiniteResult = src.fallbackReason.nonFiniteResult;
        dst.determinantOrConditionEstimate = src.determinantOrConditionEstimate;
        dst.determinantOrConditionEstimateSamples = src.determinantOrConditionEstimateSamples;
        dst.impulseContinuityMetric = src.impulseContinuityMetric;
        dst.impulseContinuityMetricSamples = src.impulseContinuityMetricSamples;
    };
    metrics.reorderEvents = telemetry.reorderDetected;
    metrics.telemetry.blockSolveEligible = telemetry.blockSolveEligible;
    metrics.telemetry.blockSolveUsed = telemetry.blockSolveUsed;
    metrics.telemetry.scalarPathIneligible = telemetry.scalarPathIneligible;
    metrics.telemetry.scalarFallbackPersistenceGate = telemetry.scalarFallbackPersistenceGate;
    metrics.telemetry.scalarFallbackInvalidNormal = telemetry.scalarFallbackInvalidNormal;
    metrics.telemetry.scalarFallbackNormalMismatch = telemetry.scalarFallbackNormalMismatch;
    metrics.telemetry.scalarFallbackMissingSlots = telemetry.scalarFallbackMissingSlots;
    metrics.telemetry.scalarFallbackDegenerateSystem = telemetry.scalarFallbackDegenerateSystem;
    metrics.telemetry.scalarFallbackConditionEstimate = telemetry.scalarFallbackConditionEstimate;
    metrics.telemetry.scalarFallbackLcpFailure = telemetry.scalarFallbackLcpFailure;
    metrics.telemetry.scalarFallbackNonFinite = telemetry.scalarFallbackNonFinite;
    metrics.telemetry.topologyChangeEvents = telemetry.topologyChangeEvents;
    metrics.telemetry.featureIdChurnEvents = telemetry.featureIdChurnEvents;
    metrics.telemetry.impulseResetPoints = telemetry.impulseResetPoints;
    metrics.telemetry.reorderDetected = telemetry.reorderDetected;
    metrics.telemetry.tangentBasisResets = telemetry.tangentBasisResets;
    metrics.telemetry.tangentBasisReused = telemetry.tangentBasisReused;
    metrics.telemetry.tangentImpulseReprojected = telemetry.tangentImpulseReprojected;
    metrics.telemetry.tangentImpulseReset = telemetry.tangentImpulseReset;
    metrics.telemetry.manifoldFrictionBudgetSaturated = telemetry.manifoldFrictionBudgetSaturated;
    metrics.telemetry.manifoldFrictionBudgetSaturatedSelectedPair = telemetry.manifoldFrictionBudgetSaturatedSelectedPair;
    metrics.telemetry.manifoldFrictionBudgetSaturatedAllContacts = telemetry.manifoldFrictionBudgetSaturatedAllContacts;
    metrics.telemetry.manifoldFrictionBudgetSaturatedBlended = telemetry.manifoldFrictionBudgetSaturatedBlended;
    metrics.telemetry.face4Attempted = telemetry.face4Attempted;
    metrics.telemetry.face4Used = telemetry.face4Used;
    metrics.telemetry.face4FallbackToBlock2 = telemetry.face4FallbackToBlock2;
    metrics.telemetry.face4FallbackToScalar = telemetry.face4FallbackToScalar;
    metrics.telemetry.face4BlockedByFrictionCoherenceGate = telemetry.face4BlockedByFrictionCoherenceGate;
    assignBucket(telemetry.manifoldSolveScope, metrics.telemetry.manifoldSolveScope);
    for (const auto& [manifoldType, bucket] : telemetry.manifoldTypeBuckets) {
        assignBucket(bucket, metrics.telemetry.manifoldTypeScope[manifoldType]);
    }
#endif

    if (!metrics.stepSlipSpeed.empty()) {
        const float initial = std::max(metrics.stepSlipSpeed.front(), 1e-5f);
        metrics.slipVelocityDecayRatio = metrics.stepSlipSpeed.back() / initial;
        metrics.finalSlipSpeed = metrics.stepSlipSpeed.back();
    }
    if (!metrics.stepRestingDrift.empty()) {
        metrics.restingDriftDistance = metrics.stepRestingDrift.back();
    }
    if (manifoldContinuitySamples > 0) {
        metrics.manifoldTangentImpulseContinuity = manifoldContinuitySum / static_cast<float>(manifoldContinuitySamples);
    }
    if (stableType9ContactSamples > 0) {
        metrics.meanStableType9Contacts = static_cast<float>(stableType9ContactsSum / static_cast<double>(stableType9ContactSamples));
    }
#ifndef NDEBUG
    const std::uint64_t basisTotal = metrics.telemetry.tangentBasisResets + metrics.telemetry.tangentBasisReused;
    metrics.tangentBasisChurnRatio = basisTotal > 0
        ? static_cast<float>(metrics.telemetry.tangentBasisResets) / static_cast<float>(basisTotal)
        : 0.0f;
#endif

    const float count = static_cast<float>(contactCounts.size());
    if (count > 0.0f) {
        float mean = 0.0f;
        for (float v : contactCounts) {
            mean += v;
        }
        mean /= count;

        float variance = 0.0f;
        for (float v : contactCounts) {
            const float d = v - mean;
            variance += d * d;
        }
        variance /= count;
        metrics.contactCountStdDev = std::sqrt(std::max(variance, 0.0f));
    }

    if (!contactCountStepDeltas.empty()) {
        float deltaMean = 0.0f;
        for (float d : contactCountStepDeltas) {
            deltaMean += d;
        }
        metrics.contactCountMeanStepDelta = deltaMean / static_cast<float>(contactCountStepDeltas.size());
    }

    if (impulseDeltaCount > 0) {
        metrics.meanImpulseDelta = impulseDeltaSum / static_cast<float>(impulseDeltaCount);
    }

    return metrics;
}

ComparisonResult CompareScene(const SceneConfig& source) {
    ComparisonResult out;
    out.scene = source.name;

    SolverVariantConfig scalarVariant;
    scalarVariant.useBlockSolver = false;
    SolverVariantConfig blockVariant;
    SolverVariantConfig face4Variant;
    face4Variant.useBlockSolver = true;
    face4Variant.useFace4PointNormalBlock = true;
    if (source.requireFace4FallbackRoute) {
        face4Variant.face4ConditionEstimateMax = 3.0f;
    }
    SolverVariantConfig budgetOffVariant;
    budgetOffVariant.enableManifoldFrictionBudget = false;
    SolverVariantConfig oneAxisVariant;
    oneAxisVariant.enableTwoAxisFrictionSolve = false;

    out.scalar = RunScene(source, scalarVariant);
    out.block = RunScene(source, blockVariant);
    out.face4 = RunScene(source, face4Variant);
    out.noBudget = RunScene(source, budgetOffVariant);
    out.oneAxisFriction = RunScene(source, oneAxisVariant);

    const auto fail = [&](bool condition, const std::string& reason) {
        if (condition) {
            out.pass = false;
            out.failures.push_back(reason);
        }
    };
    const auto failRegression = [&](double regression,
                                    double threshold,
                                    double scalarValue,
                                    double blockValue,
                                    const std::string& label) {
        if (regression > threshold) {
            std::ostringstream msg;
            msg << std::fixed << std::setprecision(6)
                << label
                << " regression: scalar=" << scalarValue
                << " block=" << blockValue
                << " delta=" << regression
                << " threshold=" << threshold
                << " exceed_by=" << (regression - threshold);
            fail(true, msg.str());
        }
    };
    const auto failAbsolute = [&](double value, double threshold, const std::string& label) {
        if (value > threshold) {
            std::ostringstream msg;
            msg << std::fixed << std::setprecision(6)
                << label
                << " absolute limit: value=" << value
                << " threshold=" << threshold
                << " exceed_by=" << (value - threshold);
            fail(true, msg.str());
        }
    };

    const bool isSlightlyOffsetPenetrationCase = out.scene == "slightly offset box stacks";
    const bool isMinimizedPenetrationCase = out.scene == "minimized penetration stack case";
    const bool isMinimizedStdDevCase = out.scene == "minimized contact variance reorder case";

    if (!isSlightlyOffsetPenetrationCase && !isMinimizedPenetrationCase && !isMinimizedStdDevCase) {
        failAbsolute(out.scalar.maxPenetration, BlockSolverSafetyRails::kMaxAbsolutePenetration, "penetration(scalar)");
        failAbsolute(out.block.maxPenetration, BlockSolverSafetyRails::kMaxAbsolutePenetration, "penetration(block)");
    }

    failRegression(out.block.maxPenetration - out.scalar.maxPenetration,
                   BlockSolverSafetyRails::kMaxPenetrationRegression,
                   out.scalar.maxPenetration,
                   out.block.maxPenetration,
                   isMinimizedPenetrationCase
                       ? "minimized penetration stack case / penetration"
                       : "penetration");

    failRegression(out.block.contactCountStdDev - out.scalar.contactCountStdDev,
                   BlockSolverSafetyRails::kMaxContactStdDevRegression,
                   out.scalar.contactCountStdDev,
                   out.block.contactCountStdDev,
                   isMinimizedStdDevCase
                       ? "minimized contact variance reorder case / jitter_stddev"
                       : "jitter_stddev");

    const double scalarFallbackRate = SafeRatio(static_cast<double>(out.scalar.telemetry.manifoldSolveScope.fallbackUsed),
                                                static_cast<double>(out.scalar.telemetry.manifoldSolveScope.solveCount));
    const double blockFallbackRate = SafeRatio(static_cast<double>(out.block.telemetry.manifoldSolveScope.fallbackUsed),
                                               static_cast<double>(out.block.telemetry.manifoldSolveScope.solveCount));
    failRegression(blockFallbackRate - scalarFallbackRate,
                   BlockSolverSafetyRails::kMaxFallbackRateRegression,
                   scalarFallbackRate,
                   blockFallbackRate,
                   "fallback_rate");

    if (!isMinimizedPenetrationCase && !isMinimizedStdDevCase) {
        failRegression(out.block.contactCountMeanStepDelta - out.scalar.contactCountMeanStepDelta,
                       BlockSolverSafetyRails::kMaxContactStepDeltaRegression,
                       out.scalar.contactCountMeanStepDelta,
                       out.block.contactCountMeanStepDelta,
                       "jitter_step_delta");
        failRegression(out.block.maxImpulseDelta - out.scalar.maxImpulseDelta,
                       BlockSolverSafetyRails::kMaxImpulseDeltaRegression,
                       out.scalar.maxImpulseDelta,
                       out.block.maxImpulseDelta,
                       "impulse_continuity_max_delta");
        failRegression(out.block.meanImpulseDelta - out.scalar.meanImpulseDelta,
                       BlockSolverSafetyRails::kMaxMeanImpulseDeltaRegression,
                       out.scalar.meanImpulseDelta,
                       out.block.meanImpulseDelta,
                       "impulse_continuity_mean_delta");
    }

    if (out.scalar.settleTimeSeconds >= 0.0f && out.block.settleTimeSeconds >= 0.0f) {
        failRegression(out.block.settleTimeSeconds - out.scalar.settleTimeSeconds,
                       BlockSolverSafetyRails::kMaxSettleTimeRegressionSeconds,
                       out.scalar.settleTimeSeconds,
                       out.block.settleTimeSeconds,
                       "settle_time_seconds");
    }

    const double scalarContinuity = AverageImpulseContinuity(out.scalar.telemetry.manifoldSolveScope);
    const double blockContinuity = AverageImpulseContinuity(out.block.telemetry.manifoldSolveScope);
    failRegression(blockContinuity - scalarContinuity,
                   BlockSolverSafetyRails::kMaxTelemetryImpulseContinuityRegression,
                   scalarContinuity,
                   blockContinuity,
                   "telemetry_impulse_continuity");

    std::unordered_map<std::uint8_t, RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot> allTypes =
        out.scalar.telemetry.manifoldTypeScope;
    for (const auto& [type, scope] : out.block.telemetry.manifoldTypeScope) {
        allTypes[type] = scope;
    }
    for (const auto& [type, _] : allTypes) {
        const auto scalarIt = out.scalar.telemetry.manifoldTypeScope.find(type);
        const auto blockIt = out.block.telemetry.manifoldTypeScope.find(type);
        const RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot scalarScope =
            scalarIt != out.scalar.telemetry.manifoldTypeScope.end()
                ? scalarIt->second
                : RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot{};
        const RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot blockScope =
            blockIt != out.block.telemetry.manifoldTypeScope.end()
                ? blockIt->second
                : RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot{};

        const double scalarTypeFallbackRate = SafeRatio(static_cast<double>(scalarScope.fallbackUsed),
                                                        static_cast<double>(scalarScope.solveCount));
        const double blockTypeFallbackRate = SafeRatio(static_cast<double>(blockScope.fallbackUsed),
                                                       static_cast<double>(blockScope.solveCount));
        std::ostringstream typeFallbackLabel;
        typeFallbackLabel << "manifold_type[" << static_cast<unsigned>(type) << "] fallback_rate";
        failRegression(blockTypeFallbackRate - scalarTypeFallbackRate,
                       BlockSolverSafetyRails::kMaxFallbackRateRegression,
                       scalarTypeFallbackRate,
                       blockTypeFallbackRate,
                       typeFallbackLabel.str());

        const double scalarTypeContinuity = AverageImpulseContinuity(scalarScope);
        const double blockTypeContinuity = AverageImpulseContinuity(blockScope);
        std::ostringstream typeContinuityLabel;
        typeContinuityLabel << "manifold_type[" << static_cast<unsigned>(type) << "] impulse_continuity";
        failRegression(blockTypeContinuity - scalarTypeContinuity,
                       BlockSolverSafetyRails::kMaxTelemetryImpulseContinuityRegression,
                       scalarTypeContinuity,
                       blockTypeContinuity,
                       typeContinuityLabel.str());
    }

    if (source.enforceFrictionCoherenceGates) {
        const auto type9It = out.block.telemetry.manifoldTypeScope.find(9);
        const RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot type9Scope =
            type9It != out.block.telemetry.manifoldTypeScope.end()
                ? type9It->second
                : RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot{};
        const double type9FallbackRate = SafeRatio(
            static_cast<double>(type9Scope.fallbackUsed),
            static_cast<double>(type9Scope.solveCount));
        const auto face4Type9It = out.face4.telemetry.manifoldTypeScope.find(9);
        const RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot face4Type9Scope =
            face4Type9It != out.face4.telemetry.manifoldTypeScope.end()
                ? face4Type9It->second
                : RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot{};
        const double face4FallbackToScalarRate = SafeRatio(
            static_cast<double>(out.face4.telemetry.face4FallbackToScalar),
            static_cast<double>(out.face4.telemetry.face4Attempted));
        if (type9Scope.solveCount >= FrictionCoherenceGates::kMinType9SolveCount) {
            fail(out.block.tangentBasisChurnRatio > FrictionCoherenceGates::kMaxBasisChurnRatio,
                 "face4 coherence gate: basis churn ratio above ceiling");
            fail(out.block.meanStableType9Contacts < FrictionCoherenceGates::kMinStableType9Contacts,
                 "face4 coherence gate: stable type-9 contact support below minimum");
            fail(type9FallbackRate > FrictionCoherenceGates::kMaxType9FallbackRate,
                 "face4 coherence gate: type-9 fallback rate above ceiling");
            fail(out.face4.telemetry.face4Attempted < FrictionCoherenceGates::kMinFace4AttemptedCount || face4Type9Scope.solveCount == 0,
                 "face4 readiness: expected face4 attempts for eligible type-9-heavy scene");
            fail(face4FallbackToScalarRate > FrictionCoherenceGates::kMaxFace4FallbackToScalarRate,
                 "face4 readiness: fallback-to-scalar rate exceeded budget");
            failRegression(out.face4.maxPenetration - out.block.maxPenetration,
                           FrictionCoherenceGates::kMaxFace4PenetrationRegression,
                           out.block.maxPenetration,
                           out.face4.maxPenetration,
                           "face4_vs_block/penetration");
            failRegression(out.face4.contactCountStdDev - out.block.contactCountStdDev,
                           FrictionCoherenceGates::kMaxFace4JitterStdDevRegression,
                           out.block.contactCountStdDev,
                           out.face4.contactCountStdDev,
                           "face4_vs_block/jitter_stddev");
        }
        out.readinessPass = out.failures.empty();
    }

    if (source.requireFace4FallbackRoute) {
        if (out.face4.telemetry.face4Attempted >= FrictionCoherenceGates::kMinFace4AttemptedCount) {
            fail(out.face4.telemetry.face4FallbackToBlock2 == 0,
                 "face4 fallback route: expected fallback-to-block2 on ill-conditioned manifold");
        }
    }

    const auto failIfWorse = [&](double baseline,
                                 double ablated,
                                 double maxIncrease,
                                 const std::string& label) {
        const double delta = ablated - baseline;
        if (delta > maxIncrease) {
            std::ostringstream msg;
            msg << std::fixed << std::setprecision(6)
                << label << " regression: baseline=" << baseline
                << " ablated=" << ablated
                << " delta=" << delta
                << " threshold=" << maxIncrease;
            fail(true, msg.str());
        }
    };
    if (source.evaluateFrictionAblations) {
        failIfWorse(out.block.restingDriftDistance, out.noBudget.restingDriftDistance, 0.50, "budget_off/resting_drift");
        failIfWorse(out.block.manifoldTangentImpulseContinuity, out.noBudget.manifoldTangentImpulseContinuity, 0.45, "budget_off/manifold_impulse_continuity");
        failIfWorse(out.block.finalSlipSpeed, out.noBudget.finalSlipSpeed, 0.35, "budget_off/final_slip_speed");
        failIfWorse(out.block.restingDriftDistance, out.oneAxisFriction.restingDriftDistance, 0.60, "one_axis/resting_drift");
        failIfWorse(out.block.manifoldTangentImpulseContinuity, out.oneAxisFriction.manifoldTangentImpulseContinuity, 0.55, "one_axis/manifold_impulse_continuity");
        failIfWorse(out.block.finalSlipSpeed, out.oneAxisFriction.finalSlipSpeed, 0.40, "one_axis/final_slip_speed");
    }

    return out;
}

SceneConfig BuildTwoPointBoxRestingOnPlane() {
    SceneConfig cfg;
    cfg.name = "2-point box resting on plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.55f, 0.20f, 0.35f};
    box.mass = 2.0f;
    box.position = {0.0f, 1.3f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9659258f, 0.0f, 0.0f, 0.2588190f});
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 840;
    return cfg;
}

SceneConfig BuildTiltedBoxSettling() {
    SceneConfig cfg;
    cfg.name = "tilted box settling";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.45f, 0.25f, 0.35f};
    box.mass = 2.4f;
    box.position = {0.15f, 1.6f, -0.08f};
    box.orientation = minphys3d::Normalize(Quat{0.9537169f, 0.1331883f, 0.2317487f, 0.1331883f});
    box.angularVelocity = {0.5f, -0.2f, 0.35f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 920;
    return cfg;
}

SceneConfig BuildSlightlyOffsetBoxStacks() {
    SceneConfig cfg;
    cfg.name = "slightly offset box stacks";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body bottom;
    bottom.shape = ShapeType::Box;
    bottom.halfExtents = {0.45f, 0.25f, 0.45f};
    bottom.mass = 3.0f;
    bottom.position = {0.0f, 0.9f, 0.0f};
    const auto bottomId = cfg.world.CreateBody(bottom);

    Body middle = bottom;
    middle.mass = 2.2f;
    middle.position = {0.05f, 1.55f, -0.03f};
    const auto middleId = cfg.world.CreateBody(middle);

    Body top = bottom;
    top.mass = 1.4f;
    top.position = {0.10f, 2.20f, 0.02f};
    const auto topId = cfg.world.CreateBody(top);

    cfg.trackedDynamicBodies = {bottomId, middleId, topId};
    cfg.steps = 1000;
    return cfg;
}

SceneConfig BuildMinimizedPenetrationStackCase() {
    SceneConfig cfg;
    // Purpose: short stack derived from BuildSlightlyOffsetBoxStacks that keeps
    // the penetration-regression signature while reducing total simulation length.
    cfg.name = "minimized penetration stack case";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body bottom;
    bottom.shape = ShapeType::Box;
    bottom.halfExtents = {0.45f, 0.25f, 0.45f};
    bottom.mass = 3.0f;
    bottom.position = {0.0f, 0.9f, 0.0f};
    const auto bottomId = cfg.world.CreateBody(bottom);

    Body middle = bottom;
    middle.mass = 2.2f;
    middle.position = {0.05f, 1.55f, -0.03f};
    const auto middleId = cfg.world.CreateBody(middle);

    Body top = bottom;
    top.mass = 1.4f;
    top.position = {0.10f, 2.20f, 0.02f};
    const auto topId = cfg.world.CreateBody(top);

    cfg.trackedDynamicBodies = {bottomId, middleId, topId};
    cfg.steps = 520;
    return cfg;
}

SceneConfig BuildSlidingBoxComingToRest() {
    SceneConfig cfg;
    cfg.name = "sliding box coming to rest";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.35f, 0.20f, 0.30f};
    box.mass = 1.8f;
    box.position = {-1.4f, 0.9f, 0.0f};
    box.velocity = {3.5f, 0.0f, 0.35f};
    box.staticFriction = 0.75f;
    box.dynamicFriction = 0.55f;
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 900;
    return cfg;
}

SceneConfig BuildFaceContactSlideToRest() {
    SceneConfig cfg;
    cfg.name = "face-contact slide-to-rest";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.45f, 0.18f, 0.35f};
    box.mass = 2.0f;
    box.position = {-1.2f, 0.95f, 0.0f};
    box.velocity = {2.8f, 0.0f, 0.2f};
    box.staticFriction = 0.85f;
    box.dynamicFriction = 0.60f;
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 960;
    cfg.enforceFrictionCoherenceGates = true;
    cfg.evaluateFrictionAblations = true;
    return cfg;
}

SceneConfig BuildFaceContactMildRocking() {
    SceneConfig cfg;
    cfg.name = "face-contact mild rocking";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.50f, 0.20f, 0.40f};
    box.mass = 2.6f;
    box.position = {0.0f, 0.70f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9961947f, 0.0f, 0.0871557f, 0.0f});
    box.angularVelocity = {0.35f, 0.0f, 0.25f};
    box.velocity = {0.6f, 0.0f, -0.4f};
    box.staticFriction = 0.88f;
    box.dynamicFriction = 0.62f;
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 1080;
    cfg.enforceFrictionCoherenceGates = true;
    cfg.evaluateFrictionAblations = true;
    return cfg;
}

SceneConfig BuildFaceContactStickSlipTransition() {
    SceneConfig cfg;
    cfg.name = "face-contact stick-slip transition";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.42f, 0.18f, 0.30f};
    box.mass = 1.9f;
    box.position = {-0.8f, 0.92f, 0.0f};
    box.velocity = {1.4f, 0.0f, 0.0f};
    box.angularVelocity = {0.0f, 0.18f, 0.0f};
    box.staticFriction = 0.90f;
    box.dynamicFriction = 0.45f;
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 1020;
    cfg.enforceFrictionCoherenceGates = true;
    cfg.evaluateFrictionAblations = true;
    return cfg;
}

SceneConfig BuildFace4IllConditionedFallbackCase() {
    SceneConfig cfg;
    cfg.name = "face4 ill-conditioned fallback case";
    cfg.world = World({0.0f, -9.81f, 0.0f});

    Body support;
    support.shape = ShapeType::Box;
    support.halfExtents = {3.0f, 0.25f, 3.0f};
    support.mass = 0.0f;
    support.position = {0.0f, 0.25f, 0.0f};
    cfg.world.CreateBody(support);

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {2.5f, 0.04f, 0.04f};
    box.mass = 0.75f;
    box.position = {0.0f, 0.58f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9996573f, 0.0f, 0.0261769f, 0.0f});
    box.velocity = {0.1f, 0.0f, 0.0f};
    box.angularVelocity = {0.0f, 0.25f, 0.0f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 780;
    cfg.requireFace4FallbackRoute = true;
    return cfg;
}

SceneConfig BuildManifoldReorderStressCase() {
    SceneConfig cfg;
    cfg.name = "manifold reorder stress case";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.60f, 0.18f, 0.28f};
    box.mass = 2.6f;
    box.position = {0.0f, 1.25f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9848077f, 0.0f, 0.1736482f, 0.0f});
    box.velocity = {0.40f, 0.0f, -0.35f};
    box.angularVelocity = {0.0f, 1.0f, 0.0f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 960;
    cfg.logStepContactCount = false;
    cfg.logStepManifoldIds = false;
    return cfg;
}

SceneConfig BuildMinimizedContactVarianceReorderCase() {
    SceneConfig cfg;
    // Purpose: compact manifold-reorder case for contact-cardinality variance.
    // If this fails, prioritize contact-count stddev/reordering diagnostics.
    cfg.name = "minimized contact variance reorder case";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.60f, 0.18f, 0.28f};
    box.mass = 2.6f;
    box.position = {0.0f, 1.25f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9848077f, 0.0f, 0.1736482f, 0.0f});
    box.velocity = {0.40f, 0.0f, -0.35f};
    box.angularVelocity = {0.0f, 1.0f, 0.0f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 480;
    cfg.logStepContactCount = false;
    cfg.logStepManifoldIds = false;
    return cfg;
}

std::string ToJson(const std::vector<ComparisonResult>& results) {
    const auto writeTelemetryJson = [&](std::ostringstream& out, const RunMetrics::SolverTelemetrySnapshot& t) {
        out << "        \"telemetry\": {\n";
        out << "          \"blockSolveEligible\": " << t.blockSolveEligible << ",\n";
        out << "          \"blockSolveUsed\": " << t.blockSolveUsed << ",\n";
        out << "          \"scalarFallbackBuckets\": {\n";
        out << "            \"scalarPathIneligible\": " << t.scalarPathIneligible << ",\n";
        out << "            \"persistenceGate\": " << t.scalarFallbackPersistenceGate << ",\n";
        out << "            \"invalidNormal\": " << t.scalarFallbackInvalidNormal << ",\n";
        out << "            \"normalMismatch\": " << t.scalarFallbackNormalMismatch << ",\n";
        out << "            \"missingSlots\": " << t.scalarFallbackMissingSlots << ",\n";
        out << "            \"degenerateSystem\": " << t.scalarFallbackDegenerateSystem << ",\n";
        out << "            \"conditionEstimate\": " << t.scalarFallbackConditionEstimate << ",\n";
        out << "            \"lcpFailure\": " << t.scalarFallbackLcpFailure << ",\n";
        out << "            \"nonFinite\": " << t.scalarFallbackNonFinite << "\n";
        out << "          },\n";
        out << "          \"topologyChangeEvents\": " << t.topologyChangeEvents << ",\n";
        out << "          \"featureIdChurnEvents\": " << t.featureIdChurnEvents << ",\n";
        out << "          \"impulseResetPoints\": " << t.impulseResetPoints << ",\n";
        out << "          \"reorderDetected\": " << t.reorderDetected << ",\n";
        out << "          \"tangentBasisResets\": " << t.tangentBasisResets << ",\n";
        out << "          \"tangentBasisReused\": " << t.tangentBasisReused << ",\n";
        out << "          \"tangentImpulseReprojected\": " << t.tangentImpulseReprojected << ",\n";
        out << "          \"tangentImpulseReset\": " << t.tangentImpulseReset << ",\n";
        out << "          \"manifoldFrictionBudgetSaturated\": " << t.manifoldFrictionBudgetSaturated << ",\n";
        out << "          \"face4\": {\n";
        out << "            \"attempted\": " << t.face4Attempted << ",\n";
        out << "            \"used\": " << t.face4Used << ",\n";
        out << "            \"fallback_to_block2\": " << t.face4FallbackToBlock2 << ",\n";
        out << "            \"fallback_to_scalar\": " << t.face4FallbackToScalar << ",\n";
        out << "            \"blocked_by_friction_coherence_gate\": " << t.face4BlockedByFrictionCoherenceGate << "\n";
        out << "          },\n";
        out << "          \"budgetSaturationByPolicy\": {\n";
        out << "            \"selected_block_pair_only\": " << t.manifoldFrictionBudgetSaturatedSelectedPair << ",\n";
        out << "            \"all_manifold_contacts\": " << t.manifoldFrictionBudgetSaturatedAllContacts << ",\n";
        out << "            \"blended_selected_pair_and_manifold\": " << t.manifoldFrictionBudgetSaturatedBlended << "\n";
        out << "          },\n";
        out << "          \"manifold_solve_scope\": {\n";
        out << "            \"manifold_contact_count\": " << t.manifoldSolveScope.manifoldContactCount << ",\n";
        out << "            \"selected_block_size\": " << t.manifoldSolveScope.selectedBlockSize << ",\n";
        out << "            \"block_used\": " << t.manifoldSolveScope.blockUsed << ",\n";
        out << "            \"fallback_used\": " << t.manifoldSolveScope.fallbackUsed << ",\n";
        out << "            \"fallback_reason\": {\n";
        out << "              \"none\": " << t.manifoldSolveScope.fallbackReason.none << ",\n";
        out << "              \"ineligible\": " << t.manifoldSolveScope.fallbackReason.ineligible << ",\n";
        out << "              \"persistence_gate\": " << t.manifoldSolveScope.fallbackReason.persistenceGate << ",\n";
        out << "              \"invalid_manifold_normal\": " << t.manifoldSolveScope.fallbackReason.invalidManifoldNormal << ",\n";
        out << "              \"contact_normal_mismatch\": " << t.manifoldSolveScope.fallbackReason.contactNormalMismatch << ",\n";
        out << "              \"missing_block_slots\": " << t.manifoldSolveScope.fallbackReason.missingBlockSlots << ",\n";
        out << "              \"degenerate_mass_matrix\": " << t.manifoldSolveScope.fallbackReason.degenerateMassMatrix << ",\n";
        out << "              \"condition_estimate_exceeded\": " << t.manifoldSolveScope.fallbackReason.conditionEstimateExceeded << ",\n";
        out << "              \"lcp_failure\": " << t.manifoldSolveScope.fallbackReason.lcpFailure << ",\n";
        out << "              \"non_finite_result\": " << t.manifoldSolveScope.fallbackReason.nonFiniteResult << "\n";
        out << "            },\n";
        out << "            \"determinant_or_condition_estimate_sum\": " << t.manifoldSolveScope.determinantOrConditionEstimate << ",\n";
        out << "            \"determinant_or_condition_estimate_samples\": " << t.manifoldSolveScope.determinantOrConditionEstimateSamples << ",\n";
        out << "            \"impulse_continuity_metric_sum\": " << t.manifoldSolveScope.impulseContinuityMetric << ",\n";
        out << "            \"impulse_continuity_metric_samples\": " << t.manifoldSolveScope.impulseContinuityMetricSamples << "\n";
        out << "          },\n";
        out << "          \"manifold_type_scope\": {";
        if (t.manifoldTypeScope.empty()) {
            out << "}\n";
        } else {
            out << "\n";
            std::vector<std::uint8_t> keys;
            keys.reserve(t.manifoldTypeScope.size());
            for (const auto& [key, _] : t.manifoldTypeScope) keys.push_back(key);
            std::sort(keys.begin(), keys.end());
            for (std::size_t i = 0; i < keys.size(); ++i) {
                const auto& b = t.manifoldTypeScope.at(keys[i]);
                out << "            \"" << static_cast<unsigned>(keys[i]) << "\": {"
                    << "\"manifold_contact_count\": " << b.manifoldContactCount
                    << ", \"selected_block_size\": " << b.selectedBlockSize
                    << ", \"block_used\": " << b.blockUsed
                    << ", \"fallback_used\": " << b.fallbackUsed
                    << ", \"determinant_or_condition_estimate_sum\": " << b.determinantOrConditionEstimate
                    << ", \"impulse_continuity_metric_sum\": " << b.impulseContinuityMetric
                    << "}" << (i + 1 == keys.size() ? "\n" : ",\n");
            }
            out << "          }\n";
        }
        out << "        }";
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n  \"suite\": \"block_solver_regression\",\n  \"scenes\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const ComparisonResult& r = results[i];
        out << "    {\n";
        out << "      \"name\": \"" << r.scene << "\",\n";
        out << "      \"pass\": " << (r.pass ? "true" : "false") << ",\n";
        out << "      \"readiness_pass\": " << (r.readinessPass ? "true" : "false") << ",\n";
        out << "      \"scalar\": {\n";
        out << "        \"max_penetration\": " << r.scalar.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.scalar.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.scalar.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.scalar.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.scalar.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.scalar.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.scalar.reorderEvents << ",\n";
        out << "        \"tangent_basis_churn_ratio\": " << r.scalar.tangentBasisChurnRatio << ",\n";
        out << "        \"manifold_tangent_impulse_continuity\": " << r.scalar.manifoldTangentImpulseContinuity << ",\n";
        out << "        \"slip_velocity_decay_ratio\": " << r.scalar.slipVelocityDecayRatio << ",\n";
        out << "        \"final_slip_speed\": " << r.scalar.finalSlipSpeed << ",\n";
        out << "        \"resting_drift_distance\": " << r.scalar.restingDriftDistance << ",\n";
        out << "        \"mean_stable_type9_contacts\": " << r.scalar.meanStableType9Contacts << ",\n";
        writeTelemetryJson(out, r.scalar.telemetry);
        out << "\n";
        out << "      },\n";
        out << "      \"block\": {\n";
        out << "        \"max_penetration\": " << r.block.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.block.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.block.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.block.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.block.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.block.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.block.reorderEvents << ",\n";
        out << "        \"tangent_basis_churn_ratio\": " << r.block.tangentBasisChurnRatio << ",\n";
        out << "        \"manifold_tangent_impulse_continuity\": " << r.block.manifoldTangentImpulseContinuity << ",\n";
        out << "        \"slip_velocity_decay_ratio\": " << r.block.slipVelocityDecayRatio << ",\n";
        out << "        \"final_slip_speed\": " << r.block.finalSlipSpeed << ",\n";
        out << "        \"resting_drift_distance\": " << r.block.restingDriftDistance << ",\n";
        out << "        \"mean_stable_type9_contacts\": " << r.block.meanStableType9Contacts << ",\n";
        writeTelemetryJson(out, r.block.telemetry);
        out << "\n";
        out << "      },\n";
        out << "      \"face4\": {\n";
        out << "        \"max_penetration\": " << r.face4.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.face4.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.face4.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.face4.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.face4.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.face4.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.face4.reorderEvents << ",\n";
        out << "        \"tangent_basis_churn_ratio\": " << r.face4.tangentBasisChurnRatio << ",\n";
        out << "        \"manifold_tangent_impulse_continuity\": " << r.face4.manifoldTangentImpulseContinuity << ",\n";
        out << "        \"slip_velocity_decay_ratio\": " << r.face4.slipVelocityDecayRatio << ",\n";
        out << "        \"final_slip_speed\": " << r.face4.finalSlipSpeed << ",\n";
        out << "        \"resting_drift_distance\": " << r.face4.restingDriftDistance << ",\n";
        out << "        \"mean_stable_type9_contacts\": " << r.face4.meanStableType9Contacts << ",\n";
        writeTelemetryJson(out, r.face4.telemetry);
        out << "\n";
        out << "      },\n";
        out << "      \"ablation\": {\n";
        out << "        \"manifold_budget_off\": {\n";
        out << "          \"manifold_tangent_impulse_continuity\": " << r.noBudget.manifoldTangentImpulseContinuity << ",\n";
        out << "          \"slip_velocity_decay_ratio\": " << r.noBudget.slipVelocityDecayRatio << ",\n";
        out << "          \"final_slip_speed\": " << r.noBudget.finalSlipSpeed << ",\n";
        out << "          \"resting_drift_distance\": " << r.noBudget.restingDriftDistance << "\n";
        out << "        },\n";
        out << "        \"two_axis_friction_off\": {\n";
        out << "          \"manifold_tangent_impulse_continuity\": " << r.oneAxisFriction.manifoldTangentImpulseContinuity << ",\n";
        out << "          \"slip_velocity_decay_ratio\": " << r.oneAxisFriction.slipVelocityDecayRatio << ",\n";
        out << "          \"final_slip_speed\": " << r.oneAxisFriction.finalSlipSpeed << ",\n";
        out << "          \"resting_drift_distance\": " << r.oneAxisFriction.restingDriftDistance << "\n";
        out << "        }\n";
        out << "      },\n";
        out << "      \"failures\": [";
        for (std::size_t fi = 0; fi < r.failures.size(); ++fi) {
            if (fi > 0) out << ", ";
            out << "\"" << r.failures[fi] << "\"";
        }
        out << "]\n";
        out << "    }" << (i + 1 == results.size() ? "\n" : ",\n");
    }
    SolverVariantConfig baselineVariant;
    baselineVariant.useBlockSolver = false;
    const ContactSolverConfig cfg = MakeSolverConfig(baselineVariant);
    const bool readinessPassAll = std::all_of(results.begin(), results.end(), [](const ComparisonResult& r) {
        return r.readinessPass;
    });
    out << "  ],\n";
    out << "  \"readiness_pass\": " << (readinessPassAll ? "true" : "false") << ",\n";
    out << "  \"solver_config\": {\n";
    out << "    \"use_split_impulse\": " << (cfg.useSplitImpulse ? "true" : "false") << ",\n";
    out << "    \"split_impulse_correction_factor\": " << cfg.splitImpulseCorrectionFactor << ",\n";
    out << "    \"penetration_bias_factor\": " << cfg.penetrationBiasFactor << ",\n";
    out << "    \"penetration_slop\": " << cfg.penetrationSlop << ",\n";
    out << "    \"restitution_velocity_cutoff\": " << cfg.restitutionVelocityCutoff << ",\n";
    out << "    \"static_friction_speed_threshold\": " << cfg.staticFrictionSpeedThreshold << ",\n";
    out << "    \"static_to_dynamic_transition_speed\": " << cfg.staticToDynamicTransitionSpeed << ",\n";
    out << "    \"enable_two_axis_friction_solve\": " << (cfg.enableTwoAxisFrictionSolve ? "true" : "false") << ",\n";
    out << "    \"enable_manifold_friction_budget\": " << (cfg.enableManifoldFrictionBudget ? "true" : "false") << ",\n";
    out << "    \"friction_budget_normal_support_source\": " << static_cast<unsigned>(cfg.frictionBudgetNormalSupportSource) << ",\n";
    out << "    \"friction_budget_selected_pair_blend_weight\": " << cfg.frictionBudgetSelectedPairBlendWeight << ",\n";
    out << "    \"face4_require_friction_coherence\": " << (cfg.face4RequireFrictionCoherence ? "true" : "false") << ",\n";
    out << "    \"use_face4_point_normal_block\": " << (cfg.useFace4PointNormalBlock ? "true" : "false") << "\n";
    out << "  }\n}\n";
    return out.str();
}

void PrintHumanSummary(const ComparisonResult& result) {
    const auto totalScalarFallbacks = [](const RunMetrics::SolverTelemetrySnapshot& t) {
        return t.scalarPathIneligible
             + t.scalarFallbackPersistenceGate
             + t.scalarFallbackInvalidNormal
             + t.scalarFallbackNormalMismatch
             + t.scalarFallbackMissingSlots
             + t.scalarFallbackDegenerateSystem
             + t.scalarFallbackConditionEstimate
             + t.scalarFallbackLcpFailure
             + t.scalarFallbackNonFinite;
    };

    const auto printTelemetry = [&](const char* label, const RunMetrics::SolverTelemetrySnapshot& t) {
        std::cout << "    telemetry(" << label << "):"
                  << " blockSolveEligible=" << t.blockSolveEligible
                  << " blockSolveUsed=" << t.blockSolveUsed
                  << " scalarFallbackTotal=" << totalScalarFallbacks(t)
                  << " scalarPathIneligible=" << t.scalarPathIneligible
                  << " persistenceGate=" << t.scalarFallbackPersistenceGate
                  << " invalidNormal=" << t.scalarFallbackInvalidNormal
                  << " normalMismatch=" << t.scalarFallbackNormalMismatch
                  << " missingSlots=" << t.scalarFallbackMissingSlots
                  << " degenerateSystem=" << t.scalarFallbackDegenerateSystem
                  << " conditionEstimate=" << t.scalarFallbackConditionEstimate
                  << " lcpFailure=" << t.scalarFallbackLcpFailure
                  << " nonFinite=" << t.scalarFallbackNonFinite
                  << " face4Attempted=" << t.face4Attempted
                  << " face4Used=" << t.face4Used
                  << " face4FallbackToBlock2=" << t.face4FallbackToBlock2
                  << " face4FallbackToScalar=" << t.face4FallbackToScalar
                  << " face4BlockedByGate=" << t.face4BlockedByFrictionCoherenceGate
                  << " topologyChangeEvents=" << t.topologyChangeEvents
                  << " featureIdChurnEvents=" << t.featureIdChurnEvents
                  << " impulseResetPoints=" << t.impulseResetPoints
                  << " reorderDetected=" << t.reorderDetected
                  << "\n";
    };

    const auto printRun = [&](const char* label, const RunMetrics& m) {
        std::cout << "  " << label
                  << " | pen=" << std::fixed << std::setprecision(4) << m.maxPenetration
                  << " contact_stddev=" << m.contactCountStdDev
                  << " contact_dstep=" << m.contactCountMeanStepDelta
                  << " impulse_max_d=" << m.maxImpulseDelta
                  << " impulse_mean_d=" << m.meanImpulseDelta
                  << " settle_s=" << m.settleTimeSeconds
                  << " reorder=" << m.reorderEvents
                  << " churn=" << m.tangentBasisChurnRatio
                  << " tan_cont=" << m.manifoldTangentImpulseContinuity
                  << " slip_decay=" << m.slipVelocityDecayRatio
                  << " drift=" << m.restingDriftDistance
                  << " stable_type9=" << m.meanStableType9Contacts << "\n";
    };

    std::cout << (result.pass ? "PASS" : "FAIL") << " | " << result.scene << "\n";
    printRun("scalar", result.scalar);
    printRun("block ", result.block);
    printRun("face4 ", result.face4);
    printTelemetry("scalar", result.scalar.telemetry);
    printTelemetry("block ", result.block.telemetry);
    printTelemetry("face4 ", result.face4.telemetry);
    for (const std::string& failure : result.failures) {
        std::cout << "    - " << failure << "\n";
    }

    const std::uint64_t scalarFallbacks = totalScalarFallbacks(result.scalar.telemetry);
    const std::uint64_t blockFallbacks = totalScalarFallbacks(result.block.telemetry);
    if (blockFallbacks > scalarFallbacks) {
        std::cout << "    diagnosis: block path is hitting scalar fallback more often (possible fallback thrash)\n";
    }
    if (result.block.telemetry.impulseResetPoints > result.scalar.telemetry.impulseResetPoints) {
        std::cout << "    diagnosis: block path has more impulse reset points (possible persistence reset)\n";
    }
    if (result.block.telemetry.topologyChangeEvents > result.scalar.telemetry.topologyChangeEvents
        || result.block.telemetry.featureIdChurnEvents > result.scalar.telemetry.featureIdChurnEvents) {
        std::cout << "    diagnosis: block path shows more topology/churn events (possible topology churn)\n";
    }
    if (result.scene.find("face-contact") != std::string::npos || result.scene.find("face4") != std::string::npos) {
        const auto type9It = result.block.telemetry.manifoldTypeScope.find(9);
        const auto type9Scope = type9It != result.block.telemetry.manifoldTypeScope.end()
            ? type9It->second
            : RunMetrics::SolverTelemetrySnapshot::ManifoldSolveScopeSnapshot{};
        const double type9FallbackRate = SafeRatio(static_cast<double>(type9Scope.fallbackUsed), static_cast<double>(type9Scope.solveCount));
        const double face4UseRatio = SafeRatio(
            static_cast<double>(result.face4.telemetry.face4Used),
            static_cast<double>(result.face4.telemetry.face4Attempted));
        std::cout << "    face4 readiness report:"
                  << " basis_churn_ratio=" << result.block.tangentBasisChurnRatio
                  << " stable_type9_contact_support=" << result.block.meanStableType9Contacts
                  << " type9_fallback_rate=" << type9FallbackRate
                  << " face4_used_attempted_ratio=" << face4UseRatio
                  << " readiness_pass=" << (result.readinessPass ? "true" : "false")
                  << "\n";
    }

    if (result.scene == "slightly offset box stacks" || result.scene == "minimized penetration stack case") {
        std::vector<std::pair<int, float>> divergingFrames;
        const std::size_t frameCount = std::min(result.scalar.stepMaxPenetration.size(), result.block.stepMaxPenetration.size());
        for (std::size_t i = 0; i < frameCount; ++i) {
            const float delta = result.block.stepMaxPenetration[i] - result.scalar.stepMaxPenetration[i];
            if (delta > 0.20f) {
                divergingFrames.emplace_back(static_cast<int>(i), delta);
            }
        }
        std::sort(divergingFrames.begin(), divergingFrames.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second > rhs.second;
        });
        std::cout << "    penetration_trace(block-pen - scalar-pen > 0.20):";
        if (divergingFrames.empty()) {
            std::cout << " none\n";
        } else {
            const std::size_t limit = std::min<std::size_t>(12, divergingFrames.size());
            for (std::size_t i = 0; i < limit; ++i) {
                std::cout << " [step=" << divergingFrames[i].first
                          << " delta=" << std::fixed << std::setprecision(4) << divergingFrames[i].second << "]";
            }
            std::cout << "\n";
        }
    }

    if (result.scene == "manifold reorder stress case" || result.scene == "minimized contact variance reorder case") {
        std::vector<std::pair<int, float>> contactDeltaFrames;
        const std::size_t frameCount = std::min(result.scalar.stepContactCounts.size(), result.block.stepContactCounts.size());
        for (std::size_t i = 0; i < frameCount; ++i) {
            const float delta = result.block.stepContactCounts[i] - result.scalar.stepContactCounts[i];
            if (std::abs(delta) >= 1.0f) {
                contactDeltaFrames.emplace_back(static_cast<int>(i), delta);
            }
        }
        std::sort(contactDeltaFrames.begin(), contactDeltaFrames.end(), [](const auto& lhs, const auto& rhs) {
            return std::abs(lhs.second) > std::abs(rhs.second);
        });
        std::cout << "    contact_count_trace(|block-scalar| >= 1):";
        if (contactDeltaFrames.empty()) {
            std::cout << " none\n";
        } else {
            const std::size_t limit = std::min<std::size_t>(10, contactDeltaFrames.size());
            for (std::size_t i = 0; i < limit; ++i) {
                std::cout << " [step=" << contactDeltaFrames[i].first
                          << " delta=" << std::fixed << std::setprecision(1) << contactDeltaFrames[i].second << "]";
            }
            std::cout << "\n";
        }
    }
}

void PrintBlockSolverSafetyRails() {
    std::cout << "block solver safety rails:\n"
              << "  penetration: abs<=" << BlockSolverSafetyRails::kMaxAbsolutePenetration
              << ", regression<=" << BlockSolverSafetyRails::kMaxPenetrationRegression << "\n"
              << "  jitter/contact variance: stddev regression<=" << BlockSolverSafetyRails::kMaxContactStdDevRegression
              << ", step-delta regression<=" << BlockSolverSafetyRails::kMaxContactStepDeltaRegression << "\n"
              << "  fallback rate: regression<=" << BlockSolverSafetyRails::kMaxFallbackRateRegression << "\n"
              << "  settle time: regression<=" << BlockSolverSafetyRails::kMaxSettleTimeRegressionSeconds << " s\n"
              << "  impulse continuity: max-delta regression<=" << BlockSolverSafetyRails::kMaxImpulseDeltaRegression
              << ", mean-delta regression<=" << BlockSolverSafetyRails::kMaxMeanImpulseDeltaRegression
              << ", telemetry regression<=" << BlockSolverSafetyRails::kMaxTelemetryImpulseContinuityRegression << "\n"
              << "  face4 friction coherence gates(type-9 heavy scenes): basis_churn<=" << FrictionCoherenceGates::kMaxBasisChurnRatio
              << ", stable_type9_contacts>=" << FrictionCoherenceGates::kMinStableType9Contacts
              << ", type9_fallback_rate<=" << FrictionCoherenceGates::kMaxType9FallbackRate
              << ", type9_solve_count>=" << FrictionCoherenceGates::kMinType9SolveCount
              << ", face4_attempted>=" << FrictionCoherenceGates::kMinFace4AttemptedCount
              << ", face4_fallback_to_scalar_rate<=" << FrictionCoherenceGates::kMaxFace4FallbackToScalarRate << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string metricsPath = "tests/block_solver_metrics.json";
    bool logReorderContactCounts = false;
    bool logReorderManifoldIds = false;
    bool printHumanSummary = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--metrics-out" && i + 1 < argc) {
            metricsPath = argv[++i];
        } else if (arg == "--log-reorder-contact-count") {
            logReorderContactCounts = true;
        } else if (arg == "--log-reorder-manifold-ids") {
            logReorderManifoldIds = true;
        } else if (arg == "--print-human-summary") {
            printHumanSummary = true;
        }
    }

    std::vector<SceneConfig> scenes;
    SceneConfig reorderStress = BuildManifoldReorderStressCase();
    reorderStress.logStepContactCount = logReorderContactCounts;
    reorderStress.logStepManifoldIds = logReorderManifoldIds;
    scenes.push_back(std::move(reorderStress));
    scenes.push_back(BuildTwoPointBoxRestingOnPlane());
    scenes.push_back(BuildTiltedBoxSettling());
    scenes.push_back(BuildSlightlyOffsetBoxStacks());
    scenes.push_back(BuildMinimizedPenetrationStackCase());
    scenes.push_back(BuildMinimizedContactVarianceReorderCase());
    scenes.push_back(BuildSlidingBoxComingToRest());
    scenes.push_back(BuildFaceContactSlideToRest());
    scenes.push_back(BuildFaceContactMildRocking());
    scenes.push_back(BuildFaceContactStickSlipTransition());
    scenes.push_back(BuildFace4IllConditionedFallbackCase());

    std::vector<ComparisonResult> results;
    results.reserve(scenes.size());
    if (printHumanSummary) {
        PrintBlockSolverSafetyRails();
    }

    bool allPass = true;
    for (const SceneConfig& scene : scenes) {
        ComparisonResult result = CompareScene(scene);
        if (printHumanSummary) {
            PrintHumanSummary(result);
        } else if (!result.pass) {
            std::cout << "FAIL | " << result.scene << "\n";
            for (const std::string& failure : result.failures) {
                std::cout << "  - " << failure << "\n";
            }
        }
        allPass = allPass && result.pass;
        results.push_back(std::move(result));
    }

    const std::string json = ToJson(results);
    std::ofstream out(metricsPath);
    if (out) {
        out << json;
    }
    std::cout << "\nMetrics JSON written to: " << metricsPath << "\n";
    std::cout << "Block solver regression suite: " << (allPass ? "PASS" : "FAIL") << "\n";

    return allPass ? 0 : 1;
}
