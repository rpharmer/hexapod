#include <algorithm>
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
    SolverTelemetrySnapshot telemetry;
    std::vector<float> stepMaxPenetration;
    std::vector<float> stepContactCounts;
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
};

struct ComparisonResult {
    std::string scene;
    RunMetrics scalar;
    RunMetrics block;
    bool pass = true;
    std::vector<std::string> failures;
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

ContactSolverConfig MakeSolverConfig(bool useBlockSolver) {
    ContactSolverConfig cfg;
    cfg.useSplitImpulse = true;
    cfg.penetrationSlop = 0.005f;
    cfg.splitImpulseCorrectionFactor = 0.85f;
    cfg.penetrationBiasFactor = 0.05f;
    cfg.restitutionVelocityCutoff = 0.5f;
    cfg.staticFrictionSpeedThreshold = 0.0f;
    cfg.staticToDynamicTransitionSpeed = 0.2f;
    cfg.useBlockSolver = useBlockSolver;
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

RunMetrics RunScene(SceneConfig config, bool useBlockSolver) {
    config.world.SetContactSolverConfig(MakeSolverConfig(useBlockSolver));

    RunMetrics metrics;
    std::vector<float> contactCounts;
    std::vector<float> contactCountStepDeltas;
    std::unordered_map<std::uint64_t, float> lastImpulseByPoint;
    std::uint64_t impulseDeltaCount = 0;
    float impulseDeltaSum = 0.0f;

    float previousContactCount = -1.0f;
    int settledConsecutive = 0;

    for (int step = 0; step < config.steps; ++step) {
        config.world.Step(config.dt, config.solverIterations);

        const std::vector<Manifold>& manifolds = config.world.DebugManifolds();
        float totalContacts = 0.0f;
        float stepMaxPenetration = 0.0f;

        for (const Manifold& manifold : manifolds) {
            std::unordered_map<std::uint64_t, std::uint8_t> ordinalCount;
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
        if (config.logStepContactCount) {
            std::cout << "[manifold-reorder-step] solver=" << (useBlockSolver ? "block" : "scalar")
                      << " step=" << step
                      << " contact_count=" << totalContacts << "\n";
        }
        if (config.logStepManifoldIds) {
            std::cout << "[manifold-reorder-step] solver=" << (useBlockSolver ? "block" : "scalar")
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
    assignBucket(telemetry.manifoldSolveScope, metrics.telemetry.manifoldSolveScope);
    for (const auto& [manifoldType, bucket] : telemetry.manifoldTypeBuckets) {
        assignBucket(bucket, metrics.telemetry.manifoldTypeScope[manifoldType]);
    }
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

    out.scalar = RunScene(source, false);
    out.block = RunScene(source, true);

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

    const bool isMinimizedPenetrationCase = out.scene == "minimized penetration stack case";
    const bool isMinimizedStdDevCase = out.scene == "minimized contact variance reorder case";

    if (!isMinimizedPenetrationCase && !isMinimizedStdDevCase) {
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
        out << "      \"scalar\": {\n";
        out << "        \"max_penetration\": " << r.scalar.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.scalar.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.scalar.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.scalar.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.scalar.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.scalar.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.scalar.reorderEvents << ",\n";
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
        writeTelemetryJson(out, r.block.telemetry);
        out << "\n";
        out << "      },\n";
        out << "      \"failures\": [";
        for (std::size_t fi = 0; fi < r.failures.size(); ++fi) {
            if (fi > 0) out << ", ";
            out << "\"" << r.failures[fi] << "\"";
        }
        out << "]\n";
        out << "    }" << (i + 1 == results.size() ? "\n" : ",\n");
    }
    const ContactSolverConfig cfg = MakeSolverConfig(false);
    out << "  ],\n";
    out << "  \"solver_config\": {\n";
    out << "    \"use_split_impulse\": " << (cfg.useSplitImpulse ? "true" : "false") << ",\n";
    out << "    \"split_impulse_correction_factor\": " << cfg.splitImpulseCorrectionFactor << ",\n";
    out << "    \"penetration_bias_factor\": " << cfg.penetrationBiasFactor << ",\n";
    out << "    \"penetration_slop\": " << cfg.penetrationSlop << ",\n";
    out << "    \"restitution_velocity_cutoff\": " << cfg.restitutionVelocityCutoff << ",\n";
    out << "    \"static_friction_speed_threshold\": " << cfg.staticFrictionSpeedThreshold << ",\n";
    out << "    \"static_to_dynamic_transition_speed\": " << cfg.staticToDynamicTransitionSpeed << "\n";
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
                  << " reorder=" << m.reorderEvents << "\n";
    };

    std::cout << (result.pass ? "PASS" : "FAIL") << " | " << result.scene << "\n";
    printRun("scalar", result.scalar);
    printRun("block ", result.block);
    printTelemetry("scalar", result.scalar.telemetry);
    printTelemetry("block ", result.block.telemetry);
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
              << ", telemetry regression<=" << BlockSolverSafetyRails::kMaxTelemetryImpulseContinuityRegression << "\n";
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
