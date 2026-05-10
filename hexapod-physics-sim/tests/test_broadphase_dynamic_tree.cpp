#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>

#include "minphys3d/core/world.hpp"

int main() {
    using namespace minphys3d;

    World overlapWorld({0.0, 0.0, 0.0});

    for (int i = 0; i < 12; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0;
        sphere.radius = 0.6;
        sphere.position = {static_cast<float>(i % 4) * 0.9, static_cast<float>(i / 4) * 0.9, 0.0};
        overlapWorld.CreateBody(sphere);
    }

    assert(overlapWorld.BroadphasePairCount() == overlapWorld.BruteForcePairCount());

    World world({0.0, 0.0, 0.0});
    const std::size_t bodyCount = 12;
    for (std::uint32_t i = 0; i < bodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0;
        sphere.radius = 0.25;
        sphere.position = {static_cast<float>(i) * 1.25, 0.0, 0.0};
        world.CreateBody(sphere);
    }
    world.GetBody(0).velocity = {0.05, 0.0, 0.0};
    world.GetBody(1).velocity = {-0.05, 0.0, 0.0};

    world.Step(1.0 / 120.0, 1);

    assert(world.BroadphasePairCount() == world.BruteForcePairCount());
    assert(world.LastBroadphaseMovedProxyCount() <= bodyCount / 2);

    // Randomized stress run: broadphase pair correctness against brute force while
    // collecting quality/perf-oriented broadphase metrics.
    World stressWorld({0.0, 0.0, 0.0});
    BroadphaseConfig broadphaseCfg = stressWorld.GetBroadphaseConfig();
    broadphaseCfg.partialRebuildAreaRatioThreshold = 2.0;
    broadphaseCfg.partialRebuildAvgDepthThreshold = 20.0;
    broadphaseCfg.fullRebuildAreaRatioThreshold = 3.5;
    broadphaseCfg.fullRebuildQueryNodeVisitsPerProxyThreshold = 96.0;
    stressWorld.SetBroadphaseConfig(broadphaseCfg);

    std::mt19937 rng(1337u);
    std::uniform_real_distribution<Real> posDist(-15.0, 15.0);
    std::uniform_real_distribution<Real> velDist(-2.5, 2.5);
    std::uniform_real_distribution<Real> radiusDist(0.15, 0.55);
    constexpr std::size_t stressBodyCount = 64;
    constexpr std::size_t stressSteps = 1200;
    for (std::size_t i = 0; i < stressBodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0;
        sphere.radius = radiusDist(rng);
        sphere.position = {posDist(rng), posDist(rng), posDist(rng)};
        sphere.velocity = {velDist(rng), velDist(rng), velDist(rng)};
        stressWorld.CreateBody(sphere);
    }

    std::uint64_t totalQueryVisits = 0;
    std::uint64_t partialRebuildCount = 0;
    std::uint64_t fullRebuildCount = 0;
    Real maxAreaRatio = 0.0;
    Real maxDepth = 0.0;
    Real maxQueryVisitsPerProxy = 0.0;
    double totalPairGenerationMs = 0.0;
    double totalPairCacheHitRate = 0.0;
    std::uint64_t movedSetOnlyFrames = 0;
    for (std::size_t step = 0; step < stressSteps; ++step) {
        stressWorld.Step(1.0 / 120.0, 2);

        const std::size_t broadphasePairs = stressWorld.BroadphasePairCount();
        const std::size_t bruteForcePairs = stressWorld.BruteForcePairCount();
        assert(broadphasePairs == bruteForcePairs);

        const BroadphaseMetrics& metrics = stressWorld.GetBroadphaseMetrics();
        totalQueryVisits += metrics.queryNodeVisits;
        partialRebuildCount += metrics.partialRebuildTriggered ? 1u : 0u;
        fullRebuildCount += metrics.fullRebuildTriggered ? 1u : 0u;
        maxAreaRatio = std::max(maxAreaRatio, metrics.areaRatio);
        maxDepth = std::max(maxDepth, metrics.maxDepth);
        maxQueryVisitsPerProxy = std::max(maxQueryVisitsPerProxy, metrics.queryNodeVisitsPerProxy);
        totalPairGenerationMs += metrics.pairGenerationMs;
        totalPairCacheHitRate += metrics.pairCacheHitRate;
        movedSetOnlyFrames += metrics.usedMovedSetOnlyUpdate ? 1u : 0u;
    }

    const double avgQueryVisitsPerStep = static_cast<double>(totalQueryVisits) / static_cast<double>(stressSteps);
    std::printf(
        "stress_metrics steps=%zu bodies=%zu avg_query_visits=%.2f max_area_ratio=%.2f max_depth=%.2f max_qvpp=%.2f avg_pair_ms=%.4f avg_cache_hit=%.3f moved_set_only=%llu partial=%llu full=%llu\n",
        stressSteps,
        stressBodyCount,
        avgQueryVisitsPerStep,
        maxAreaRatio,
        maxDepth,
        maxQueryVisitsPerProxy,
        totalPairGenerationMs / static_cast<double>(stressSteps),
        totalPairCacheHitRate / static_cast<double>(stressSteps),
        static_cast<unsigned long long>(movedSetOnlyFrames),
        static_cast<unsigned long long>(partialRebuildCount),
        static_cast<unsigned long long>(fullRebuildCount));

    // Scalability run with mixed sleeping/active/static populations and a high body count.
    World mixedWorld({0.0, 0.0, 0.0});
    BroadphaseConfig mixedCfg = mixedWorld.GetBroadphaseConfig();
    mixedCfg.enableMovedSetOnlyUpdates = true;
    mixedCfg.enablePairCacheReuseForQuasiStatic = true;
    mixedWorld.SetBroadphaseConfig(mixedCfg);

    constexpr std::size_t mixedBodyCount = 2500;
    constexpr std::size_t mixedSteps = 200;
    std::uniform_real_distribution<Real> staticPosDist(-80.0, 80.0);
    std::uniform_real_distribution<Real> activePosDist(-40.0, 40.0);
    std::uniform_real_distribution<Real> activeVelDist(-1.0, 1.0);
    for (std::size_t i = 0; i < mixedBodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.35;
        sphere.position = {staticPosDist(rng), staticPosDist(rng), staticPosDist(rng)};
        if (i % 10 == 0) {
            sphere.mass = 0.0;
            sphere.velocity = {0.0, 0.0, 0.0};
        } else {
            sphere.mass = 1.0;
            sphere.position = {activePosDist(rng), activePosDist(rng), activePosDist(rng)};
            if (i % 7 == 0) {
                sphere.isSleeping = true;
                sphere.velocity = {0.0, 0.0, 0.0};
            } else {
                sphere.velocity = {activeVelDist(rng), activeVelDist(rng), activeVelDist(rng)};
            }
        }
        mixedWorld.CreateBody(sphere);
    }

    std::uint64_t mixedMovedSetOnlyFrames = 0;
    std::uint64_t mixedParityFrames = 0;
    double mixedAvgQueryVisitsPerProxy = 0.0;
    double mixedAvgPairGenerationMs = 0.0;
    double mixedAvgCacheHitRate = 0.0;
    for (std::size_t step = 0; step < mixedSteps; ++step) {
        // Keep most sleeping/static islands unchanged while nudging a small active subset.
        for (std::size_t i = 0; i < mixedBodyCount; ++i) {
            Body& body = mixedWorld.GetBody(static_cast<std::uint32_t>(i));
            if (body.invMass == 0.0 || body.isSleeping) {
                continue;
            }
            if ((i + step) % 23 != 0) {
                body.velocity = {0.0, 0.0, 0.0};
                continue;
            }
            body.velocity = {activeVelDist(rng) * 0.75, 0.0, activeVelDist(rng) * 0.75};
        }
        mixedWorld.Step(1.0 / 120.0, 1);
        const std::size_t broadphasePairs = mixedWorld.BroadphasePairCount();
        const std::size_t bruteForcePairs = mixedWorld.BruteForcePairCount();
        assert(broadphasePairs == bruteForcePairs);
        ++mixedParityFrames;

        const BroadphaseMetrics& metrics = mixedWorld.GetBroadphaseMetrics();
        mixedMovedSetOnlyFrames += metrics.usedMovedSetOnlyUpdate ? 1u : 0u;
        mixedAvgQueryVisitsPerProxy += static_cast<double>(metrics.queryNodeVisitsPerProxy);
        mixedAvgPairGenerationMs += static_cast<double>(metrics.pairGenerationMs);
        mixedAvgCacheHitRate += static_cast<double>(metrics.pairCacheHitRate);
    }
    mixedAvgQueryVisitsPerProxy /= static_cast<double>(mixedSteps);
    mixedAvgPairGenerationMs /= static_cast<double>(mixedSteps);
    mixedAvgCacheHitRate /= static_cast<double>(mixedSteps);

    assert(mixedMovedSetOnlyFrames > 0);
    assert(mixedParityFrames == mixedSteps);
    std::printf(
        "scalability_metrics steps=%zu bodies=%zu moved_set_only=%llu avg_qvpp=%.2f avg_pair_ms=%.4f avg_cache_hit=%.3f\n",
        mixedSteps,
        mixedBodyCount,
        static_cast<unsigned long long>(mixedMovedSetOnlyFrames),
        mixedAvgQueryVisitsPerProxy,
        mixedAvgPairGenerationMs,
        mixedAvgCacheHitRate);

    return 0;
}
