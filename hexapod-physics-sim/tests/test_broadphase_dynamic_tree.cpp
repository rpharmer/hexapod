#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>

#include "minphys3d/core/world.hpp"

int main() {
    using namespace minphys3d;

    World overlapWorld({0.0f, 0.0f, 0.0f});

    for (int i = 0; i < 12; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0f;
        sphere.radius = 0.6f;
        sphere.position = {static_cast<float>(i % 4) * 0.9f, static_cast<float>(i / 4) * 0.9f, 0.0f};
        overlapWorld.CreateBody(sphere);
    }

    assert(overlapWorld.BroadphasePairCount() == overlapWorld.BruteForcePairCount());

    World world({0.0f, 0.0f, 0.0f});
    const std::size_t bodyCount = 12;
    for (std::uint32_t i = 0; i < bodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0f;
        sphere.radius = 0.25f;
        sphere.position = {static_cast<float>(i) * 1.25f, 0.0f, 0.0f};
        world.CreateBody(sphere);
    }
    world.GetBody(0).velocity = {0.05f, 0.0f, 0.0f};
    world.GetBody(1).velocity = {-0.05f, 0.0f, 0.0f};

    world.Step(1.0f / 120.0f, 1);

    assert(world.BroadphasePairCount() == world.BruteForcePairCount());
    assert(world.LastBroadphaseMovedProxyCount() <= bodyCount / 2);

    // Randomized stress run: broadphase pair correctness against brute force while
    // collecting quality/perf-oriented broadphase metrics.
    World stressWorld({0.0f, 0.0f, 0.0f});
    BroadphaseConfig broadphaseCfg = stressWorld.GetBroadphaseConfig();
    broadphaseCfg.partialRebuildAreaRatioThreshold = 2.0f;
    broadphaseCfg.partialRebuildAvgDepthThreshold = 20.0f;
    broadphaseCfg.fullRebuildAreaRatioThreshold = 3.5f;
    broadphaseCfg.fullRebuildQueryNodeVisitsPerProxyThreshold = 96.0f;
    stressWorld.SetBroadphaseConfig(broadphaseCfg);

    std::mt19937 rng(1337u);
    std::uniform_real_distribution<float> posDist(-15.0f, 15.0f);
    std::uniform_real_distribution<float> velDist(-2.5f, 2.5f);
    std::uniform_real_distribution<float> radiusDist(0.15f, 0.55f);
    constexpr std::size_t stressBodyCount = 64;
    constexpr std::size_t stressSteps = 1200;
    for (std::size_t i = 0; i < stressBodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0f;
        sphere.radius = radiusDist(rng);
        sphere.position = {posDist(rng), posDist(rng), posDist(rng)};
        sphere.velocity = {velDist(rng), velDist(rng), velDist(rng)};
        stressWorld.CreateBody(sphere);
    }

    std::uint64_t totalQueryVisits = 0;
    std::uint64_t partialRebuildCount = 0;
    std::uint64_t fullRebuildCount = 0;
    float maxAreaRatio = 0.0f;
    float maxDepth = 0.0f;
    float maxQueryVisitsPerProxy = 0.0f;
    double totalPairGenerationMs = 0.0;
    double totalPairCacheHitRate = 0.0;
    std::uint64_t movedSetOnlyFrames = 0;
    for (std::size_t step = 0; step < stressSteps; ++step) {
        stressWorld.Step(1.0f / 120.0f, 2);

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
    World mixedWorld({0.0f, 0.0f, 0.0f});
    BroadphaseConfig mixedCfg = mixedWorld.GetBroadphaseConfig();
    mixedCfg.enableMovedSetOnlyUpdates = true;
    mixedCfg.enablePairCacheReuseForQuasiStatic = true;
    mixedWorld.SetBroadphaseConfig(mixedCfg);

    constexpr std::size_t mixedBodyCount = 2500;
    constexpr std::size_t mixedSteps = 200;
    std::uniform_real_distribution<float> staticPosDist(-80.0f, 80.0f);
    std::uniform_real_distribution<float> activePosDist(-40.0f, 40.0f);
    std::uniform_real_distribution<float> activeVelDist(-1.0f, 1.0f);
    for (std::size_t i = 0; i < mixedBodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.35f;
        sphere.position = {staticPosDist(rng), staticPosDist(rng), staticPosDist(rng)};
        if (i % 10 == 0) {
            sphere.mass = 0.0f;
            sphere.velocity = {0.0f, 0.0f, 0.0f};
        } else {
            sphere.mass = 1.0f;
            sphere.position = {activePosDist(rng), activePosDist(rng), activePosDist(rng)};
            if (i % 7 == 0) {
                sphere.isSleeping = true;
                sphere.velocity = {0.0f, 0.0f, 0.0f};
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
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            if ((i + step) % 23 != 0) {
                body.velocity = {0.0f, 0.0f, 0.0f};
                continue;
            }
            body.velocity = {activeVelDist(rng) * 0.75f, 0.0f, activeVelDist(rng) * 0.75f};
        }
        mixedWorld.Step(1.0f / 120.0f, 1);
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
