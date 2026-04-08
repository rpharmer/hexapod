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
    }

    const double avgQueryVisitsPerStep = static_cast<double>(totalQueryVisits) / static_cast<double>(stressSteps);
    std::printf(
        "stress_metrics steps=%zu bodies=%zu avg_query_visits=%.2f max_area_ratio=%.2f max_depth=%.2f max_qvpp=%.2f partial=%llu full=%llu\n",
        stressSteps,
        stressBodyCount,
        avgQueryVisitsPerStep,
        maxAreaRatio,
        maxDepth,
        maxQueryVisitsPerProxy,
        static_cast<unsigned long long>(partialRebuildCount),
        static_cast<unsigned long long>(fullRebuildCount));

    return 0;
}
