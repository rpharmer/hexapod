#pragma once

#include <functional>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace minphys3d::core_internal {

struct BroadphaseUpdateContext {
    const std::vector<Body>& bodies;
    std::vector<BroadphaseProxy>& proxies;
    std::vector<std::uint32_t>& movedProxyIds;
    std::vector<std::uint8_t>& previousBodyActiveState;
    BroadphaseMetrics& metrics;
    const BroadphaseConfig& config;
    std::uint64_t& broadphaseStepCounter;
    std::uint64_t& lastBroadphaseRebuildStep;
    std::size_t& lastBroadphaseMovedProxyCount;

    std::function<float(const Body&)> computeProxyMargin;
    std::function<void(std::uint32_t)> ensureProxyInTree;
    std::function<void(std::int32_t)> removeLeaf;
    std::function<std::int32_t(std::uint32_t, const AABB&)> insertLeaf;
    std::function<AABB(const AABB&, float)> expandAabb;
    std::function<bool(const AABB&, const AABB&)> contains;
    std::function<void()> updateBroadphaseQualityMetrics;
    std::function<void()> maybeTriggerBroadphaseRebuild;
};

struct BroadphaseContext {
    std::function<std::vector<Pair>()> computePotentialPairs;
};

class BroadphaseSystem {
public:
    std::vector<Pair> ComputePotentialPairs(const BroadphaseContext& context) const;
    void UpdateProxies(const BroadphaseUpdateContext& context) const;
};

} // namespace minphys3d::core_internal
