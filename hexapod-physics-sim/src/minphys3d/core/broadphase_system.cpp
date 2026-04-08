#include "broadphase_system.hpp"

namespace minphys3d::core_internal {

std::vector<Pair> BroadphaseSystem::ComputePotentialPairs(const BroadphaseContext& context) const {
    return context.computePotentialPairs ? context.computePotentialPairs() : std::vector<Pair>{};
}

void BroadphaseSystem::UpdateProxies(const BroadphaseUpdateContext& context) const {
    if (context.proxies.size() != context.bodies.size()) {
        context.proxies.resize(context.bodies.size());
    }

    ++context.broadphaseStepCounter;
    context.lastBroadphaseMovedProxyCount = 0;
    context.movedProxyIds.clear();
    context.metrics.partialRebuildTriggered = false;
    context.metrics.fullRebuildTriggered = false;

    for (std::size_t i = 0; i < context.bodies.size(); ++i) {
        const Body& body = context.bodies[i];
        const AABB current = body.ComputeAABB();
        const float margin = context.computeProxyMargin(body);
        if (!context.proxies[i].valid) {
            context.proxies[i].fatBox = context.expandAabb(current, margin);
            context.proxies[i].leaf = -1;
            context.proxies[i].valid = true;
            context.ensureProxyInTree(static_cast<std::uint32_t>(i));
        } else if (!context.contains(context.proxies[i].fatBox, current)) {
            if (context.proxies[i].leaf >= 0) {
                context.removeLeaf(context.proxies[i].leaf);
            }
            context.proxies[i].fatBox = context.expandAabb(current, margin);
            context.proxies[i].leaf = context.insertLeaf(static_cast<std::uint32_t>(i), context.proxies[i].fatBox);
            ++context.lastBroadphaseMovedProxyCount;
            context.movedProxyIds.push_back(static_cast<std::uint32_t>(i));
        } else {
            context.ensureProxyInTree(static_cast<std::uint32_t>(i));
        }
    }

    if (context.previousBodyActiveState.size() != context.bodies.size()) {
        context.previousBodyActiveState.assign(context.bodies.size(), 0u);
        for (std::size_t i = 0; i < context.bodies.size(); ++i) {
            const bool active = !(context.bodies[i].invMass == 0.0f || context.bodies[i].isSleeping);
            context.previousBodyActiveState[i] = active ? 1u : 0u;
        }
    } else {
        for (std::size_t i = 0; i < context.bodies.size(); ++i) {
            const bool active = !(context.bodies[i].invMass == 0.0f || context.bodies[i].isSleeping);
            const std::uint8_t current = active ? 1u : 0u;
            if (context.previousBodyActiveState[i] != current) {
                context.movedProxyIds.push_back(static_cast<std::uint32_t>(i));
                context.previousBodyActiveState[i] = current;
            }
        }
    }

    context.updateBroadphaseQualityMetrics();
    context.maybeTriggerBroadphaseRebuild();
}

} // namespace minphys3d::core_internal
