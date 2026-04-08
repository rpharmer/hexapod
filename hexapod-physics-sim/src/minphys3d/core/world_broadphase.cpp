#include "minphys3d/core/world.hpp"
#include "subsystems.hpp"

#include <chrono>

namespace minphys3d {
float World::ComputeProxyMargin(const Body& body) const {
    const float linearSweep = Length(body.velocity) * currentSubstepDt_ * broadphaseConfig_.linearVelocityMarginScale;
    float radiusLike = 0.5f;
    if (body.shape == ShapeType::Sphere) {
        radiusLike = body.radius;
    } else if (body.shape == ShapeType::Box) {
        radiusLike = Length(body.halfExtents);
    } else if (body.shape == ShapeType::Capsule) {
        radiusLike = body.halfHeight + body.radius;
    }
    const float angularSweep = Length(body.angularVelocity) * radiusLike * currentSubstepDt_ * broadphaseConfig_.angularVelocityMarginScale;
    const float sweptMargin = linearSweep + angularSweep;
    const float dynamicMargin = std::clamp(sweptMargin, broadphaseConfig_.minSweptMargin, broadphaseConfig_.maxSweptMargin);
    return std::max(broadphaseConfig_.baseFatAabbMargin, dynamicMargin);
}

void World::UpdateBroadphaseProxies() {
    if (proxies_.size() != bodies_.size()) {
        proxies_.resize(bodies_.size());
    }
    ++broadphaseStepCounter_;
    lastBroadphaseMovedProxyCount_ = 0;
    movedProxyIds_.clear();
    broadphaseMetrics_.partialRebuildTriggered = false;
    broadphaseMetrics_.fullRebuildTriggered = false;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const Body& body = bodies_[i];
        const AABB current = bodies_[i].ComputeAABB();
        const float margin = ComputeProxyMargin(body);
        if (!proxies_[i].valid) {
            proxies_[i].fatBox = ExpandAABB(current, margin);
            proxies_[i].leaf = -1;
            proxies_[i].valid = true;
            EnsureProxyInTree(static_cast<std::uint32_t>(i));
        } else if (!Contains(proxies_[i].fatBox, current)) {
            if (proxies_[i].leaf >= 0) {
                RemoveLeaf(proxies_[i].leaf);
            }
            proxies_[i].fatBox = ExpandAABB(current, margin);
            proxies_[i].leaf = InsertLeaf(static_cast<std::uint32_t>(i), proxies_[i].fatBox);
            ++lastBroadphaseMovedProxyCount_;
            movedProxyIds_.push_back(static_cast<std::uint32_t>(i));
        } else {
            EnsureProxyInTree(static_cast<std::uint32_t>(i));
        }
    }
    if (previousBodyActiveState_.size() != bodies_.size()) {
        previousBodyActiveState_.assign(bodies_.size(), 0u);
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            const bool active = !(bodies_[i].invMass == 0.0f || bodies_[i].isSleeping);
            previousBodyActiveState_[i] = active ? 1u : 0u;
        }
    } else {
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            const bool active = !(bodies_[i].invMass == 0.0f || bodies_[i].isSleeping);
            const std::uint8_t current = active ? 1u : 0u;
            if (previousBodyActiveState_[i] != current) {
                movedProxyIds_.push_back(static_cast<std::uint32_t>(i));
                previousBodyActiveState_[i] = current;
            }
        }
    }
    UpdateBroadphaseQualityMetrics();
    MaybeTriggerBroadphaseRebuild();
}

void World::ReinsertProxy(std::uint32_t bodyId) {
    if (bodyId >= proxies_.size() || !proxies_[bodyId].valid) {
        return;
    }
    if (proxies_[bodyId].leaf >= 0) {
        RemoveLeaf(proxies_[bodyId].leaf);
    }
    proxies_[bodyId].leaf = InsertLeaf(bodyId, proxies_[bodyId].fatBox);
}

void World::PartialRebuildBroadphase() {
    for (const std::uint32_t movedId : movedProxyIds_) {
        ReinsertProxy(movedId);
    }
}

void World::FullRebuildBroadphase() {
    rootNode_ = -1;
    freeNode_ = -1;
    treeNodes_.clear();
    for (BroadphaseProxy& proxy : proxies_) {
        if (!proxy.valid) {
            continue;
        }
        proxy.leaf = -1;
    }
    for (std::uint32_t bodyId = 0; bodyId < proxies_.size(); ++bodyId) {
        EnsureProxyInTree(bodyId);
    }
}

void World::UpdateBroadphaseQualityMetrics() {
    broadphaseMetrics_ = {};
    if (rootNode_ < 0 || proxies_.empty()) {
        return;
    }

    std::uint32_t validProxyCount = 0;
    for (const BroadphaseProxy& proxy : proxies_) {
        if (proxy.valid && proxy.leaf >= 0) {
            ++validProxyCount;
        }
    }
    broadphaseMetrics_.validProxyCount = validProxyCount;
    if (validProxyCount == 0) {
        return;
    }

    float totalArea = 0.0f;
    std::uint32_t maxDepth = 0;
    std::uint32_t leafCount = 0;
    std::uint64_t depthSum = 0;
    std::vector<std::pair<std::int32_t, std::uint32_t>> stack;
    stack.reserve(treeNodes_.size());
    stack.push_back({rootNode_, 0u});
    while (!stack.empty()) {
        const auto [nodeId, depth] = stack.back();
        stack.pop_back();
        if (nodeId < 0) {
            continue;
        }
        const TreeNode& node = treeNodes_[nodeId];
        totalArea += SurfaceArea(node.box);
        if (node.IsLeaf()) {
            ++leafCount;
            depthSum += depth;
            maxDepth = std::max(maxDepth, depth);
            continue;
        }
        stack.push_back({node.left, depth + 1u});
        stack.push_back({node.right, depth + 1u});
    }

    const float rootArea = std::max(SurfaceArea(treeNodes_[rootNode_].box), kEpsilon);
    broadphaseMetrics_.areaRatio = totalArea / rootArea;
    broadphaseMetrics_.maxDepth = static_cast<float>(maxDepth);
    broadphaseMetrics_.avgDepth = (leafCount > 0) ? static_cast<float>(depthSum) / static_cast<float>(leafCount) : 0.0f;
    broadphaseMetrics_.movedProxyRatio = static_cast<float>(lastBroadphaseMovedProxyCount_) / static_cast<float>(validProxyCount);
}

void World::MaybeTriggerBroadphaseRebuild() {
    if (broadphaseStepCounter_ - lastBroadphaseRebuildStep_ < broadphaseConfig_.minStepsBetweenRebuilds) {
        return;
    }
    const bool fullRebuild =
        broadphaseMetrics_.areaRatio >= broadphaseConfig_.fullRebuildAreaRatioThreshold
        || broadphaseMetrics_.maxDepth >= broadphaseConfig_.fullRebuildMaxDepthThreshold
        || broadphaseMetrics_.movedProxyRatio >= broadphaseConfig_.fullRebuildMovedProxyRatioThreshold
        || broadphaseMetrics_.queryNodeVisitsPerProxy >= broadphaseConfig_.fullRebuildQueryNodeVisitsPerProxyThreshold;
    const bool partialRebuild =
        broadphaseMetrics_.areaRatio >= broadphaseConfig_.partialRebuildAreaRatioThreshold
        || broadphaseMetrics_.avgDepth >= broadphaseConfig_.partialRebuildAvgDepthThreshold
        || broadphaseMetrics_.movedProxyRatio >= broadphaseConfig_.partialRebuildMovedProxyRatioThreshold;

    if (fullRebuild) {
        FullRebuildBroadphase();
        broadphaseMetrics_.fullRebuildTriggered = true;
        lastBroadphaseRebuildStep_ = broadphaseStepCounter_;
        UpdateBroadphaseQualityMetrics();
    } else if (partialRebuild) {
        PartialRebuildBroadphase();
        broadphaseMetrics_.partialRebuildTriggered = true;
        lastBroadphaseRebuildStep_ = broadphaseStepCounter_;
        UpdateBroadphaseQualityMetrics();
    }
}

void World::EnsureProxyInTree(std::uint32_t bodyId) {
    if (bodyId >= proxies_.size() || !proxies_[bodyId].valid) {
        return;
    }
    if (proxies_[bodyId].leaf < 0) {
        proxies_[bodyId].leaf = InsertLeaf(bodyId, proxies_[bodyId].fatBox);
    }
}

std::int32_t World::AllocateNode() {
        if (freeNode_ != -1) {
            const std::int32_t nodeId = freeNode_;
            freeNode_ = treeNodes_[nodeId].next;
            treeNodes_[nodeId] = TreeNode{};
            return nodeId;
        }
        treeNodes_.push_back(TreeNode{});
        return static_cast<std::int32_t>(treeNodes_.size() - 1);
    }

std::int32_t World::InsertLeaf(std::uint32_t bodyId, const AABB& fatBox) {
        const std::int32_t leaf = AllocateNode();
        treeNodes_[leaf].box = fatBox;
        treeNodes_[leaf].left = -1;
        treeNodes_[leaf].right = -1;
        treeNodes_[leaf].parent = -1;
        treeNodes_[leaf].bodyId = static_cast<std::int32_t>(bodyId);
        treeNodes_[leaf].height = 0;

        if (rootNode_ == -1) {
            rootNode_ = leaf;
            return leaf;
        }

        std::int32_t index = rootNode_;
        while (!treeNodes_[index].IsLeaf()) {
            const std::int32_t left = treeNodes_[index].left;
            const std::int32_t right = treeNodes_[index].right;
            const float baseCost = 2.0f * SurfaceArea(MergeAABB(treeNodes_[index].box, fatBox));
            const float inheritanceCost = 2.0f * (SurfaceArea(MergeAABB(treeNodes_[index].box, fatBox)) - SurfaceArea(treeNodes_[index].box));

            float leftCost = 0.0f;
            if (treeNodes_[left].IsLeaf()) {
                leftCost = SurfaceArea(MergeAABB(treeNodes_[left].box, fatBox)) + inheritanceCost;
            } else {
                const float oldArea = SurfaceArea(treeNodes_[left].box);
                const float newArea = SurfaceArea(MergeAABB(treeNodes_[left].box, fatBox));
                leftCost = (newArea - oldArea) + inheritanceCost;
            }

            float rightCost = 0.0f;
            if (treeNodes_[right].IsLeaf()) {
                rightCost = SurfaceArea(MergeAABB(treeNodes_[right].box, fatBox)) + inheritanceCost;
            } else {
                const float oldArea = SurfaceArea(treeNodes_[right].box);
                const float newArea = SurfaceArea(MergeAABB(treeNodes_[right].box, fatBox));
                rightCost = (newArea - oldArea) + inheritanceCost;
            }

            if (baseCost < leftCost && baseCost < rightCost) {
                break;
            }
            index = (leftCost < rightCost) ? left : right;
        }

        const std::int32_t sibling = index;
        const std::int32_t oldParent = treeNodes_[sibling].parent;
        const std::int32_t newParent = AllocateNode();
        treeNodes_[newParent].parent = oldParent;
        treeNodes_[newParent].box = MergeAABB(fatBox, treeNodes_[sibling].box);
        treeNodes_[newParent].height = treeNodes_[sibling].height + 1;
        treeNodes_[newParent].left = sibling;
        treeNodes_[newParent].right = leaf;
        treeNodes_[newParent].bodyId = -1;

        treeNodes_[sibling].parent = newParent;
        treeNodes_[leaf].parent = newParent;

        if (oldParent == -1) {
            rootNode_ = newParent;
        } else if (treeNodes_[oldParent].left == sibling) {
            treeNodes_[oldParent].left = newParent;
        } else {
            treeNodes_[oldParent].right = newParent;
        }

        index = treeNodes_[leaf].parent;
        while (index != -1) {
            index = Balance(index);
            const std::int32_t left = treeNodes_[index].left;
            const std::int32_t right = treeNodes_[index].right;
            treeNodes_[index].height = 1 + std::max(treeNodes_[left].height, treeNodes_[right].height);
            treeNodes_[index].box = MergeAABB(treeNodes_[left].box, treeNodes_[right].box);
            index = treeNodes_[index].parent;
        }

        return leaf;
    }

void World::RemoveLeaf(std::int32_t leaf) {
        if (leaf == rootNode_) {
            rootNode_ = -1;
            FreeNode(leaf);
            return;
        }

        const std::int32_t parent = treeNodes_[leaf].parent;
        const std::int32_t grandParent = treeNodes_[parent].parent;
        const std::int32_t sibling = (treeNodes_[parent].left == leaf) ? treeNodes_[parent].right : treeNodes_[parent].left;

        if (grandParent != -1) {
            if (treeNodes_[grandParent].left == parent) {
                treeNodes_[grandParent].left = sibling;
            } else {
                treeNodes_[grandParent].right = sibling;
            }
            treeNodes_[sibling].parent = grandParent;
            FreeNode(parent);

            std::int32_t index = grandParent;
            while (index != -1) {
                index = Balance(index);
                const std::int32_t left = treeNodes_[index].left;
                const std::int32_t right = treeNodes_[index].right;
                treeNodes_[index].box = MergeAABB(treeNodes_[left].box, treeNodes_[right].box);
                treeNodes_[index].height = 1 + std::max(treeNodes_[left].height, treeNodes_[right].height);
                index = treeNodes_[index].parent;
            }
        } else {
            rootNode_ = sibling;
            treeNodes_[sibling].parent = -1;
            FreeNode(parent);
        }

        FreeNode(leaf);
    }

std::int32_t World::Balance(std::int32_t iA) {
        if (iA == -1 || treeNodes_[iA].IsLeaf() || treeNodes_[iA].height < 2) {
            return iA;
        }

        const std::int32_t iB = treeNodes_[iA].left;
        const std::int32_t iC = treeNodes_[iA].right;
        const int balance = treeNodes_[iC].height - treeNodes_[iB].height;

        if (balance > 1) {
            const std::int32_t iF = treeNodes_[iC].left;
            const std::int32_t iG = treeNodes_[iC].right;
            treeNodes_[iC].left = iA;
            treeNodes_[iC].parent = treeNodes_[iA].parent;
            treeNodes_[iA].parent = iC;

            if (treeNodes_[iC].parent != -1) {
                if (treeNodes_[treeNodes_[iC].parent].left == iA) treeNodes_[treeNodes_[iC].parent].left = iC;
                else treeNodes_[treeNodes_[iC].parent].right = iC;
            } else {
                rootNode_ = iC;
            }

            if (treeNodes_[iF].height > treeNodes_[iG].height) {
                treeNodes_[iC].right = iF;
                treeNodes_[iA].right = iG;
                treeNodes_[iG].parent = iA;
                treeNodes_[iA].box = MergeAABB(treeNodes_[iB].box, treeNodes_[iG].box);
                treeNodes_[iC].box = MergeAABB(treeNodes_[iA].box, treeNodes_[iF].box);
                treeNodes_[iA].height = 1 + std::max(treeNodes_[iB].height, treeNodes_[iG].height);
                treeNodes_[iC].height = 1 + std::max(treeNodes_[iA].height, treeNodes_[iF].height);
            } else {
                treeNodes_[iC].right = iG;
                treeNodes_[iA].right = iF;
                treeNodes_[iF].parent = iA;
                treeNodes_[iA].box = MergeAABB(treeNodes_[iB].box, treeNodes_[iF].box);
                treeNodes_[iC].box = MergeAABB(treeNodes_[iA].box, treeNodes_[iG].box);
                treeNodes_[iA].height = 1 + std::max(treeNodes_[iB].height, treeNodes_[iF].height);
                treeNodes_[iC].height = 1 + std::max(treeNodes_[iA].height, treeNodes_[iG].height);
            }
            return iC;
        }

        if (balance < -1) {
            const std::int32_t iD = treeNodes_[iB].left;
            const std::int32_t iE = treeNodes_[iB].right;
            treeNodes_[iB].left = iA;
            treeNodes_[iB].parent = treeNodes_[iA].parent;
            treeNodes_[iA].parent = iB;

            if (treeNodes_[iB].parent != -1) {
                if (treeNodes_[treeNodes_[iB].parent].left == iA) treeNodes_[treeNodes_[iB].parent].left = iB;
                else treeNodes_[treeNodes_[iB].parent].right = iB;
            } else {
                rootNode_ = iB;
            }

            if (treeNodes_[iD].height > treeNodes_[iE].height) {
                treeNodes_[iB].right = iD;
                treeNodes_[iA].left = iE;
                treeNodes_[iE].parent = iA;
                treeNodes_[iA].box = MergeAABB(treeNodes_[iC].box, treeNodes_[iE].box);
                treeNodes_[iB].box = MergeAABB(treeNodes_[iA].box, treeNodes_[iD].box);
                treeNodes_[iA].height = 1 + std::max(treeNodes_[iC].height, treeNodes_[iE].height);
                treeNodes_[iB].height = 1 + std::max(treeNodes_[iA].height, treeNodes_[iD].height);
            } else {
                treeNodes_[iB].right = iE;
                treeNodes_[iA].left = iD;
                treeNodes_[iD].parent = iA;
                treeNodes_[iA].box = MergeAABB(treeNodes_[iC].box, treeNodes_[iD].box);
                treeNodes_[iB].box = MergeAABB(treeNodes_[iA].box, treeNodes_[iE].box);
                treeNodes_[iA].height = 1 + std::max(treeNodes_[iC].height, treeNodes_[iD].height);
                treeNodes_[iB].height = 1 + std::max(treeNodes_[iA].height, treeNodes_[iE].height);
            }
            return iB;
        }

        return iA;
    }

std::vector<Pair> World::ComputePotentialPairs() const {
    const auto startTime = std::chrono::steady_clock::now();
    std::vector<Pair> pairs;
    broadphaseMetrics_.queryNodeVisits = 0;
    broadphaseMetrics_.queryNodeVisitsPerProxy = 0.0f;
    broadphaseMetrics_.pairGenerationMs = 0.0f;
    broadphaseMetrics_.pairCacheHitRate = 0.0f;
    broadphaseMetrics_.pairCacheQueries = 0;
    broadphaseMetrics_.pairCacheHits = 0;
    broadphaseMetrics_.usedMovedSetOnlyUpdate = false;
    if (bodies_.empty() || rootNode_ == -1) {
        return pairs;
    }

    std::unordered_set<std::uint32_t> movedSet;
    movedSet.reserve(movedProxyIds_.size() * 2);
    for (const std::uint32_t movedId : movedProxyIds_) {
        if (movedId < proxies_.size()) {
            movedSet.insert(movedId);
        }
    }

    std::unordered_set<std::uint64_t> seenPairs;
    seenPairs.reserve(std::max<std::size_t>(bodies_.size() * 4, cachedPotentialPairKeySet_.size() * 2));
    if (broadphaseConfig_.enablePairCacheReuseForQuasiStatic && !cachedPotentialPairs_.empty()) {
        for (const Pair& cachedPair : cachedPotentialPairs_) {
            ++broadphaseMetrics_.pairCacheQueries;
            if (!IsPairEligible(cachedPair.a, cachedPair.b)) {
                continue;
            }
            if (broadphaseConfig_.enableMovedSetOnlyUpdates
                && !movedSet.empty()
                && (movedSet.find(cachedPair.a) != movedSet.end() || movedSet.find(cachedPair.b) != movedSet.end())) {
                continue;
            }
            const std::uint64_t key = MakePairKey(cachedPair.a, cachedPair.b);
            if (seenPairs.insert(key).second) {
                ++broadphaseMetrics_.pairCacheHits;
                pairs.push_back(cachedPair);
            }
        }
    }

    std::vector<std::int32_t> stack;
    stack.reserve(treeNodes_.size());
    auto traverseProxy = [&](std::uint32_t bodyId) {
        if (bodyId >= proxies_.size()) {
            return;
        }
        const BroadphaseProxy& proxy = proxies_[bodyId];
        if (!proxy.valid || proxy.leaf < 0) {
            return;
        }
        stack.clear();
        stack.push_back(rootNode_);
        while (!stack.empty()) {
            const std::int32_t nodeId = stack.back();
            stack.pop_back();
            ++broadphaseMetrics_.queryNodeVisits;
            if (nodeId < 0 || nodeId == proxy.leaf || !Overlaps(proxy.fatBox, treeNodes_[nodeId].box)) {
                continue;
            }
            if (treeNodes_[nodeId].IsLeaf()) {
                const std::uint32_t other = static_cast<std::uint32_t>(treeNodes_[nodeId].bodyId);
                if (bodyId == other || !IsPairEligible(bodyId, other)) {
                    continue;
                }
                const std::uint64_t key = MakePairKey(bodyId, other);
                if (seenPairs.insert(key).second) {
                    pairs.push_back({std::min(bodyId, other), std::max(bodyId, other)});
                }
                continue;
            }
            stack.push_back(treeNodes_[nodeId].left);
            stack.push_back(treeNodes_[nodeId].right);
        }
    };

    const bool useMovedSetOnlyTraversal =
        broadphaseConfig_.enableMovedSetOnlyUpdates
        && broadphaseConfig_.enablePairCacheReuseForQuasiStatic
        && !cachedPotentialPairs_.empty()
        && !movedSet.empty()
        && movedSet.size() < proxies_.size();

    if (useMovedSetOnlyTraversal) {
        broadphaseMetrics_.usedMovedSetOnlyUpdate = true;
        for (const std::uint32_t movedId : movedSet) {
            traverseProxy(movedId);
        }
    } else {
        for (std::uint32_t bodyId = 0; bodyId < proxies_.size(); ++bodyId) {
            traverseProxy(bodyId);
        }
    }

    const std::uint32_t validProxyCount = broadphaseMetrics_.validProxyCount;
    if (validProxyCount > 0) {
        broadphaseMetrics_.queryNodeVisitsPerProxy =
            static_cast<float>(broadphaseMetrics_.queryNodeVisits) / static_cast<float>(validProxyCount);
    }
    if (broadphaseMetrics_.pairCacheQueries > 0) {
        broadphaseMetrics_.pairCacheHitRate =
            static_cast<float>(broadphaseMetrics_.pairCacheHits) / static_cast<float>(broadphaseMetrics_.pairCacheQueries);
    }

#ifndef NDEBUG
    const std::vector<Pair> bruteForcePairs = ComputePotentialPairsBruteForce();
    assert(PairSetFromPairs(pairs) == PairSetFromPairs(bruteForcePairs));
#endif

    cachedPotentialPairs_ = pairs;
    cachedPotentialPairKeySet_.clear();
    cachedPotentialPairKeySet_.reserve(cachedPotentialPairs_.size() * 2);
    for (const Pair& pair : cachedPotentialPairs_) {
        cachedPotentialPairKeySet_.insert(MakePairKey(pair.a, pair.b));
    }

    const auto endTime = std::chrono::steady_clock::now();
    broadphaseMetrics_.pairGenerationMs =
        std::chrono::duration<float, std::milli>(endTime - startTime).count();
    return pairs;
}

bool World::IsPairEligible(std::uint32_t a, std::uint32_t b) const {
    if (a >= proxies_.size() || b >= proxies_.size()) {
        return false;
    }
    const BroadphaseProxy& proxyA = proxies_[a];
    const BroadphaseProxy& proxyB = proxies_[b];
    if (!proxyA.valid || proxyA.leaf < 0 || !proxyB.valid || proxyB.leaf < 0) {
        return false;
    }
    if (!Overlaps(proxyA.fatBox, proxyB.fatBox)) {
        return false;
    }
    const Body& bodyA = bodies_[a];
    const Body& bodyB = bodies_[b];
    return !((bodyA.invMass == 0.0f && bodyB.invMass == 0.0f) || (bodyA.isSleeping && bodyB.isSleeping));
}

std::uint64_t World::MakePairKey(std::uint32_t a, std::uint32_t b) const {
    const std::uint32_t lo = std::min(a, b);
    const std::uint32_t hi = std::max(a, b);
    return (static_cast<std::uint64_t>(lo) << 32) | hi;
}


} // namespace minphys3d
