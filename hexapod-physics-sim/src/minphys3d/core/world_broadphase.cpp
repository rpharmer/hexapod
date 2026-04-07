#include "minphys3d/core/world.hpp"
#include "subsystems.hpp"

namespace minphys3d {
void World::UpdateBroadphaseProxies() {
    if (proxies_.size() != bodies_.size()) {
        proxies_.resize(bodies_.size());
    }
    lastBroadphaseMovedProxyCount_ = 0;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const AABB current = bodies_[i].ComputeAABB();
        if (!proxies_[i].valid) {
            proxies_[i].fatBox = ExpandAABB(current, 0.1f);
            proxies_[i].leaf = -1;
            proxies_[i].valid = true;
            EnsureProxyInTree(static_cast<std::uint32_t>(i));
        } else if (!Contains(proxies_[i].fatBox, current)) {
            if (proxies_[i].leaf >= 0) {
                RemoveLeaf(proxies_[i].leaf);
            }
            proxies_[i].fatBox = ExpandAABB(current, 0.1f);
            proxies_[i].leaf = InsertLeaf(static_cast<std::uint32_t>(i), proxies_[i].fatBox);
            ++lastBroadphaseMovedProxyCount_;
        } else {
            EnsureProxyInTree(static_cast<std::uint32_t>(i));
        }
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
        std::vector<Pair> pairs;
        if (bodies_.empty() || rootNode_ == -1) {
            return pairs;
        }
        std::unordered_set<std::uint64_t> seenPairs;
        seenPairs.reserve(bodies_.size() * 4);
        std::vector<std::int32_t> stack;
        stack.reserve(treeNodes_.size());
        for (std::uint32_t i = 0; i < proxies_.size(); ++i) {
            const BroadphaseProxy& proxy = proxies_[i];
            if (!proxy.valid || proxy.leaf < 0) {
                continue;
            }
            stack.clear();
            stack.push_back(rootNode_);
            while (!stack.empty()) {
                const std::int32_t nodeId = stack.back();
                stack.pop_back();
                if (nodeId < 0 || nodeId == proxy.leaf || !Overlaps(proxy.fatBox, treeNodes_[nodeId].box)) {
                    continue;
                }
                if (treeNodes_[nodeId].IsLeaf()) {
                    const std::uint32_t j = static_cast<std::uint32_t>(treeNodes_[nodeId].bodyId);
                    if (i >= j) {
                        continue;
                    }
                    const Body& a = bodies_[i];
                    const Body& b = bodies_[j];
                    if ((a.invMass == 0.0f && b.invMass == 0.0f) || (a.isSleeping && b.isSleeping)) {
                        continue;
                    }
                    const std::uint64_t key = (static_cast<std::uint64_t>(i) << 32) | j;
                    if (seenPairs.insert(key).second) {
                        pairs.push_back({i, j});
                    }
                    continue;
                }
                stack.push_back(treeNodes_[nodeId].left);
                stack.push_back(treeNodes_[nodeId].right);
            }
        }

#ifndef NDEBUG
        const std::vector<Pair> bruteForcePairs = ComputePotentialPairsBruteForce();
        assert(PairSetFromPairs(pairs) == PairSetFromPairs(bruteForcePairs));
#endif

        return pairs;
    }


} // namespace minphys3d
