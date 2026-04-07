#include "minphys3d/core/world.hpp"

namespace minphys3d {

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

void World::ResolveTOIPipeline(float dt) {
        float remaining = dt;
        constexpr int kMaxTOIIterations = 8;
        int iterations = 0;
        while (remaining > 1e-6f && iterations < kMaxTOIIterations) {
            const TOIEvent hit = FindEarliestTOI(remaining);
            if (!hit.hit) {
                AdvanceDynamicBodies(remaining);
                remaining = 0.0f;
                break;
            }

            const float advanceTime = std::max(0.0f, hit.toi);
            if (advanceTime > 0.0f) {
                AdvanceDynamicBodies(advanceTime);
                remaining -= advanceTime;
            }
            ResolveTOIImpact(hit);
            if (hit.toi <= 1e-6f) {
                remaining = std::max(0.0f, remaining - 1e-6f);
            }
            ++iterations;
        }

        if (remaining > 0.0f) {
            AdvanceDynamicBodies(remaining);
        }
    }

void World::BuildManifolds() {
        for (const Contact& c : contacts_) {
            bool found = false;
            for (Manifold& m : manifolds_) {
                if ((m.a == c.a && m.b == c.b) || (m.a == c.b && m.b == c.a)) {
                    m.contacts.push_back(c);
                    assert(m.contacts.size() <= kMaxContactsPerManifold);
                    m.normal = c.normal;
                    found = true;
                    break;
                }
            }
            if (!found) {
                Manifold m;
                m.a = c.a;
                m.b = c.b;
                m.normal = c.normal;
                m.manifoldType = ManifoldTypeFromFeatureKey(c.featureKey);
                m.contacts.push_back(c);
                manifolds_.push_back(m);
            }
        }

        for (Manifold& m : manifolds_) {
            if (!m.contacts.empty()) {
                m.manifoldType = ManifoldTypeFromFeatureKey(m.contacts.front().featureKey);
            }
            m.lowQuality = false;
            m.blockSolveEligible = false;
            m.usedBlockSolve = false;
            SortManifoldContacts(m.contacts);
            const Manifold* previous = nullptr;
            for (const Manifold& old : previousManifolds_) {
                if (old.pairKey() == m.pairKey()) {
                    previous = &old;
                    m.blockNormalImpulseSum = old.blockNormalImpulseSum;
                    m.blockContactKeys = old.blockContactKeys;
                    m.blockSlotValid = old.blockSlotValid;
                    m.selectedBlockContactKeys = old.selectedBlockContactKeys;
                    break;
                }
            }

            if (previous == nullptr) {
                m.blockNormalImpulseSum = {0.0f, 0.0f};
                m.blockContactKeys = {0u, 0u};
                m.blockSlotValid = {false, false};
                m.selectedBlockContactKeys = {0u, 0u};
            }

#ifndef NDEBUG
            if (previous != nullptr) {
                DebugLogContactTransitions(*previous, m);
            }
#endif

            const ManifoldKey manifoldId = MakeManifoldId(m.a, m.b, m.manifoldType);
            std::unordered_map<std::uint64_t, std::uint8_t> currentFeatureOrdinal;
            std::unordered_map<std::uint64_t, std::uint8_t> currentFeatureCounts;
            currentFeatureOrdinal.reserve(m.contacts.size());
            currentFeatureCounts.reserve(m.contacts.size());
            for (Contact& c : m.contacts) {
                const std::uint8_t ordinal = currentFeatureOrdinal[c.featureKey]++;
                ++currentFeatureCounts[c.featureKey];
                const PersistentPointKey pointKey = MakePersistentPointKey(manifoldId, c.featureKey, ordinal);
                const auto stateIt = persistentPointImpulses_.find(pointKey);
                if (stateIt != persistentPointImpulses_.end()) {
                    c.normalImpulseSum = stateIt->second.normalImpulseSum;
                    c.tangentImpulseSum = stateIt->second.tangentImpulseSum;
                    c.persistenceAge = stateIt->second.persistenceAge;
                } else {
                    c.normalImpulseSum = 0.0f;
                    c.tangentImpulseSum = 0.0f;
                    c.persistenceAge = 0;
#ifndef NDEBUG
                    ++solverTelemetry_.impulseResetPoints;
                    if (debugContactPersistence_) {
                        std::fprintf(
                            stderr,
                            "[minphys3d] impulse reset pair=(%u,%u) type=%u feature=0x%llx\n",
                            m.a,
                            m.b,
                            static_cast<unsigned>(m.manifoldType),
                            static_cast<unsigned long long>(c.featureKey));
                    }
#endif
                }
            }

#ifndef NDEBUG
            if (previous != nullptr) {
                bool manifoldTypeChanged = previous->manifoldType != m.manifoldType;
                bool removedOrAdded = previous->contacts.size() != m.contacts.size();
                bool featureMismatch = false;
                std::unordered_map<std::uint64_t, std::uint8_t> previousFeatures;
                previousFeatures.reserve(previous->contacts.size());
                for (const Contact& oldContact : previous->contacts) {
                    ++previousFeatures[oldContact.featureKey];
                }
                for (const auto& [feature, count] : currentFeatureCounts) {
                    const auto previousIt = previousFeatures.find(feature);
                    if (previousIt == previousFeatures.end() || previousIt->second != count) {
                        featureMismatch = true;
                        break;
                    }
                }
                if (!featureMismatch) {
                    for (const auto& [feature, count] : previousFeatures) {
                        const auto currentIt = currentFeatureCounts.find(feature);
                        if (currentIt == currentFeatureCounts.end() || currentIt->second != count) {
                            featureMismatch = true;
                            break;
                        }
                    }
                }

                if (manifoldTypeChanged || removedOrAdded || featureMismatch) {
                    m.lowQuality = true;
                    ++solverTelemetry_.topologyChangeEvents;
                    if (featureMismatch) {
                        ++solverTelemetry_.featureIdChurnEvents;
                    }
                    if (debugContactPersistence_) {
                        std::fprintf(
                            stderr,
                            "[minphys3d] contact topology change pair=(%u,%u) type:%u->%u contacts:%zu->%zu featureMismatch=%d\n",
                            m.a,
                            m.b,
                            static_cast<unsigned>(previous->manifoldType),
                            static_cast<unsigned>(m.manifoldType),
                            previous->contacts.size(),
                            m.contacts.size(),
                            featureMismatch ? 1 : 0);
                    }
                }
            }
#endif

            RefreshManifoldBlockCache(m);
#ifndef NDEBUG
            DebugLogManifoldContactKeys(m);
            if (debugContactPersistence_) {
                std::fprintf(stderr,
                             "[minphys3d] frame=%llu block_slots pair=(%u,%u) type=%u slots=[(%d,%llu),(%d,%llu)] selected_keys=(%llu,%llu)\n",
                             static_cast<unsigned long long>(debugFrameIndex_),
                             m.a,
                             m.b,
                             static_cast<unsigned>(m.manifoldType),
                             m.blockSlotValid[0] ? 1 : 0,
                             static_cast<unsigned long long>(m.blockContactKeys[0]),
                             m.blockSlotValid[1] ? 1 : 0,
                             static_cast<unsigned long long>(m.blockContactKeys[1]),
                             static_cast<unsigned long long>(m.selectedBlockContactKeys[0]),
                             static_cast<unsigned long long>(m.selectedBlockContactKeys[1]));
            }
#endif
            SelectBlockSolvePair(m);
#ifndef NDEBUG
            RecordSelectedPairHistory(m);
#endif
        }

        std::unordered_map<ManifoldKey, std::size_t, ManifoldKeyHash> manifoldCountByPair;
        for (const Manifold& m : manifolds_) {
            const ManifoldKey key = MakeManifoldId(m.a, m.b, m.manifoldType);
            ++manifoldCountByPair[key];
            assert(manifoldCountByPair[key] == 1);
            assert(m.contacts.size() <= kMaxContactsPerManifold);
        }
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

void World::GenerateContacts() {
        const std::vector<Pair> pairs = ComputePotentialPairs();
        for (const Pair& pair : pairs) {
            const Body& a = bodies_[pair.a];
            const Body& b = bodies_[pair.b];

            if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
                SphereSphere(pair.a, pair.b);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Capsule) {
                SphereCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Sphere) {
                SphereCapsule(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Capsule) {
                CapsuleCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Plane) {
                CapsulePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Capsule) {
                CapsulePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Box) {
                CapsuleBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Capsule) {
                CapsuleBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
                SpherePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
                SpherePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Plane) {
                BoxPlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Box) {
                BoxPlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Box) {
                SphereBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Sphere) {
                SphereBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) {
                BoxBox(pair.a, pair.b);
            }
        }
    }

void World::SphereSphere(std::uint32_t ia, std::uint32_t ib) {
        const Body& a = bodies_[ia];
        const Body& b = bodies_[ib];
        const Vec3 delta = b.position - a.position;
        const float distSq = LengthSquared(delta);
        const float radiusSum = a.radius + b.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }

        const float dist = std::sqrt(std::max(distSq, kEpsilon));
        const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{1.0f, 0.0f, 0.0f};
        const std::uint64_t featureKey = PackFeatureKey(1, 0, 0, 0, 0, 0);
        AddContact(ia, ib, normal, a.position + normal * a.radius, radiusSum - dist, featureKey);
    }

void World::BoxBox(std::uint32_t aId, std::uint32_t bId) {
        const Body& a = bodies_[aId];
        const Body& b = bodies_[bId];

        const Vec3 aAxes[3] = {
            Rotate(a.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(a.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(a.orientation, {0.0f, 0.0f, 1.0f}),
        };
        const Vec3 bAxes[3] = {
            Rotate(b.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(b.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(b.orientation, {0.0f, 0.0f, 1.0f}),
        };

        struct AxisCandidate {
            Vec3 axis{};
            int type = 0; // 0 = face A, 1 = face B, 2 = edge-edge
            int indexA = -1;
            int indexB = -1;
            float overlap = 0.0f;
        };

        AxisCandidate candidates[15];
        int axisCount = 0;
        for (int i = 0; i < 3; ++i) candidates[axisCount++] = {aAxes[i], 0, i, -1};
        for (int i = 0; i < 3; ++i) candidates[axisCount++] = {bAxes[i], 1, -1, i};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                candidates[axisCount++] = {Cross(aAxes[i], bAxes[j]), 2, i, j};
            }
        }

        const Vec3 centerDelta = b.position - a.position;
        constexpr float kParallelAxisEps = 1e-8f;
        constexpr float kFaceAxisBias = 0.0025f;
        float bestOverlap = std::numeric_limits<float>::infinity();
        float bestScore = std::numeric_limits<float>::infinity();
        AxisCandidate best{};

        for (int i = 0; i < axisCount; ++i) {
            Vec3 axis = candidates[i].axis;
            const float lenSq = LengthSquared(axis);
            if (lenSq <= kParallelAxisEps) {
                continue;
            }
            axis = axis / std::sqrt(lenSq);

            const float ra = ProjectBoxOntoAxis(a, axis);
            const float rb = ProjectBoxOntoAxis(b, axis);
            const float distance = std::abs(Dot(centerDelta, axis));
            const float overlap = ra + rb - distance;
            if (overlap < 0.0f) {
                return;
            }
            const float biasedOverlap = overlap + ((candidates[i].type == 2) ? kFaceAxisBias : 0.0f);
            if (biasedOverlap < bestScore) {
                bestScore = biasedOverlap;
                bestOverlap = overlap;
                best = candidates[i];
                best.axis = axis;
                best.overlap = overlap;
                if (Dot(centerDelta, best.axis) < 0.0f) {
                    best.axis = -best.axis;
                }
            }
        }

        if (bestOverlap == std::numeric_limits<float>::infinity()) {
            return;
        }

        struct ClipVertex {
            Vec3 point{};
            std::uint8_t incidentFeature = 0; // 0..3 face vertices, 4..7 clipped face edges
            std::uint8_t clipMask = 0; // reference-side clipping planes touched
            float depth = 0.0f;
        };

        auto localFaceVertices = [](const Vec3& he, int axis, float sign) {
            std::vector<ClipVertex> verts;
            verts.reserve(4);
            if (axis == 0) {
                verts.push_back({{sign * he.x, -he.y, -he.z}, 0, 0});
                verts.push_back({{sign * he.x,  he.y, -he.z}, 1, 0});
                verts.push_back({{sign * he.x,  he.y,  he.z}, 2, 0});
                verts.push_back({{sign * he.x, -he.y,  he.z}, 3, 0});
            } else if (axis == 1) {
                verts.push_back({{-he.x, sign * he.y, -he.z}, 0, 0});
                verts.push_back({{ he.x, sign * he.y, -he.z}, 1, 0});
                verts.push_back({{ he.x, sign * he.y,  he.z}, 2, 0});
                verts.push_back({{-he.x, sign * he.y,  he.z}, 3, 0});
            } else {
                verts.push_back({{-he.x, -he.y, sign * he.z}, 0, 0});
                verts.push_back({{ he.x, -he.y, sign * he.z}, 1, 0});
                verts.push_back({{ he.x,  he.y, sign * he.z}, 2, 0});
                verts.push_back({{-he.x,  he.y, sign * he.z}, 3, 0});
            }
            return verts;
        };

        auto worldFromLocal = [](const Body& body, const Vec3& p) {
            return body.position + Rotate(body.orientation, p);
        };

        auto clipPolygonAgainstPlane = [](const std::vector<ClipVertex>& poly, const Vec3& n, float d, std::uint8_t planeBit) {
            std::vector<ClipVertex> out;
            if (poly.empty()) {
                return out;
            }
            for (std::size_t i = 0; i < poly.size(); ++i) {
                const ClipVertex& va = poly[i];
                const ClipVertex& vb = poly[(i + 1) % poly.size()];
                const float da = Dot(n, va.point) - d;
                const float db = Dot(n, vb.point) - d;
                const bool ina = da <= 0.0f;
                const bool inb = db <= 0.0f;

                if (ina && inb) {
                    ClipVertex kept = vb;
                    if (std::abs(db) <= 1e-4f) {
                        kept.clipMask = static_cast<std::uint8_t>(kept.clipMask | (1u << planeBit));
                    }
                    out.push_back(kept);
                } else if (ina && !inb) {
                    const float t = da / (da - db + kEpsilon);
                    const std::uint8_t edgeId = static_cast<std::uint8_t>(4u + (i & 3u));
                    out.push_back({va.point + (vb.point - va.point) * t, edgeId, static_cast<std::uint8_t>((va.clipMask | vb.clipMask) | (1u << planeBit)), 0.0f});
                } else if (!ina && inb) {
                    const float t = da / (da - db + kEpsilon);
                    const std::uint8_t edgeId = static_cast<std::uint8_t>(4u + (i & 3u));
                    out.push_back({va.point + (vb.point - va.point) * t, edgeId, static_cast<std::uint8_t>((va.clipMask | vb.clipMask) | (1u << planeBit)), 0.0f});
                    ClipVertex kept = vb;
                    if (std::abs(db) <= 1e-4f) {
                        kept.clipMask = static_cast<std::uint8_t>(kept.clipMask | (1u << planeBit));
                    }
                    out.push_back(kept);
                }
            }
            return out;
        };

        auto addUniqueContact = [&](std::vector<ClipVertex>& pts, const ClipVertex& candidate) {
            for (ClipVertex& existing : pts) {
                if (LengthSquared(existing.point - candidate.point) < 1e-4f) {
                    existing.clipMask = static_cast<std::uint8_t>(existing.clipMask | candidate.clipMask);
                    return;
                }
            }
            pts.push_back(candidate);
        };

        if (best.type == 0 || best.type == 1) {
            const bool aReference = (best.type == 0);
            const Body& ref = aReference ? a : b;
            const Body& inc = aReference ? b : a;
            const int refAxis = aReference ? best.indexA : best.indexB;

            Vec3 refNormal = best.axis;
            if (!aReference) {
                refNormal = -refNormal;
            }

            const Vec3 refBasis[3] = {
                Rotate(ref.orientation, {1.0f, 0.0f, 0.0f}),
                Rotate(ref.orientation, {0.0f, 1.0f, 0.0f}),
                Rotate(ref.orientation, {0.0f, 0.0f, 1.0f}),
            };
            const Vec3 incBasis[3] = {
                Rotate(inc.orientation, {1.0f, 0.0f, 0.0f}),
                Rotate(inc.orientation, {0.0f, 1.0f, 0.0f}),
                Rotate(inc.orientation, {0.0f, 0.0f, 1.0f}),
            };

            Vec3 refAxisDir = refBasis[refAxis];
            float refSign = 1.0f;
            if (Dot(refAxisDir, refNormal) < 0.0f) {
                refAxisDir = -refAxisDir;
                refSign = -1.0f;
            }

            int incidentAxis = 0;
            float minDot = std::numeric_limits<float>::infinity();
            for (int i = 0; i < 3; ++i) {
                const float d = Dot(incBasis[i], refNormal);
                if (d < minDot) {
                    minDot = d;
                    incidentAxis = i;
                }
            }
            float incSign = (Dot(incBasis[incidentAxis], refNormal) > 0.0f) ? -1.0f : 1.0f;

            std::vector<ClipVertex> poly = localFaceVertices(inc.halfExtents, incidentAxis, incSign);
            for (ClipVertex& p : poly) {
                p.point = worldFromLocal(inc, p.point);
            }

            const int u = (refAxis + 1) % 3;
            const int v = (refAxis + 2) % 3;
            const Vec3 sideU = refBasis[u];
            const Vec3 sideV = refBasis[v];
            const float limitU = (u == 0 ? ref.halfExtents.x : (u == 1 ? ref.halfExtents.y : ref.halfExtents.z));
            const float limitV = (v == 0 ? ref.halfExtents.x : (v == 1 ? ref.halfExtents.y : ref.halfExtents.z));

            poly = clipPolygonAgainstPlane(poly,  sideU, Dot(sideU, ref.position) + limitU, 0);
            poly = clipPolygonAgainstPlane(poly, -sideU, Dot(-sideU, ref.position) + limitU, 1);
            poly = clipPolygonAgainstPlane(poly,  sideV, Dot(sideV, ref.position) + limitV, 2);
            poly = clipPolygonAgainstPlane(poly, -sideV, Dot(-sideV, ref.position) + limitV, 3);

            Vec3 localPlanePoint{};
            if (refAxis == 0) localPlanePoint = {refSign * ref.halfExtents.x, 0.0f, 0.0f};
            if (refAxis == 1) localPlanePoint = {0.0f, refSign * ref.halfExtents.y, 0.0f};
            if (refAxis == 2) localPlanePoint = {0.0f, 0.0f, refSign * ref.halfExtents.z};
            const Vec3 planePoint = worldFromLocal(ref, localPlanePoint);

            std::vector<ClipVertex> contacts;
            for (const ClipVertex& p : poly) {
                const float depth = Dot(planePoint - p.point, refNormal);
                if (depth >= -0.02f) {
                    ClipVertex c = p;
                    c.point = p.point;
                    c.depth = depth;
                    addUniqueContact(contacts, c);
                }
            }

            if (contacts.empty()) {
                const Vec3 pointA = ClosestPointOnBox(a, b.position);
                const Vec3 pointB = ClosestPointOnBox(b, a.position);
                contacts.push_back({0.5f * (pointA + pointB), 0, 0, best.overlap});
            }

            const auto coordOnAxis = [](const Vec3& p, const Vec3& origin, const Vec3& axis) {
                return Dot(p - origin, axis);
            };
            std::sort(contacts.begin(), contacts.end(), [&](const ClipVertex& lhs, const ClipVertex& rhs) {
                const float lu = coordOnAxis(lhs.point, ref.position, sideU);
                const float ru = coordOnAxis(rhs.point, ref.position, sideU);
                if (std::abs(lu - ru) > 1e-4f) return lu < ru;
                const float lv = coordOnAxis(lhs.point, ref.position, sideV);
                const float rv = coordOnAxis(rhs.point, ref.position, sideV);
                return lv < rv;
            });

            if (contacts.size() > 4) {
                const auto pickIndex = [&](bool maxU, bool maxV) {
                    std::size_t bestIndex = 0;
                    float bestScore = -std::numeric_limits<float>::infinity();
                    for (std::size_t i = 0; i < contacts.size(); ++i) {
                        const float uCoord = coordOnAxis(contacts[i].point, ref.position, sideU);
                        const float vCoord = coordOnAxis(contacts[i].point, ref.position, sideV);
                        const float score = (maxU ? uCoord : -uCoord) + (maxV ? vCoord : -vCoord);
                        if (score > bestScore) {
                            bestScore = score;
                            bestIndex = i;
                        }
                    }
                    return bestIndex;
                };

                std::array<std::size_t, 4> picked = {
                    pickIndex(false, false),
                    pickIndex(true, false),
                    pickIndex(true, true),
                    pickIndex(false, true),
                };
                std::vector<ClipVertex> reduced;
                reduced.reserve(4);
                for (std::size_t idx : picked) {
                    addUniqueContact(reduced, contacts[idx]);
                }
                contacts = std::move(reduced);
                std::sort(contacts.begin(), contacts.end(), [&](const ClipVertex& lhs, const ClipVertex& rhs) {
                    const float lu = coordOnAxis(lhs.point, ref.position, sideU);
                    const float ru = coordOnAxis(rhs.point, ref.position, sideU);
                    if (std::abs(lu - ru) > 1e-4f) return lu < ru;
                    const float lv = coordOnAxis(lhs.point, ref.position, sideV);
                    const float rv = coordOnAxis(rhs.point, ref.position, sideV);
                    return lv < rv;
                });
            }

            const Vec3 normalAB = aReference ? best.axis : -best.axis;
            const std::uint8_t referenceFaceIndex = static_cast<std::uint8_t>(refAxis * 2 + (refSign > 0.0f ? 0 : 1));
            const std::uint8_t incidentFaceIndex = static_cast<std::uint8_t>(incidentAxis * 2 + (incSign > 0.0f ? 0 : 1));
            for (std::size_t i = 0; i < std::min<std::size_t>(4, contacts.size()); ++i) {
                const std::uint8_t referenceFeature = contacts[i].clipMask;
                const std::uint8_t incidentFeature = contacts[i].incidentFeature;
                const std::uint8_t clippedMap = static_cast<std::uint8_t>(contacts[i].clipMask ^ ((contacts[i].incidentFeature & 0xFu) << 4));
                const std::uint64_t featureKey = PackFeatureKey(9, referenceFaceIndex, incidentFaceIndex, referenceFeature, incidentFeature, clippedMap);
                AddContact(aId, bId, normalAB, contacts[i].point, bestOverlap, featureKey);
            }
            return;
        }

        const auto halfExtentAxis = [](const Vec3& he, int axis) {
            return (axis == 0) ? he.x : ((axis == 1) ? he.y : he.z);
        };
        const auto edgeSegment = [&](const Body& box, const Vec3 axes[3], int axis, const Vec3& n) {
            Vec3 local{0.0f, 0.0f, 0.0f};
            for (int i = 0; i < 3; ++i) {
                if (i == axis) continue;
                const float extent = halfExtentAxis(box.halfExtents, i);
                const float value = (Dot(axes[i], n) >= 0.0f) ? extent : -extent;
                if (i == 0) local.x = value;
                else if (i == 1) local.y = value;
                else local.z = value;
            }
            const float edgeExtent = halfExtentAxis(box.halfExtents, axis);
            const Vec3 p0 = box.position + Rotate(box.orientation, local - Vec3{axis == 0 ? edgeExtent : 0.0f, axis == 1 ? edgeExtent : 0.0f, axis == 2 ? edgeExtent : 0.0f});
            const Vec3 p1 = box.position + Rotate(box.orientation, local + Vec3{axis == 0 ? edgeExtent : 0.0f, axis == 1 ? edgeExtent : 0.0f, axis == 2 ? edgeExtent : 0.0f});
            return std::pair<Vec3, Vec3>{p0, p1};
        };
        const auto closestPointsOnSegments = [](const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
            const Vec3 d1 = q1 - p1;
            const Vec3 d2 = q2 - p2;
            const Vec3 r = p1 - p2;
            const float aLen = Dot(d1, d1);
            const float eLen = Dot(d2, d2);
            const float f = Dot(d2, r);

            float s = 0.0f;
            float t = 0.0f;
            if (aLen <= kEpsilon && eLen <= kEpsilon) {
                return std::pair<Vec3, Vec3>{p1, p2};
            }
            if (aLen <= kEpsilon) {
                t = std::clamp(f / (eLen + kEpsilon), 0.0f, 1.0f);
            } else {
                const float c = Dot(d1, r);
                if (eLen <= kEpsilon) {
                    s = std::clamp(-c / (aLen + kEpsilon), 0.0f, 1.0f);
                } else {
                    const float bDot = Dot(d1, d2);
                    const float denom = aLen * eLen - bDot * bDot;
                    if (denom > kEpsilon) {
                        s = std::clamp((bDot * f - c * eLen) / denom, 0.0f, 1.0f);
                    }
                    t = (bDot * s + f) / (eLen + kEpsilon);
                    if (t < 0.0f) {
                        t = 0.0f;
                        s = std::clamp(-c / (aLen + kEpsilon), 0.0f, 1.0f);
                    } else if (t > 1.0f) {
                        t = 1.0f;
                        s = std::clamp((bDot - c) / (aLen + kEpsilon), 0.0f, 1.0f);
                    }
                }
            }
            return std::pair<Vec3, Vec3>{p1 + d1 * s, p2 + d2 * t};
        };

        const std::uint8_t edgeA = static_cast<std::uint8_t>(best.indexA < 0 ? 0 : best.indexA);
        const std::uint8_t edgeB = static_cast<std::uint8_t>(best.indexB < 0 ? 0 : best.indexB);
        const auto [a0, a1] = edgeSegment(a, aAxes, edgeA, best.axis);
        const auto [b0, b1] = edgeSegment(b, bAxes, edgeB, -best.axis);
        const auto [pointA, pointB] = closestPointsOnSegments(a0, a1, b0, b1);

        const std::uint64_t featureKey = PackFeatureKey(10, edgeA, edgeB, edgeA, edgeB, 0);
        AddContact(aId, bId, best.axis, 0.5f * (pointA + pointB), best.overlap, featureKey);
    }

void World::WarmStartContacts() {
        for (Manifold& m : manifolds_) {
            EnsureStableTwoPointOrder(m);
            std::array<bool, 2> skipPerContactNormal{false, false};
            if (m.contacts.size() == 2 && m.blockSlotValid[0] && m.blockSlotValid[1]) {
                const int slotForContact0 = FindBlockSlot(m, m.contacts[0].key);
                const int slotForContact1 = FindBlockSlot(m, m.contacts[1].key);
                const bool blockCacheMatchesPair = slotForContact0 >= 0 && slotForContact1 >= 0 && slotForContact0 != slotForContact1;
                if (blockCacheMatchesPair) {
                    for (int contactIndex = 0; contactIndex < 2; ++contactIndex) {
                        Contact& c = m.contacts[contactIndex];
                        const int slot = FindBlockSlot(m, c.key);
                        if (slot < 0) {
                            continue;
                        }
                        const float cachedNormalImpulse = m.blockNormalImpulseSum[slot];
                        if (cachedNormalImpulse == 0.0f) {
                            skipPerContactNormal[contactIndex] = true;
                            continue;
                        }

                        Body& a = bodies_[c.a];
                        Body& b = bodies_[c.b];
                        const Mat3 invIA = a.InvInertiaWorld();
                        const Mat3 invIB = b.InvInertiaWorld();
                        const Vec3 ra = c.point - a.position;
                        const Vec3 rb = c.point - b.position;
                        ApplyImpulse(a, b, invIA, invIB, ra, rb, cachedNormalImpulse * m.normal);
                        skipPerContactNormal[contactIndex] = true;
                    }
                }
            }

            std::size_t contactIndex = 0;
            for (Contact& c : m.contacts) {
                const bool skipNormal = contactIndex < skipPerContactNormal.size() && skipPerContactNormal[contactIndex];
                if (!skipNormal && c.normalImpulseSum == 0.0f && c.tangentImpulseSum == 0.0f) {
                    ++contactIndex;
                    continue;
                }
                if (skipNormal && c.tangentImpulseSum == 0.0f) {
                    ++contactIndex;
                    continue;
                }

                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                const Mat3 invIA = a.InvInertiaWorld();
                const Mat3 invIB = b.InvInertiaWorld();
                const Vec3 ra = c.point - a.position;
                const Vec3 rb = c.point - b.position;

                Vec3 tangent{0.0f, 0.0f, 0.0f};
                const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
                const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
                const Vec3 rv = vb - va;
                Vec3 trial = rv - Dot(rv, c.normal) * c.normal;
                if (LengthSquared(trial) > kEpsilon) {
                    tangent = Normalize(trial);
                }

                const Vec3 normalImpulse = skipNormal ? Vec3{0.0f, 0.0f, 0.0f} : c.normalImpulseSum * c.normal;
                const Vec3 impulse = normalImpulse + c.tangentImpulseSum * tangent;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
                ++contactIndex;
            }
        }
    }

bool World::SolveNormalBlock2(Manifold& manifold, BlockSolveFallbackReason& fallbackReason) {
        fallbackReason = BlockSolveFallbackReason::None;
        if (manifold.contacts.size() < 2) {
            fallbackReason = BlockSolveFallbackReason::LcpFailure;
            return false;
        }

        const int idx0 = manifold.selectedBlockContactIndices[0];
        const int idx1 = manifold.selectedBlockContactIndices[1];
        if (idx0 < 0 || idx1 < 0 || idx0 == idx1
            || static_cast<std::size_t>(std::max(idx0, idx1)) >= manifold.contacts.size()) {
            fallbackReason = BlockSolveFallbackReason::Ineligible;
            return false;
        }

        Contact& c0 = manifold.contacts[static_cast<std::size_t>(idx0)];
        Contact& c1 = manifold.contacts[static_cast<std::size_t>(idx1)];
        Body& a = bodies_[c0.a];
        Body& b = bodies_[c0.b];

        const Vec3 manifoldNormal = Normalize(manifold.normal);
        if (LengthSquared(manifoldNormal) <= kEpsilon) {
            fallbackReason = BlockSolveFallbackReason::InvalidManifoldNormal;
            return false;
        }
        if (Dot(c0.normal, manifoldNormal) < 0.95f || Dot(c1.normal, manifoldNormal) < 0.95f) {
            fallbackReason = BlockSolveFallbackReason::ContactNormalMismatch;
            return false;
        }
        Vec3 normal0 = Normalize(c0.normal);
        Vec3 normal1 = Normalize(c1.normal);
        if (LengthSquared(normal0) <= kEpsilon || LengthSquared(normal1) <= kEpsilon) {
            fallbackReason = BlockSolveFallbackReason::InvalidManifoldNormal;
            return false;
        }

        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();
        const Vec3 ra0 = c0.point - a.position;
        const Vec3 rb0 = c0.point - b.position;
        const Vec3 ra1 = c1.point - a.position;
        const Vec3 rb1 = c1.point - b.position;

        const Vec3 ra0CrossN0 = Cross(ra0, normal0);
        const Vec3 rb0CrossN0 = Cross(rb0, normal0);
        const Vec3 ra1CrossN1 = Cross(ra1, normal1);
        const Vec3 rb1CrossN1 = Cross(rb1, normal1);

        const float invMassSum = a.invMass + b.invMass;
        const float normalDot = Dot(normal0, normal1);
        const float k11 = invMassSum + Dot(ra0CrossN0, invIA * ra0CrossN0) + Dot(rb0CrossN0, invIB * rb0CrossN0);
        const float k22 = invMassSum + Dot(ra1CrossN1, invIA * ra1CrossN1) + Dot(rb1CrossN1, invIB * rb1CrossN1);
        const float k12 = invMassSum * normalDot + Dot(ra0CrossN0, invIA * ra1CrossN1) + Dot(rb0CrossN0, invIB * rb1CrossN1);
        const float k21 = k12;

        const float det = k11 * k22 - k12 * k21;
        if (k11 <= contactSolverConfig_.blockDiagonalMinimum || k22 <= contactSolverConfig_.blockDiagonalMinimum) {
            fallbackReason = BlockSolveFallbackReason::DegenerateMassMatrix;
            return false;
        }
        if (std::abs(det) <= contactSolverConfig_.blockDeterminantEpsilon) {
            fallbackReason = BlockSolveFallbackReason::DegenerateMassMatrix;
            return false;
        }
        if (contactSolverConfig_.blockConditionEstimateMax > 0.0f) {
            const float matrixNorm = std::max(std::abs(k11) + std::abs(k12), std::abs(k21) + std::abs(k22));
            const float invNorm = std::max(std::abs(k22) + std::abs(k12), std::abs(k21) + std::abs(k11)) / std::abs(det);
            const float conditionEstimate = matrixNorm * invNorm;
            if (!std::isfinite(conditionEstimate) || conditionEstimate > contactSolverConfig_.blockConditionEstimateMax) {
                fallbackReason = BlockSolveFallbackReason::ConditionEstimateExceeded;
                return false;
            }
        }

        const int slot0 = FindBlockSlot(manifold, c0.key);
        const int slot1 = FindBlockSlot(manifold, c1.key);
        if (slot0 < 0 || slot1 < 0 || slot0 == slot1) {
            fallbackReason = BlockSolveFallbackReason::MissingBlockSlots;
            return false;
        }

        const Vec3 va0 = a.velocity + Cross(a.angularVelocity, ra0);
        const Vec3 vb0 = b.velocity + Cross(b.angularVelocity, rb0);
        const Vec3 va1 = a.velocity + Cross(a.angularVelocity, ra1);
        const Vec3 vb1 = b.velocity + Cross(b.angularVelocity, rb1);
        const float vn0 = Dot(vb0 - va0, normal0);
        const float vn1 = Dot(vb1 - va1, normal1);

        const auto computeRhs = [&](Contact& c, const Vec3& contactNormal, float separatingVelocity) {
            const float speedIntoContact = -separatingVelocity;
            const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);

            float biasTerm = 0.0f;
            const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
            if (contactSolverConfig_.useSplitImpulse) {
                if (penetrationError > 0.0f) {
                    const float invMassSum = a.invMass + b.invMass;
                    if (invMassSum > kEpsilon) {
                        const float correctionMagnitude = contactSolverConfig_.splitImpulseCorrectionFactor * penetrationError / invMassSum;
                        const Vec3 correction = correctionMagnitude * contactNormal;
                        if (!a.isSleeping) {
                            a.position -= correction * a.invMass;
                        }
                        if (!b.isSleeping) {
                            b.position += correction * b.invMass;
                        }
                    }
                }
            } else if (currentSubstepDt_ > kEpsilon) {
                const float maxSafeSeparatingSpeed = penetrationError / currentSubstepDt_;
                if (separatingVelocity <= maxSafeSeparatingSpeed) {
                    biasTerm = (contactSolverConfig_.penetrationBiasFactor * penetrationError) / currentSubstepDt_;
                }
            }

            return -(1.0f + restitution) * separatingVelocity + std::max(biasTerm, 0.0f);
        };

        const float rhs0 = computeRhs(c0, normal0, vn0);
        const float rhs1 = computeRhs(c1, normal1, vn1);

        const float old0 = std::max(c0.normalImpulseSum, 0.0f);
        const float old1 = std::max(c1.normalImpulseSum, 0.0f);
#ifndef NDEBUG
        manifold.blockSolveDebug.selectedPreNormalImpulses = {old0, old1};
        manifold.blockSolveDebug.selectedPairPenetrationStep = c0.penetration + c1.penetration;
#endif
        const float q0 = rhs0 + k11 * old0 + k12 * old1;
        const float q1 = rhs1 + k21 * old0 + k22 * old1;

        auto residualW = [&](float lambda0, float lambda1) {
            return std::array<float, 2>{
                k11 * lambda0 + k12 * lambda1 - q0,
                k21 * lambda0 + k22 * lambda1 - q1,
            };
        };

        constexpr float lcpEpsilon = 1e-5f;
        bool solved = false;
        float new0 = old0;
        float new1 = old1;

        const float invDet = 1.0f / det;
        const float both0 = (k22 * q0 - k12 * q1) * invDet;
        const float both1 = (-k21 * q0 + k11 * q1) * invDet;
        if (both0 >= 0.0f && both1 >= 0.0f) {
            const auto w = residualW(both0, both1);
            if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
                new0 = both0;
                new1 = both1;
                solved = true;
            }
        }

        if (!solved && k11 > kEpsilon) {
            const float one0 = std::max(0.0f, q0 / k11);
            const float one1 = 0.0f;
            const auto w = residualW(one0, one1);
            if (w[1] >= -lcpEpsilon) {
                new0 = one0;
                new1 = one1;
                solved = true;
            }
        }

        if (!solved && k22 > kEpsilon) {
            const float one0 = 0.0f;
            const float one1 = std::max(0.0f, q1 / k22);
            const auto w = residualW(one0, one1);
            if (w[0] >= -lcpEpsilon) {
                new0 = one0;
                new1 = one1;
                solved = true;
            }
        }

        if (!solved) {
            const auto w = residualW(0.0f, 0.0f);
            if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
                new0 = 0.0f;
                new1 = 0.0f;
                solved = true;
            }
        }

        if (!solved) {
            fallbackReason = BlockSolveFallbackReason::LcpFailure;
            return false;
        }
        if (!std::isfinite(new0) || !std::isfinite(new1)) {
            fallbackReason = BlockSolveFallbackReason::NonFiniteResult;
            return false;
        }

        new0 = std::max(0.0f, new0);
        new1 = std::max(0.0f, new1);
        manifold.blockNormalImpulseSum[slot0] = new0;
        manifold.blockNormalImpulseSum[slot1] = new1;
        c0.normalImpulseSum = new0;
        c1.normalImpulseSum = new1;
#ifndef NDEBUG
        manifold.blockSolveDebug.selectedPostNormalImpulses = {new0, new1};
#endif
        const float delta0 = new0 - old0;
        const float delta1 = new1 - old1;
        ApplyImpulse(a, b, invIA, invIB, ra0, rb0, delta0 * normal0);
        ApplyImpulse(a, b, invIA, invIB, ra1, rb1, delta1 * normal1);
        return true;
    }

void World::SolveDistanceJoint(DistanceJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
        const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
        const Vec3 pa = a.position + ra;
        const Vec3 pb = b.position + rb;
        const Vec3 delta = pb - pa;
        const float len = Length(delta);
        if (len <= kEpsilon) {
            return;
        }

        const Vec3 n = delta / len;
        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const float relVel = Dot(vb - va, n);
        const float error = len - j.restLength;
        if (std::abs(error) > kWakeContactPenetrationThreshold
            || std::abs(relVel) > kWakeJointRelativeSpeedThreshold) {
            WakeConnectedBodies(j.a);
            WakeConnectedBodies(j.b);
        }

        const Vec3 raCrossN = Cross(ra, n);
        const Vec3 rbCrossN = Cross(rb, n);
        const float effMass = a.invMass + b.invMass
            + Dot(raCrossN, invIA * raCrossN)
            + Dot(rbCrossN, invIB * rbCrossN);
        if (effMass <= kEpsilon) {
            return;
        }

        const float bias = j.stiffness * error + j.damping * relVel;
        const float lambda = -bias / effMass;
        j.impulseSum += lambda;
        ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
    }

void World::SolveHingeJoint(HingeJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
        const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
        const Vec3 pa = a.position + ra;
        const Vec3 pb = b.position + rb;
        const Vec3 error = pb - pa;
        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const Vec3 relVel = vb - va;
        const float relAngSpeed = Length(b.angularVelocity - a.angularVelocity);
        if (Length(error) > kWakeContactPenetrationThreshold
            || Length(relVel) > kWakeJointRelativeSpeedThreshold
            || relAngSpeed > kWakeJointRelativeSpeedThreshold
            || j.motorEnabled) {
            WakeConnectedBodies(j.a);
            WakeConnectedBodies(j.b);
        }

        const Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        float* impulseSums[3] = {&j.impulseX, &j.impulseY, &j.impulseZ};

        for (int i = 0; i < 3; ++i) {
            const Vec3 n = axes[i];
            const Vec3 raCrossN = Cross(ra, n);
            const Vec3 rbCrossN = Cross(rb, n);
            const float effMass = a.invMass + b.invMass
                + Dot(raCrossN, invIA * raCrossN)
                + Dot(rbCrossN, invIB * rbCrossN);
            if (effMass <= kEpsilon) {
                continue;
            }

            const float bias = 0.2f * Dot(error, n) + 0.1f * Dot(relVel, n);
            const float lambda = -bias / effMass;
            *impulseSums[i] += lambda;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
        }

        const Vec3 axisA = Normalize(Rotate(a.orientation, j.localAxisA));
        const Vec3 axisB = Normalize(Rotate(b.orientation, j.localAxisB));
        Vec3 t1 = Cross(axisA, {1.0f, 0.0f, 0.0f});
        if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axisA, {0.0f, 0.0f, 1.0f});
        t1 = Normalize(t1);
        const Vec3 t2 = Normalize(Cross(axisA, t1));

        const Vec3 angularError = Cross(axisA, axisB);
        const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;

        const Vec3 angAxes[2] = {t1, t2};
        float* angImpulseSums[2] = {&j.angularImpulse1, &j.angularImpulse2};
        for (int i = 0; i < 2; ++i) {
            const Vec3 n = angAxes[i];
            const float effMass = Dot(n, invIA * n) + Dot(n, invIB * n);
            if (effMass <= kEpsilon) {
                continue;
            }
            const float bias = 0.2f * Dot(angularError, n) + 0.1f * Dot(relAngVel, n);
            const float lambda = -bias / effMass;
            *angImpulseSums[i] += lambda;
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * n);
        }

        float hingeAngle = std::atan2(Dot(Cross(axisA, axisB), t1), Dot(axisA, axisB));
        if (j.limitsEnabled) {
            float limitError = 0.0f;
            if (hingeAngle < j.lowerAngle) {
                limitError = hingeAngle - j.lowerAngle;
            } else if (hingeAngle > j.upperAngle) {
                limitError = hingeAngle - j.upperAngle;
            }
            if (std::abs(limitError) > kEpsilon) {
                const float effMass = Dot(axisA, invIA * axisA) + Dot(axisA, invIB * axisA);
                if (effMass > kEpsilon) {
                    const float lambda = -(0.2f * limitError + 0.05f * Dot(relAngVel, axisA)) / effMass;
                    ApplyAngularImpulse(a, b, invIA, invIB, lambda * axisA);
                }
            }
        }

        if (j.motorEnabled) {
            const float effMass = Dot(axisA, invIA * axisA) + Dot(axisA, invIB * axisA);
            if (effMass > kEpsilon) {
                float lambda = -(Dot(relAngVel, axisA) - j.motorSpeed) / effMass;
                const float oldImpulse = j.motorImpulseSum;
                j.motorImpulseSum = std::clamp(j.motorImpulseSum + lambda, -j.maxMotorTorque, j.maxMotorTorque);
                lambda = j.motorImpulseSum - oldImpulse;
                ApplyAngularImpulse(a, b, invIA, invIB, lambda * axisA);
            }
        }
    }

void World::SolveIslands() {
        for (const Island& island : islands_) {
            for (std::size_t mi : island.manifolds) {
                SolveContactsInManifold(manifolds_[mi]);
            }
            for (std::size_t ji : island.joints) {
                SolveDistanceJoint(joints_[ji]);
            }
            for (std::size_t hi : island.hinges) {
                SolveHingeJoint(hingeJoints_[hi]);
            }
        }
    }

void World::UpdateSleeping() {
        std::vector<bool> visited(bodies_.size(), false);
        for (std::uint32_t start = 0; start < bodies_.size(); ++start) {
            if (visited[start] || bodies_[start].invMass == 0.0f) {
                continue;
            }

            std::vector<std::uint32_t> islandBodies;
            std::vector<std::uint32_t> stack{start};
            visited[start] = true;
            bool islandNearlyStill = true;

            while (!stack.empty()) {
                const std::uint32_t id = stack.back();
                stack.pop_back();
                Body& body = bodies_[id];
                islandBodies.push_back(id);

                const float linearSpeedSq = LengthSquared(body.velocity);
                const float angularSpeedSq = LengthSquared(body.angularVelocity);
                const bool nearlyStill = linearSpeedSq < (kSleepLinearThreshold * kSleepLinearThreshold)
                                      && angularSpeedSq < (kSleepAngularThreshold * kSleepAngularThreshold);
                islandNearlyStill = islandNearlyStill && nearlyStill;

                for (const Manifold& m : manifolds_) {
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (m.a == id) other = m.b;
                    else if (m.b == id) other = m.a;
                    else continue;
                    if (other < bodies_.size() && bodies_[other].invMass != 0.0f && !visited[other]) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (const DistanceJoint& j : joints_) {
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == id) other = j.b;
                    else if (j.b == id) other = j.a;
                    else continue;
                    if (other < bodies_.size() && bodies_[other].invMass != 0.0f && !visited[other]) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (const HingeJoint& j : hingeJoints_) {
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == id) other = j.b;
                    else if (j.b == id) other = j.a;
                    else continue;
                    if (other < bodies_.size() && bodies_[other].invMass != 0.0f && !visited[other]) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }
            }

            if (islandNearlyStill) {
                for (std::uint32_t id : islandBodies) {
                    Body& body = bodies_[id];
                    ++body.sleepCounter;
                    if (body.sleepCounter >= kSleepFramesThreshold) {
                        body.isSleeping = true;
                        body.velocity = {0.0f, 0.0f, 0.0f};
                        body.angularVelocity = {0.0f, 0.0f, 0.0f};
                    }
                }
            } else {
                for (std::uint32_t id : islandBodies) {
                    Body& body = bodies_[id];
                    body.isSleeping = false;
                    body.sleepCounter = 0;
                }
            }
        }
    }

} // namespace minphys3d
