#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "minphys3d/broadphase/types.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/joints/types.hpp"
#include "minphys3d/narrowphase/dispatch.hpp"
#include "minphys3d/solver/types.hpp"

namespace minphys3d {

class World {
public:
    explicit World(Vec3 gravity = {0.0f, -9.81f, 0.0f}) : gravity_(gravity) {}

    void SetContactSolverConfig(const ContactSolverConfig& config) {
        contactSolverConfig_ = config;
    }

    const ContactSolverConfig& GetContactSolverConfig() const {
        return contactSolverConfig_;
    }

    std::uint32_t CreateBody(const Body& bodyDef) {
        Body body = bodyDef;
        body.RecomputeMassProperties();
        bodies_.push_back(body);

        BroadphaseProxy proxy;
        proxy.fatBox = ExpandAABB(body.ComputeAABB(), 0.1f);
        proxy.leaf = -1;
        proxy.valid = true;
        proxies_.push_back(proxy);
        EnsureProxyInTree(static_cast<std::uint32_t>(bodies_.size() - 1));

        return static_cast<std::uint32_t>(bodies_.size() - 1);
    }

    Body& GetBody(std::uint32_t id) {
        return bodies_.at(id);
    }

    const Body& GetBody(std::uint32_t id) const {
        return bodies_.at(id);
    }

    std::uint32_t CreateDistanceJoint(std::uint32_t a, std::uint32_t b, const Vec3& worldAnchorA, const Vec3& worldAnchorB, float stiffness = 1.0f, float damping = 0.1f) {
        DistanceJoint j;
        j.a = a;
        j.b = b;
        j.localAnchorA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), worldAnchorA - bodies_.at(a).position);
        j.localAnchorB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), worldAnchorB - bodies_.at(b).position);
        j.restLength = Length(worldAnchorB - worldAnchorA);
        j.stiffness = stiffness;
        j.damping = damping;
        joints_.push_back(j);
        return static_cast<std::uint32_t>(joints_.size() - 1);
    }

    std::uint32_t CreateHingeJoint(
        std::uint32_t a,
        std::uint32_t b,
        const Vec3& worldAnchor,
        const Vec3& worldAxis = {0.0f, 1.0f, 0.0f},
        bool enableLimits = false,
        float lowerAngle = 0.0f,
        float upperAngle = 0.0f,
        bool enableMotor = false,
        float motorSpeed = 0.0f,
        float maxMotorTorque = 0.0f) {
        HingeJoint j;
        j.a = a;
        j.b = b;
        j.localAnchorA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), worldAnchor - bodies_.at(a).position);
        j.localAnchorB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), worldAnchor - bodies_.at(b).position);
        const Vec3 axisN = Normalize(worldAxis);
        j.localAxisA = Rotate(Conjugate(Normalize(bodies_.at(a).orientation)), axisN);
        j.localAxisB = Rotate(Conjugate(Normalize(bodies_.at(b).orientation)), axisN);
        j.limitsEnabled = enableLimits;
        j.lowerAngle = lowerAngle;
        j.upperAngle = upperAngle;
        j.motorEnabled = enableMotor;
        j.motorSpeed = motorSpeed;
        j.maxMotorTorque = maxMotorTorque;
        hingeJoints_.push_back(j);
        return static_cast<std::uint32_t>(hingeJoints_.size() - 1);
    }

    void Step(float dt, int solverIterations = 8) {
        if (dt <= 0.0f) {
            return;
        }

        AssertBodyInvariants();
        previousContacts_ = contacts_;
        previousManifolds_ = manifolds_;

        const int substeps = ComputeSubsteps(dt);
        const float subDt = dt / static_cast<float>(substeps);

        for (int stepIndex = 0; stepIndex < substeps; ++stepIndex) {
            currentSubstepDt_ = subDt;
            if (stepIndex == 0) {
                previousContacts_ = contacts_;
                previousManifolds_ = manifolds_;
            }
            IntegrateForces(subDt);
            contacts_.clear();
            manifolds_.clear();
            UpdateBroadphaseProxies();
            GenerateContacts();
            BuildManifolds();
            BuildIslands();
            WarmStartContacts();
            WarmStartJoints();

            for (int i = 0; i < solverIterations; ++i) {
                SolveIslands();
            }

            ResolveTOIPipeline(subDt);
            IntegrateOrientation(subDt);
            SolveJointPositions();
            PositionalCorrection();
            UpdateSleeping();
            ClearAccumulators();
            previousContacts_ = contacts_;
            previousManifolds_ = manifolds_;
            AssertBodyInvariants();
        }
    }

    void AddForce(std::uint32_t id, const Vec3& force) {
        Body& b = bodies_.at(id);
        WakeBody(b);
        b.force += force;
    }

    void AddTorque(std::uint32_t id, const Vec3& torque) {
        Body& b = bodies_.at(id);
        WakeBody(b);
        b.torque += torque;
    }

    void AddForceAtPoint(std::uint32_t id, const Vec3& force, const Vec3& worldPoint) {
        Body& body = bodies_.at(id);
        WakeBody(body);
        body.force += force;
        const Vec3 r = worldPoint - body.position;
        body.torque += Cross(r, force);
    }

    std::size_t LastBroadphaseMovedProxyCount() const {
        return lastBroadphaseMovedProxyCount_;
    }

    std::size_t BroadphasePairCount() const {
        return ComputePotentialPairs().size();
    }

    std::size_t BruteForcePairCount() const {
        std::size_t count = 0;
        for (std::uint32_t i = 0; i < bodies_.size(); ++i) {
            const AABB ai = bodies_[i].ComputeAABB();
            for (std::uint32_t j = i + 1; j < bodies_.size(); ++j) {
                const Body& a = bodies_[i];
                const Body& b = bodies_[j];
                if ((a.invMass == 0.0f && b.invMass == 0.0f) || (a.isSleeping && b.isSleeping)) {
                    continue;
                }
                if (Overlaps(ai, bodies_[j].ComputeAABB())) {
                    ++count;
                }
            }
        }
        return count;
    }

private:
    static constexpr float kQuaternionNormalizationTolerance = 1e-3f;
    static constexpr std::size_t kMaxContactsPerManifold = 4;
    static constexpr float kWakeContactRelativeSpeedThreshold = 0.35f;
    static constexpr float kWakeContactPenetrationThreshold = 0.02f;
    static constexpr float kWakeJointRelativeSpeedThreshold = 0.15f;

    static bool IsFinite(float value) {
        return std::isfinite(value);
    }

    static bool IsFinite(const Vec3& v) {
        return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
    }

    static bool IsFinite(const Quat& q) {
        return IsFinite(q.w) && IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z);
    }

    static bool IsFinite(const Mat3& m) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (!IsFinite(m.m[row][col])) {
                    return false;
                }
            }
        }
        return true;
    }

    static bool IsZeroInertia(const Mat3& m) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (std::abs(m.m[row][col]) > kEpsilon) {
                    return false;
                }
            }
        }
        return true;
    }

    static void AssertBodyStateFinite(const Body& body) {
        assert(IsFinite(body.position));
        assert(IsFinite(body.orientation));
        assert(IsFinite(body.velocity));
        assert(IsFinite(body.angularVelocity));
    }

    static void AssertMassInertiaConsistency(const Body& body) {
        assert(IsFinite(body.mass) || body.invMass == 0.0f);
        assert(IsFinite(body.invMass));
        assert(IsFinite(body.invInertiaLocal));
        if (body.invMass == 0.0f) {
            assert(body.isStatic || !std::isfinite(body.mass));
            assert(IsZeroInertia(body.invInertiaLocal));
            return;
        }

        assert(body.mass > kEpsilon);
        assert(std::isfinite(body.mass));
        assert(std::abs(body.invMass - (1.0f / body.mass)) <= 1e-4f);
        assert(body.invInertiaLocal.m[0][0] >= -kEpsilon);
        assert(body.invInertiaLocal.m[1][1] >= -kEpsilon);
        assert(body.invInertiaLocal.m[2][2] >= -kEpsilon);
    }

    void AssertBodyInvariants() const {
        for (const Body& body : bodies_) {
            AssertBodyStateFinite(body);
            AssertMassInertiaConsistency(body);
        }
    }

    static void AssertQuaternionNormalized(const Quat& q) {
        const float lenSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        assert(IsFinite(lenSq));
        assert(std::abs(lenSq - 1.0f) <= kQuaternionNormalizationTolerance);
    }

    static AABB ExpandAABB(const AABB& aabb, float margin) {
        AABB out = aabb;
        out.min.x -= margin;
        out.min.y -= margin;
        out.min.z -= margin;
        out.max.x += margin;
        out.max.y += margin;
        out.max.z += margin;
        return out;
    }

    static bool Contains(const AABB& outer, const AABB& inner) {
        return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z
            && outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
    }

    static AABB MergeAABB(const AABB& a, const AABB& b) {
        AABB out;
        out.min = {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)};
        out.max = {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)};
        return out;
    }

    static float SurfaceArea(const AABB& aabb) {
        const Vec3 e = aabb.max - aabb.min;
        return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
    }

    void UpdateBroadphaseProxies() {
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

    void EnsureProxyInTree(std::uint32_t bodyId) {
        if (bodyId >= proxies_.size() || !proxies_[bodyId].valid) {
            return;
        }
        if (proxies_[bodyId].leaf < 0) {
            proxies_[bodyId].leaf = InsertLeaf(bodyId, proxies_[bodyId].fatBox);
        }
    }

    std::int32_t AllocateNode() {
        if (freeNode_ != -1) {
            const std::int32_t nodeId = freeNode_;
            freeNode_ = treeNodes_[nodeId].next;
            treeNodes_[nodeId] = TreeNode{};
            return nodeId;
        }
        treeNodes_.push_back(TreeNode{});
        return static_cast<std::int32_t>(treeNodes_.size() - 1);
    }

    void FreeNode(std::int32_t nodeId) {
        treeNodes_[nodeId].next = freeNode_;
        treeNodes_[nodeId].height = -1;
        treeNodes_[nodeId].left = -1;
        treeNodes_[nodeId].right = -1;
        treeNodes_[nodeId].parent = -1;
        treeNodes_[nodeId].bodyId = -1;
        freeNode_ = nodeId;
    }

    std::int32_t InsertLeaf(std::uint32_t bodyId, const AABB& fatBox) {
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

    void RemoveLeaf(std::int32_t leaf) {
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

    std::int32_t Balance(std::int32_t iA) {
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

    int ComputeSubsteps(float dt) const {
        float maxRatio = 0.0f;
        for (const Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            float characteristic = 1.0f;
            if (body.shape == ShapeType::Sphere) {
                characteristic = std::max(body.radius, 0.05f);
            } else if (body.shape == ShapeType::Box) {
                characteristic = std::max({body.halfExtents.x, body.halfExtents.y, body.halfExtents.z, 0.05f});
            } else if (body.shape == ShapeType::Capsule) {
                characteristic = std::max(body.radius, 0.05f);
            }
            const float travel = Length(body.velocity) * dt;
            maxRatio = std::max(maxRatio, travel / (characteristic * kMaxSubstepDistanceFactor));
        }
        return std::clamp(static_cast<int>(std::ceil(std::max(1.0f, maxRatio))), 1, 8);
    }

    struct TOIEvent {
        bool hit = false;
        float toi = 0.0f;
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        Vec3 normal{0.0f, 1.0f, 0.0f};
        Vec3 point{0.0f, 0.0f, 0.0f};
    };

    static float ClosestPointParameter(const Vec3& a, const Vec3& b, const Vec3& p) {
        const Vec3 ab = b - a;
        const float denom = Dot(ab, ab);
        if (denom <= kEpsilon) {
            return 0.0f;
        }
        return std::clamp(Dot(p - a, ab) / denom, 0.0f, 1.0f);
    }

    static float SmoothStep01(float t) {
        const float x = std::clamp(t, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }

    void AdvanceDynamicBodies(float dt) {
        if (dt <= 0.0f) {
            return;
        }
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += body.velocity * dt;
        }
    }

    void ResolveTOIImpact(const TOIEvent& hit) {
        Body& a = bodies_[hit.a];
        Body& b = bodies_[hit.b];
        const Vec3 n = Normalize(hit.normal);
        const Vec3 relativeVelocity = a.velocity - b.velocity;
        const float vn = Dot(relativeVelocity, n);
        if (vn < 0.0f) {
            const float invMassSum = a.invMass + b.invMass;
            if (invMassSum > kEpsilon) {
                float restitution = std::clamp(std::max(a.restitution, b.restitution), 0.0f, 1.0f);
                const float speedIntoContact = -vn;
                if ((speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.bounceVelocityThreshold)
                    || (speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.restitutionSuppressionSpeed)) {
                    restitution = 0.0f;
                }
                const float impulse = -(1.0f + restitution) * vn / invMassSum;
                const Vec3 impulseVector = n * impulse;
                if (a.invMass > 0.0f) {
                    a.velocity += impulseVector * a.invMass;
                }
                if (b.invMass > 0.0f) {
                    b.velocity -= impulseVector * b.invMass;
                }
            }
        }

        const float correctionSlop = 1e-4f;
        const float push = 4e-4f;
        const float invMassSum = a.invMass + b.invMass;
        if (invMassSum > kEpsilon) {
            const Vec3 correction = n * std::max(push, correctionSlop);
            if (a.invMass > 0.0f) {
                a.position += correction * (a.invMass / invMassSum);
            }
            if (b.invMass > 0.0f) {
                b.position -= correction * (b.invMass / invMassSum);
            }
        }
    }

    TOIEvent SweepSpherePlane(std::uint32_t sphereId, std::uint32_t planeId, float maxDt) const {
        const Body& s = bodies_[sphereId];
        const Body& p = bodies_[planeId];
        const Vec3 n = Normalize(p.planeNormal);
        const float d0 = Dot(n, s.position) - p.planeOffset - s.radius;
        const float vn = Dot(n, s.velocity);
        if (d0 <= 0.0f || vn >= -kEpsilon) {
            return {};
        }
        const float toi = -d0 / vn;
        if (toi < 0.0f || toi > maxDt) {
            return {};
        }
        TOIEvent hit;
        hit.hit = true;
        hit.toi = toi;
        hit.a = sphereId;
        hit.b = planeId;
        hit.normal = n;
        hit.point = (s.position + s.velocity * toi) - n * s.radius;
        return hit;
    }

    TOIEvent SweepSphereBox(std::uint32_t sphereId, std::uint32_t boxId, float maxDt) const {
        const Body& s = bodies_[sphereId];
        const Body& b = bodies_[boxId];
        const Quat invQ = Conjugate(Normalize(b.orientation));
        const Vec3 p0 = Rotate(invQ, s.position - b.position);
        const Vec3 vRel = Rotate(invQ, s.velocity - b.velocity);
        const Vec3 ext = b.halfExtents + Vec3{s.radius, s.radius, s.radius};

        float tEnter = 0.0f;
        float tExit = maxDt;
        int enteringAxis = -1;
        float enteringSign = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            const float p = (axis == 0) ? p0.x : (axis == 1) ? p0.y : p0.z;
            const float v = (axis == 0) ? vRel.x : (axis == 1) ? vRel.y : vRel.z;
            const float e = (axis == 0) ? ext.x : (axis == 1) ? ext.y : ext.z;

            if (std::abs(v) <= kEpsilon) {
                if (p < -e || p > e) {
                    return {};
                }
                continue;
            }

            float t1 = (-e - p) / v;
            float t2 = (e - p) / v;
            float sign = -1.0f;
            if (t1 > t2) {
                std::swap(t1, t2);
                sign = 1.0f;
            }
            if (t1 > tEnter) {
                tEnter = t1;
                enteringAxis = axis;
                enteringSign = sign;
            }
            tExit = std::min(tExit, t2);
            if (tEnter > tExit) {
                return {};
            }
        }

        if (tEnter < 0.0f || tEnter > maxDt || enteringAxis < 0) {
            return {};
        }

        Vec3 normalLocal{0.0f, 0.0f, 0.0f};
        if (enteringAxis == 0) {
            normalLocal.x = enteringSign;
        } else if (enteringAxis == 1) {
            normalLocal.y = enteringSign;
        } else {
            normalLocal.z = enteringSign;
        }

        TOIEvent hit;
        hit.hit = true;
        hit.toi = tEnter;
        hit.a = sphereId;
        hit.b = boxId;
        hit.normal = Rotate(b.orientation, normalLocal);
        const Vec3 centerAtHit = s.position + s.velocity * tEnter;
        hit.point = centerAtHit - hit.normal * s.radius;
        return hit;
    }

    TOIEvent SweepSphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId, float maxDt) const {
        const Body& s = bodies_[sphereId];
        const Body& c = bodies_[capsuleId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 segA = c.position - axis * c.halfHeight;
        const Vec3 segB = c.position + axis * c.halfHeight;
        const Vec3 vRel = s.velocity - c.velocity;
        const float combinedRadius = s.radius + c.radius;

        float t = 0.0f;
        for (int i = 0; i < 12 && t <= maxDt; ++i) {
            const Vec3 center = s.position + vRel * t;
            const float segT = ClosestPointParameter(segA, segB, center);
            const Vec3 closest = segA + (segB - segA) * segT;
            Vec3 delta = center - closest;
            float dist = Length(delta);
            if (dist <= combinedRadius + 1e-4f) {
                TOIEvent hit;
                hit.hit = true;
                hit.toi = std::clamp(t, 0.0f, maxDt);
                if (dist <= kEpsilon) {
                    delta = StableDirection(vRel, {{axis, -axis, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}});
                    dist = 1.0f;
                }
                hit.normal = delta * (1.0f / dist);
                hit.a = sphereId;
                hit.b = capsuleId;
                hit.point = center - hit.normal * s.radius;
                return hit;
            }

            const Vec3 n = delta / std::max(dist, kEpsilon);
            const float closingSpeed = Dot(vRel, n);
            if (closingSpeed >= -kEpsilon) {
                break;
            }
            const float dt = (dist - combinedRadius) / -closingSpeed;
            if (dt <= 1e-5f) {
                t += 1e-5f;
            } else {
                t += dt;
            }
        }
        return {};
    }

    TOIEvent FindEarliestTOI(float maxDt) const {
        TOIEvent earliest;
        earliest.toi = maxDt + 1.0f;

        for (std::uint32_t i = 0; i < bodies_.size(); ++i) {
            const Body& a = bodies_[i];
            if (a.shape != ShapeType::Sphere || a.invMass == 0.0f || a.isSleeping) {
                continue;
            }
            for (std::uint32_t j = 0; j < bodies_.size(); ++j) {
                if (i == j) {
                    continue;
                }
                const Body& b = bodies_[j];
                TOIEvent hit;
                if (b.shape == ShapeType::Plane) {
                    hit = SweepSpherePlane(i, j, maxDt);
                } else if (b.shape == ShapeType::Box) {
                    hit = SweepSphereBox(i, j, maxDt);
                } else if (b.shape == ShapeType::Capsule) {
                    hit = SweepSphereCapsule(i, j, maxDt);
                }
                if (!hit.hit) {
                    continue;
                }
                if (!earliest.hit || hit.toi < earliest.toi - 1e-6f || (std::abs(hit.toi - earliest.toi) <= 1e-6f && ((hit.a < earliest.a) || (hit.a == earliest.a && hit.b < earliest.b)))) {
                    earliest = hit;
                }
            }
        }

        return earliest.hit ? earliest : TOIEvent{};
    }

    void ResolveTOIPipeline(float dt) {
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

    static std::uint64_t PackFeatureKey(
        std::uint8_t kind,
        std::uint8_t referenceFace,
        std::uint8_t incidentFace,
        std::uint8_t referenceFeature,
        std::uint8_t incidentFeature,
        std::uint8_t clippedFeature) {
        return (static_cast<std::uint64_t>(kind) << 40)
             | (static_cast<std::uint64_t>(referenceFace) << 32)
             | (static_cast<std::uint64_t>(incidentFace) << 24)
             | (static_cast<std::uint64_t>(referenceFeature) << 16)
             | (static_cast<std::uint64_t>(incidentFeature) << 8)
             | static_cast<std::uint64_t>(clippedFeature);
    }

    static std::uint64_t MakeContactKey(std::uint32_t a, std::uint32_t b, std::uint64_t featureKey) {
        const std::uint32_t lo = std::min(a, b);
        const std::uint32_t hi = std::max(a, b);
        return (static_cast<std::uint64_t>(lo) << 48)
             ^ (static_cast<std::uint64_t>(hi) << 32)
             ^ featureKey;
    }

    static void WakeBody(Body& body) {
        if (body.invMass == 0.0f) {
            return;
        }
        body.isSleeping = false;
        body.sleepCounter = 0;
    }

    void WakeConnectedBodies(std::uint32_t start) {
        if (start >= bodies_.size()) {
            return;
        }
        if (bodies_[start].invMass == 0.0f) {
            return;
        }

        std::vector<bool> visited(bodies_.size(), false);
        std::vector<std::uint32_t> stack{start};
        visited[start] = true;
        while (!stack.empty()) {
            const std::uint32_t id = stack.back();
            stack.pop_back();
            WakeBody(bodies_[id]);

            for (const Manifold& m : manifolds_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (m.a == id) other = m.b;
                else if (m.b == id) other = m.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const DistanceJoint& j : joints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }

            for (const HingeJoint& j : hingeJoints_) {
                std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                if (j.a == id) other = j.b;
                else if (j.b == id) other = j.a;
                else continue;

                if (other < bodies_.size() && !visited[other] && bodies_[other].invMass != 0.0f) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            }
        }
    }

    static float ProjectBoxOntoAxis(const Body& box, const Vec3& axis) {
        const Vec3 u0 = Rotate(box.orientation, {1.0f, 0.0f, 0.0f});
        const Vec3 u1 = Rotate(box.orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 u2 = Rotate(box.orientation, {0.0f, 0.0f, 1.0f});
        return std::abs(Dot(axis, u0)) * box.halfExtents.x
             + std::abs(Dot(axis, u1)) * box.halfExtents.y
             + std::abs(Dot(axis, u2)) * box.halfExtents.z;
    }

    static Vec3 ClosestPointOnBox(const Body& box, const Vec3& worldPoint) {
        const Quat invQ = Conjugate(Normalize(box.orientation));
        const Vec3 local = Rotate(invQ, worldPoint - box.position);
        const Vec3 clamped = {
            std::clamp(local.x, -box.halfExtents.x, box.halfExtents.x),
            std::clamp(local.y, -box.halfExtents.y, box.halfExtents.y),
            std::clamp(local.z, -box.halfExtents.z, box.halfExtents.z),
        };
        return box.position + Rotate(box.orientation, clamped);
    }

    void IntegrateForces(float dt) {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            const Vec3 linearAcceleration = gravity_ + body.force * body.invMass;
            const Vec3 angularAcceleration = body.InvInertiaWorld() * body.torque;

            body.velocity += linearAcceleration * dt;
            body.angularVelocity += angularAcceleration * dt;
        }
    }

    void IntegrateVelocities(float dt) {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += body.velocity * dt;
        }
    }

    void IntegrateOrientation(float dt) {
        for (Body& body : bodies_) {
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            const Quat omega{0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z};
            const Quat dq = (omega * body.orientation) * (0.5f * dt);
            body.orientation = Normalize(body.orientation + dq);
            AssertQuaternionNormalized(body.orientation);
        }
    }

    void AddContact(std::uint32_t a, std::uint32_t b, const Vec3& normal, const Vec3& point, float penetration, std::uint64_t featureKey = 0) {
        Contact c;
        c.a = a;
        c.b = b;
        c.normal = normal;
        c.point = point;
        c.penetration = std::max(penetration, 0.0f);
        c.key = MakeContactKey(a, b, featureKey);
        assert(IsFinite(c.normal));
        assert(IsFinite(c.point));
        assert(IsFinite(c.penetration));
        assert(c.penetration >= 0.0f);

        for (const Contact& old : previousContacts_) {
            if (old.key == c.key) {
                c.normalImpulseSum = old.normalImpulseSum;
                c.tangentImpulseSum = old.tangentImpulseSum;
                break;
            }
        }

        contacts_.push_back(c);
        const Body& bodyA = bodies_[a];
        const Body& bodyB = bodies_[b];
        const float relSpeed = Length(bodyB.velocity - bodyA.velocity);
        const bool contactTouchesSleepingBody = (bodyA.isSleeping && !bodyB.isSleeping)
                                             || (!bodyA.isSleeping && bodyB.isSleeping);
        const bool strongContact = relSpeed >= kWakeContactRelativeSpeedThreshold
                                || c.penetration >= kWakeContactPenetrationThreshold
                                || contactTouchesSleepingBody;
        if (strongContact) {
            WakeConnectedBodies(a);
            WakeConnectedBodies(b);
        }
    }

    static int FindBlockSlot(const Manifold& manifold, std::uint64_t contactKey) {
        for (int slot = 0; slot < 2; ++slot) {
            if (manifold.blockSlotValid[slot] && manifold.blockContactKeys[slot] == contactKey) {
                return slot;
            }
        }
        return -1;
    }

    static int FindFirstFreeBlockSlot(const std::array<bool, 2>& slotOccupied) {
        for (int slot = 0; slot < 2; ++slot) {
            if (!slotOccupied[slot]) {
                return slot;
            }
        }
        return -1;
    }

    static int EnsureBlockSlotForContact(
        Manifold& manifold,
        const Contact& contact,
        std::array<bool, 2>& slotOccupied) {
        int slot = FindBlockSlot(manifold, contact.key);
        if (slot >= 0) {
            slotOccupied[slot] = true;
            return slot;
        }

        slot = FindFirstFreeBlockSlot(slotOccupied);
        if (slot < 0) {
            return -1;
        }

        manifold.blockSlotValid[slot] = true;
        manifold.blockContactKeys[slot] = contact.key;
        manifold.blockNormalImpulseSum[slot] = std::max(contact.normalImpulseSum, 0.0f);
        slotOccupied[slot] = true;
        return slot;
    }

    static void RefreshManifoldBlockCache(Manifold& manifold) {
        std::array<bool, 2> slotOccupied{false, false};
        std::array<int, 2> slotToContactIndex{-1, -1};

        for (std::size_t i = 0; i < manifold.contacts.size(); ++i) {
            Contact& contact = manifold.contacts[i];
            const int slot = EnsureBlockSlotForContact(manifold, contact, slotOccupied);
            if (slot >= 0 && slotToContactIndex[slot] < 0) {
                slotToContactIndex[slot] = static_cast<int>(i);
            }
        }

        for (int slot = 0; slot < 2; ++slot) {
            if (!slotOccupied[slot]) {
                manifold.blockSlotValid[slot] = false;
                manifold.blockContactKeys[slot] = 0u;
                manifold.blockNormalImpulseSum[slot] = 0.0f;
                continue;
            }
            if (slotToContactIndex[slot] < 0) {
                continue;
            }
            manifold.contacts[slotToContactIndex[slot]].normalImpulseSum = manifold.blockNormalImpulseSum[slot];
        }

        if (manifold.contacts.size() == 2 && slotToContactIndex[0] == 1 && slotToContactIndex[1] == 0) {
            std::swap(manifold.contacts[0], manifold.contacts[1]);
        }
    }

    void BuildManifolds() {
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
                m.contacts.push_back(c);
                manifolds_.push_back(m);
            }
        }

        for (Manifold& m : manifolds_) {
            const Manifold* previous = nullptr;
            for (const Manifold& old : previousManifolds_) {
                if (old.pairKey() == m.pairKey()) {
                    previous = &old;
                    m.blockNormalImpulseSum = old.blockNormalImpulseSum;
                    m.blockContactKeys = old.blockContactKeys;
                    m.blockSlotValid = old.blockSlotValid;
                    for (Contact& c : m.contacts) {
                        for (const Contact& oldc : old.contacts) {
                            if (oldc.key == c.key) {
                                c.normalImpulseSum = oldc.normalImpulseSum;
                                c.tangentImpulseSum = oldc.tangentImpulseSum;
                                break;
                            }
                        }
                    }
                    break;
                }
            }

            if (previous == nullptr) {
                m.blockNormalImpulseSum = {0.0f, 0.0f};
                m.blockContactKeys = {0u, 0u};
                m.blockSlotValid = {false, false};
            }

            RefreshManifoldBlockCache(m);
        }

        std::unordered_map<std::uint64_t, std::size_t> manifoldCountByPair;
        for (const Manifold& m : manifolds_) {
            const std::uint64_t key = m.pairKey();
            ++manifoldCountByPair[key];
            assert(manifoldCountByPair[key] == 1);
            assert(m.contacts.size() <= kMaxContactsPerManifold);
        }
    }

    void BuildIslands() {
        islands_.clear();
        std::vector<bool> visited(bodies_.size(), false);

        for (std::uint32_t start = 0; start < bodies_.size(); ++start) {
            if (visited[start]) {
                continue;
            }
            if (bodies_[start].invMass == 0.0f || bodies_[start].isSleeping) {
                visited[start] = true;
                continue;
            }

            Island island;
            std::vector<std::uint32_t> stack{start};
            visited[start] = true;

            while (!stack.empty()) {
                const std::uint32_t bodyId = stack.back();
                stack.pop_back();
                island.bodies.push_back(bodyId);

                for (std::size_t mi = 0; mi < manifolds_.size(); ++mi) {
                    const Manifold& m = manifolds_[mi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (m.a == bodyId) other = m.b;
                    else if (m.b == bodyId) other = m.a;
                    else continue;

                    if (std::find(island.manifolds.begin(), island.manifolds.end(), mi) == island.manifolds.end()) {
                        island.manifolds.push_back(mi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t ji = 0; ji < joints_.size(); ++ji) {
                    const DistanceJoint& j = joints_[ji];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.joints.begin(), island.joints.end(), ji) == island.joints.end()) {
                        island.joints.push_back(ji);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }

                for (std::size_t hi = 0; hi < hingeJoints_.size(); ++hi) {
                    const HingeJoint& j = hingeJoints_[hi];
                    std::uint32_t other = std::numeric_limits<std::uint32_t>::max();
                    if (j.a == bodyId) other = j.b;
                    else if (j.b == bodyId) other = j.a;
                    else continue;

                    if (std::find(island.hinges.begin(), island.hinges.end(), hi) == island.hinges.end()) {
                        island.hinges.push_back(hi);
                    }

                    if (!visited[other] && bodies_[other].invMass != 0.0f && !bodies_[other].isSleeping) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                }
            }

            if (!island.bodies.empty()) {
                islands_.push_back(island);
            }
        }
    }

    std::vector<Pair> ComputePotentialPairs() const {
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

#ifndef NDEBUG
    std::vector<Pair> ComputePotentialPairsBruteForce() const {
        std::vector<Pair> pairs;
        if (bodies_.size() < 2) {
            return pairs;
        }
        pairs.reserve((bodies_.size() * (bodies_.size() - 1)) / 2);
        for (std::uint32_t i = 0; i < proxies_.size(); ++i) {
            const BroadphaseProxy& proxyA = proxies_[i];
            if (!proxyA.valid || proxyA.leaf < 0) {
                continue;
            }
            for (std::uint32_t j = i + 1; j < proxies_.size(); ++j) {
                const BroadphaseProxy& proxyB = proxies_[j];
                if (!proxyB.valid || proxyB.leaf < 0 || !Overlaps(proxyA.fatBox, proxyB.fatBox)) {
                    continue;
                }
                const Body& a = bodies_[i];
                const Body& b = bodies_[j];
                if ((a.invMass == 0.0f && b.invMass == 0.0f) || (a.isSleeping && b.isSleeping)) {
                    continue;
                }
                pairs.push_back({i, j});
            }
        }
        return pairs;
    }

    static std::unordered_set<std::uint64_t> PairSetFromPairs(const std::vector<Pair>& pairs) {
        std::unordered_set<std::uint64_t> pairSet;
        pairSet.reserve(pairs.size());
        for (const Pair& pair : pairs) {
            pairSet.insert((static_cast<std::uint64_t>(pair.a) << 32) | pair.b);
        }
        return pairSet;
    }
#endif

    void GenerateContacts() {
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

    static Vec3 ClampPointToExtents(const Vec3& p, const Vec3& extents) {
        return {
            std::clamp(p.x, -extents.x, extents.x),
            std::clamp(p.y, -extents.y, extents.y),
            std::clamp(p.z, -extents.z, extents.z),
        };
    }

    static Vec3 StableDirection(const Vec3& primary, const std::array<Vec3, 4>& fallbacks) {
        if (LengthSquared(primary) > kEpsilon * kEpsilon) {
            return Normalize(primary);
        }
        for (const Vec3& candidate : fallbacks) {
            if (LengthSquared(candidate) > kEpsilon * kEpsilon) {
                return Normalize(candidate);
            }
        }
        return {0.0f, 1.0f, 0.0f};
    }

    static std::pair<float, float> ClosestSegmentParameters(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const Vec3 r = p1 - p2;
        const float aLen = Dot(d1, d1);
        const float eLen = Dot(d2, d2);
        const float f = Dot(d2, r);

        float s = 0.0f;
        float t = 0.0f;
        if (aLen <= kEpsilon && eLen <= kEpsilon) {
            return {0.0f, 0.0f};
        }
        if (aLen <= kEpsilon) {
            t = std::clamp(f / eLen, 0.0f, 1.0f);
            return {0.0f, t};
        }

        const float cTerm = Dot(d1, r);
        if (eLen <= kEpsilon) {
            s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
            return {s, 0.0f};
        }

        const float bDot = Dot(d1, d2);
        const float denom = aLen * eLen - bDot * bDot;
        if (std::abs(denom) > kEpsilon) {
            s = std::clamp((bDot * f - cTerm * eLen) / denom, 0.0f, 1.0f);
        } else {
            s = 0.0f;
        }

        t = (bDot * s + f) / eLen;
        if (t < 0.0f) {
            t = 0.0f;
            s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
        } else if (t > 1.0f) {
            t = 1.0f;
            s = std::clamp((bDot - cTerm) / aLen, 0.0f, 1.0f);
        }

        return {s, t};
    }

    struct SegmentBoxClosest {
        float t = 0.0f;
        Vec3 segmentPoint{};
        Vec3 boxPoint{};
        float distSq = 0.0f;
    };

    static SegmentBoxClosest ClosestSegmentPointToBox(const Vec3& segA, const Vec3& segB, const Vec3& extents) {
        const Vec3 d = segB - segA;
        auto eval = [&](float t) {
            SegmentBoxClosest result;
            result.t = std::clamp(t, 0.0f, 1.0f);
            result.segmentPoint = segA + d * result.t;
            result.boxPoint = ClampPointToExtents(result.segmentPoint, extents);
            result.distSq = LengthSquared(result.segmentPoint - result.boxPoint);
            return result;
        };

        SegmentBoxClosest best = eval(0.0f);
        auto consider = [&](float t) {
            const SegmentBoxClosest candidate = eval(t);
            if (candidate.distSq < best.distSq) {
                best = candidate;
            }
        };

        consider(1.0f);

        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            const float p0 = (axisIndex == 0) ? segA.x : (axisIndex == 1 ? segA.y : segA.z);
            const float vd = (axisIndex == 0) ? d.x : (axisIndex == 1 ? d.y : d.z);
            const float extent = (axisIndex == 0) ? extents.x : (axisIndex == 1 ? extents.y : extents.z);
            if (std::abs(vd) <= kEpsilon) {
                continue;
            }
            consider((-extent - p0) / vd);
            consider(( extent - p0) / vd);
        }

        float lo = 0.0f;
        float hi = 1.0f;
        for (int i = 0; i < 32; ++i) {
            const float m1 = lo + (hi - lo) / 3.0f;
            const float m2 = hi - (hi - lo) / 3.0f;
            if (eval(m1).distSq < eval(m2).distSq) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        consider(0.5f * (lo + hi));

        return best;
    }

    void SphereSphere(std::uint32_t ia, std::uint32_t ib) {
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

    void SpherePlane(std::uint32_t sphereId, std::uint32_t planeId) {
        const Body& s = bodies_[sphereId];
        const Body& p = bodies_[planeId];
        const Vec3 n = Normalize(p.planeNormal);
        const float signedDistance = Dot(n, s.position) - p.planeOffset;
        if (signedDistance >= s.radius) {
            return;
        }
        const std::uint64_t featureKey = PackFeatureKey(2, 0, 0, 0, 0, 0);
        AddContact(sphereId, planeId, -n, s.position - n * s.radius, s.radius - signedDistance, featureKey);
    }

    void CapsulePlane(std::uint32_t capsuleId, std::uint32_t planeId) {
        const Body& c = bodies_[capsuleId];
        const Body& p = bodies_[planeId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 ends[2] = {c.position - axis * c.halfHeight, c.position + axis * c.halfHeight};
        const Vec3 n = Normalize(p.planeNormal);
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const Vec3& center = ends[endpoint];
            const float signedDistance = Dot(n, center) - p.planeOffset;
            if (signedDistance < c.radius) {
                const std::uint64_t featureKey = PackFeatureKey(3, 0, 0, 0, static_cast<std::uint8_t>(endpoint), 0);
                AddContact(capsuleId, planeId, -n, center - n * c.radius, c.radius - signedDistance, featureKey);
            }
        }
    }

    void SphereCapsule(std::uint32_t sphereId, std::uint32_t capsuleId) {
        const Body& s = bodies_[sphereId];
        const Body& c = bodies_[capsuleId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 a = c.position - axis * c.halfHeight;
        const Vec3 b = c.position + axis * c.halfHeight;
        const Vec3 ab = b - a;
        const float denom = Dot(ab, ab);
        float t = 0.0f;
        if (denom > kEpsilon) {
            t = std::clamp(Dot(s.position - a, ab) / denom, 0.0f, 1.0f);
        }
        const Vec3 closest = a + ab * t;
        const Vec3 delta = closest - s.position;
        const float distSq = LengthSquared(delta);
        const float radiusSum = s.radius + c.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }
        const float dist = std::sqrt(std::max(distSq, kEpsilon));
        const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{0.0f, 1.0f, 0.0f};
        const std::uint8_t capsuleFeature = (t <= 1e-3f) ? 0u : ((t >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureKey = PackFeatureKey(4, 0, 0, 0, capsuleFeature, 0);
        AddContact(sphereId, capsuleId, normal, s.position + normal * s.radius, radiusSum - dist, featureKey);
    }

    void CapsuleCapsule(std::uint32_t aId, std::uint32_t bId) {
        const Body& a = bodies_[aId];
        const Body& b = bodies_[bId];

        const Vec3 axisA = Normalize(Rotate(a.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 axisB = Normalize(Rotate(b.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 p1 = a.position - axisA * a.halfHeight;
        const Vec3 q1 = a.position + axisA * a.halfHeight;
        const Vec3 p2 = b.position - axisB * b.halfHeight;
        const Vec3 q2 = b.position + axisB * b.halfHeight;

        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const auto [s, t] = ClosestSegmentParameters(p1, q1, p2, q2);

        const Vec3 c1 = p1 + d1 * s;
        const Vec3 c2 = p2 + d2 * t;
        const Vec3 delta = c2 - c1;
        const float distSq = LengthSquared(delta);
        const float radiusSum = a.radius + b.radius;
        if (distSq > radiusSum * radiusSum) {
            return;
        }

        const float dist = std::sqrt(std::max(distSq, 0.0f));
        const Vec3 centerDelta = b.position - a.position;
        Vec3 normal = StableDirection(delta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
        if (Dot(normal, centerDelta) < 0.0f) {
            normal = -normal;
        }

        const Vec3 point = c1 + normal * a.radius;
        const std::uint8_t featureA = (s <= 1e-3f) ? 0u : ((s >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint8_t featureB = (t <= 1e-3f) ? 0u : ((t >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureKey = PackFeatureKey(5, 0, 0, featureA, featureB, 0);
        AddContact(aId, bId, normal, point, radiusSum - dist, featureKey);

        const float parallelFactor = std::abs(Dot(axisA, axisB));
        if (parallelFactor > 0.98f && featureA == 2u && featureB == 2u) {
            auto addProjectedEndpointContact = [&](const Vec3& endpointA, std::uint8_t endpointFeature) {
                const auto [sProj, tProj] = ClosestSegmentParameters(endpointA, endpointA, p2, q2);
                (void)sProj;
                const Vec3 onB = p2 + d2 * tProj;
                const Vec3 endpointDelta = onB - endpointA;
                const float endpointDistSq = LengthSquared(endpointDelta);
                if (endpointDistSq > radiusSum * radiusSum) {
                    return;
                }
                Vec3 endpointNormal = StableDirection(endpointDelta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
                if (Dot(endpointNormal, centerDelta) < 0.0f) {
                    endpointNormal = -endpointNormal;
                }
                const float endpointDist = std::sqrt(std::max(endpointDistSq, 0.0f));
                const std::uint8_t featureBProj = (tProj <= 1e-3f) ? 0u : ((tProj >= 1.0f - 1e-3f) ? 1u : 2u);
                const std::uint64_t manifoldFeatureKey = PackFeatureKey(5, 0, 0, endpointFeature, featureBProj, 1);
                AddContact(aId, bId, endpointNormal, endpointA + endpointNormal * a.radius, radiusSum - endpointDist, manifoldFeatureKey);
            };

            addProjectedEndpointContact(p1, 0u);
            addProjectedEndpointContact(q1, 1u);
        }
    }

    void CapsuleBox(std::uint32_t capsuleId, std::uint32_t boxId) {
        const Body& c = bodies_[capsuleId];
        const Body& b = bodies_[boxId];
        const Vec3 axis = Normalize(Rotate(c.orientation, {0.0f, 1.0f, 0.0f}));
        const Vec3 segAWorld = c.position - axis * c.halfHeight;
        const Vec3 segBWorld = c.position + axis * c.halfHeight;

        const Quat invBoxOrientation = Conjugate(Normalize(b.orientation));
        const Vec3 segA = Rotate(invBoxOrientation, segAWorld - b.position);
        const Vec3 segB = Rotate(invBoxOrientation, segBWorld - b.position);
        const SegmentBoxClosest closest = ClosestSegmentPointToBox(segA, segB, b.halfExtents);
        if (closest.distSq > c.radius * c.radius) {
            return;
        }

        const Vec3 localDelta = closest.segmentPoint - closest.boxPoint;
        const float dist = std::sqrt(std::max(closest.distSq, 0.0f));
        Vec3 normalLocal = StableDirection(localDelta, {closest.segmentPoint, segB - segA, axis, {0.0f, 1.0f, 0.0f}});
        if (closest.distSq <= kEpsilon * kEpsilon) {
            const float dx = b.halfExtents.x - std::abs(closest.segmentPoint.x);
            const float dy = b.halfExtents.y - std::abs(closest.segmentPoint.y);
            const float dz = b.halfExtents.z - std::abs(closest.segmentPoint.z);
            if (dx <= dy && dx <= dz) {
                normalLocal = {(closest.segmentPoint.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};
            } else if (dy <= dz) {
                normalLocal = {0.0f, (closest.segmentPoint.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
            } else {
                normalLocal = {0.0f, 0.0f, (closest.segmentPoint.z >= 0.0f) ? 1.0f : -1.0f};
            }
        }
        const Vec3 normalWorld = Rotate(b.orientation, normalLocal);
        const Vec3 pointWorld = b.position + Rotate(b.orientation, closest.boxPoint);
        const float segT = closest.t;
        const std::uint8_t capsuleFeature = (segT <= 1e-3f) ? 0u : ((segT >= 1.0f - 1e-3f) ? 1u : 2u);
        const std::uint64_t featureKey = PackFeatureKey(6, 0, 0, 0, capsuleFeature, 0);
        AddContact(capsuleId, boxId, -normalWorld, pointWorld, c.radius - dist, featureKey);
    }

    void BoxPlane(std::uint32_t boxId, std::uint32_t planeId) {
        const Body& box = bodies_[boxId];
        const Body& plane = bodies_[planeId];
        const Vec3 n = Normalize(plane.planeNormal);

        const Vec3 axes[3] = {
            Rotate(box.orientation, {1.0f, 0.0f, 0.0f}),
            Rotate(box.orientation, {0.0f, 1.0f, 0.0f}),
            Rotate(box.orientation, {0.0f, 0.0f, 1.0f}),
        };

        int added = 0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Vec3 local = {
                        sx * box.halfExtents.x,
                        sy * box.halfExtents.y,
                        sz * box.halfExtents.z,
                    };
                    const Vec3 worldPoint = box.position
                        + axes[0] * local.x
                        + axes[1] * local.y
                        + axes[2] * local.z;
                    const float signedDistance = Dot(n, worldPoint) - plane.planeOffset;
                    if (signedDistance < 0.0f) {
                        const std::uint8_t vertexId = static_cast<std::uint8_t>((sx > 0 ? 1 : 0) | ((sy > 0 ? 1 : 0) << 1) | ((sz > 0 ? 1 : 0) << 2));
                        const std::uint64_t featureKey = PackFeatureKey(7, 0, 0, vertexId, 0, 0);
                        AddContact(boxId, planeId, -n, worldPoint, -signedDistance, featureKey);
                        ++added;
                        if (added >= 4) {
                            return;
                        }
                    }
                }
            }
        }
    }

    void SphereBox(std::uint32_t sphereId, std::uint32_t boxId) {
        const Body& s = bodies_[sphereId];
        const Body& b = bodies_[boxId];

        const Quat invBoxOrientation = Conjugate(Normalize(b.orientation));
        const Vec3 sphereCenterLocal = Rotate(invBoxOrientation, s.position - b.position);

        const Vec3 closestLocal = {
            std::clamp(sphereCenterLocal.x, -b.halfExtents.x, b.halfExtents.x),
            std::clamp(sphereCenterLocal.y, -b.halfExtents.y, b.halfExtents.y),
            std::clamp(sphereCenterLocal.z, -b.halfExtents.z, b.halfExtents.z),
        };

        const Vec3 deltaLocal = sphereCenterLocal - closestLocal;
        const float distSq = LengthSquared(deltaLocal);
        if (distSq > s.radius * s.radius) {
            return;
        }

        Vec3 normalWorld{};
        Vec3 pointWorld{};
        float penetration = 0.0f;

        if (distSq > kEpsilon) {
            const float dist = std::sqrt(distSq);
            const Vec3 normalLocal = deltaLocal / dist;
            normalWorld = Rotate(b.orientation, normalLocal);
            pointWorld = b.position + Rotate(b.orientation, closestLocal);
            penetration = s.radius - dist;
        } else {
            const float dx = b.halfExtents.x - std::abs(sphereCenterLocal.x);
            const float dy = b.halfExtents.y - std::abs(sphereCenterLocal.y);
            const float dz = b.halfExtents.z - std::abs(sphereCenterLocal.z);

            Vec3 normalLocal;
            if (dx <= dy && dx <= dz) {
                normalLocal = {(sphereCenterLocal.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};
                penetration = s.radius + dx;
            } else if (dy <= dz) {
                normalLocal = {0.0f, (sphereCenterLocal.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
                penetration = s.radius + dy;
            } else {
                normalLocal = {0.0f, 0.0f, (sphereCenterLocal.z >= 0.0f) ? 1.0f : -1.0f};
                penetration = s.radius + dz;
            }

            const Vec3 facePointLocal = {
                normalLocal.x != 0.0f ? normalLocal.x * b.halfExtents.x : sphereCenterLocal.x,
                normalLocal.y != 0.0f ? normalLocal.y * b.halfExtents.y : sphereCenterLocal.y,
                normalLocal.z != 0.0f ? normalLocal.z * b.halfExtents.z : sphereCenterLocal.z,
            };

            normalWorld = Rotate(b.orientation, normalLocal);
            pointWorld = b.position + Rotate(b.orientation, facePointLocal);
        }

        std::uint8_t referenceFace = 0;
        std::uint8_t incidentFeature = 0;
        const Vec3 localPoint = Rotate(invBoxOrientation, pointWorld - b.position);
        const float dx = std::abs(std::abs(localPoint.x) - b.halfExtents.x);
        const float dy = std::abs(std::abs(localPoint.y) - b.halfExtents.y);
        const float dz = std::abs(std::abs(localPoint.z) - b.halfExtents.z);
        if (dx <= dy && dx <= dz) {
            referenceFace = static_cast<std::uint8_t>((localPoint.x >= 0.0f) ? 0 : 1);
        } else if (dy <= dz) {
            referenceFace = static_cast<std::uint8_t>((localPoint.y >= 0.0f) ? 2 : 3);
        } else {
            referenceFace = static_cast<std::uint8_t>((localPoint.z >= 0.0f) ? 4 : 5);
        }
        incidentFeature = 0;
        const std::uint64_t featureKey = PackFeatureKey(8, referenceFace, 0, 0, incidentFeature, 0);
        AddContact(sphereId, boxId, -normalWorld, pointWorld, penetration, featureKey);
    }

    void BoxBox(std::uint32_t aId, std::uint32_t bId) {
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

    void WarmStartContacts() {
        for (Manifold& m : manifolds_) {
            for (Contact& c : m.contacts) {
                if (c.normalImpulseSum == 0.0f && c.tangentImpulseSum == 0.0f) {
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

                const Vec3 impulse = c.normalImpulseSum * c.normal + c.tangentImpulseSum * tangent;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
            }
        }
    }

    void WarmStartJoints() {
        for (DistanceJoint& j : joints_) {
            if (j.impulseSum == 0.0f) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 delta = (b.position + rb) - (a.position + ra);
            const float len = Length(delta);
            if (len <= kEpsilon) {
                continue;
            }
            const Vec3 n = delta / len;
            ApplyImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), ra, rb, j.impulseSum * n);
        }

        for (HingeJoint& j : hingeJoints_) {
            const float linearSum = std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ);
            const float angularSum = std::abs(j.angularImpulse1) + std::abs(j.angularImpulse2) + std::abs(j.motorImpulseSum);
            if (linearSum + angularSum <= kEpsilon) {
                continue;
            }
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 impulse{j.impulseX, j.impulseY, j.impulseZ};
            ApplyImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), ra, rb, impulse);

            const Vec3 axisA = Normalize(Rotate(a.orientation, j.localAxisA));
            Vec3 t1 = Cross(axisA, {1.0f, 0.0f, 0.0f});
            if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axisA, {0.0f, 0.0f, 1.0f});
            t1 = Normalize(t1);
            const Vec3 t2 = Normalize(Cross(axisA, t1));
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.angularImpulse1 * t1);
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.angularImpulse2 * t2);
            ApplyAngularImpulse(a, b, a.InvInertiaWorld(), b.InvInertiaWorld(), j.motorImpulseSum * axisA);
        }
    }

    void SolveNormalScalar(Contact& c) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];

        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = c.point - a.position;
        const Vec3 rb = c.point - b.position;

        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const Vec3 relativeVelocity = vb - va;
        const float separatingVelocity = Dot(relativeVelocity, c.normal);

        const float speedIntoContact = -separatingVelocity;
        float restitution = std::min(a.restitution, b.restitution);
        if ((speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.bounceVelocityThreshold)
            || (speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.restitutionSuppressionSpeed)) {
            restitution = 0.0f;
        }
        const Vec3 raCrossN = Cross(ra, c.normal);
        const Vec3 rbCrossN = Cross(rb, c.normal);
        const float angularTermA = Dot(raCrossN, invIA * raCrossN);
        const float angularTermB = Dot(rbCrossN, invIB * rbCrossN);
        const float normalMass = a.invMass + b.invMass + angularTermA + angularTermB;
        if (normalMass <= kEpsilon) {
            return;
        }

        float biasTerm = 0.0f;
        const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
        if (contactSolverConfig_.useSplitImpulse) {
            if (penetrationError > 0.0f) {
                const float invMassSum = a.invMass + b.invMass;
                if (invMassSum > kEpsilon) {
                    const float correctionMagnitude = contactSolverConfig_.splitImpulseCorrectionFactor * penetrationError / invMassSum;
                    const Vec3 correction = correctionMagnitude * c.normal;
                    if (!a.isSleeping) {
                        a.position -= correction * a.invMass;
                    }
                    if (!b.isSleeping) {
                        b.position += correction * b.invMass;
                    }
                }
            }
        } else if (currentSubstepDt_ > kEpsilon) {
            biasTerm = (contactSolverConfig_.penetrationBiasFactor * penetrationError) / currentSubstepDt_;
        }

        float lambdaN = -(1.0f + restitution) * separatingVelocity / normalMass;
        if (biasTerm > 0.0f) {
            lambdaN += biasTerm / normalMass;
        }
        const float oldNormalImpulse = c.normalImpulseSum;
        c.normalImpulseSum = std::max(0.0f, c.normalImpulseSum + lambdaN);
        lambdaN = c.normalImpulseSum - oldNormalImpulse;
        ApplyImpulse(a, b, invIA, invIB, ra, rb, lambdaN * c.normal);
    }

    bool SolveNormalBlock2(Manifold& manifold) {
        if (manifold.contacts.size() != 2) {
            return false;
        }

        Contact& c0 = manifold.contacts[0];
        Contact& c1 = manifold.contacts[1];
        Body& a = bodies_[c0.a];
        Body& b = bodies_[c0.b];

        const Vec3 manifoldNormal = Normalize(manifold.normal);
        if (LengthSquared(manifoldNormal) <= kEpsilon) {
            return false;
        }
        if (Dot(c0.normal, manifoldNormal) < 0.95f || Dot(c1.normal, manifoldNormal) < 0.95f) {
            return false;
        }

        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();
        const Vec3 ra0 = c0.point - a.position;
        const Vec3 rb0 = c0.point - b.position;
        const Vec3 ra1 = c1.point - a.position;
        const Vec3 rb1 = c1.point - b.position;

        const Vec3 ra0CrossN = Cross(ra0, manifoldNormal);
        const Vec3 rb0CrossN = Cross(rb0, manifoldNormal);
        const Vec3 ra1CrossN = Cross(ra1, manifoldNormal);
        const Vec3 rb1CrossN = Cross(rb1, manifoldNormal);

        const float k11 = a.invMass + b.invMass + Dot(ra0CrossN, invIA * ra0CrossN) + Dot(rb0CrossN, invIB * rb0CrossN);
        const float k22 = a.invMass + b.invMass + Dot(ra1CrossN, invIA * ra1CrossN) + Dot(rb1CrossN, invIB * rb1CrossN);
        const float k12 = a.invMass + b.invMass + Dot(ra0CrossN, invIA * ra1CrossN) + Dot(rb0CrossN, invIB * rb1CrossN);
        const float k21 = k12;

        const float det = k11 * k22 - k12 * k21;
        if (k11 <= kEpsilon || k22 <= kEpsilon || std::abs(det) <= 1e-8f) {
            return false;
        }

        const Vec3 va0 = a.velocity + Cross(a.angularVelocity, ra0);
        const Vec3 vb0 = b.velocity + Cross(b.angularVelocity, rb0);
        const Vec3 va1 = a.velocity + Cross(a.angularVelocity, ra1);
        const Vec3 vb1 = b.velocity + Cross(b.angularVelocity, rb1);
        const float vn0 = Dot(vb0 - va0, manifoldNormal);
        const float vn1 = Dot(vb1 - va1, manifoldNormal);

        const auto computeRhs = [&](Contact& c, float separatingVelocity) {
            const float speedIntoContact = -separatingVelocity;
            float restitution = std::min(a.restitution, b.restitution);
            if ((speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.bounceVelocityThreshold)
                || (speedIntoContact > 0.0f && speedIntoContact < contactSolverConfig_.restitutionSuppressionSpeed)) {
                restitution = 0.0f;
            }

            float biasTerm = 0.0f;
            const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
            if (contactSolverConfig_.useSplitImpulse) {
                if (penetrationError > 0.0f) {
                    const float invMassSum = a.invMass + b.invMass;
                    if (invMassSum > kEpsilon) {
                        const float correctionMagnitude = contactSolverConfig_.splitImpulseCorrectionFactor * penetrationError / invMassSum;
                        const Vec3 correction = correctionMagnitude * manifoldNormal;
                        if (!a.isSleeping) {
                            a.position -= correction * a.invMass;
                        }
                        if (!b.isSleeping) {
                            b.position += correction * b.invMass;
                        }
                    }
                }
            } else if (currentSubstepDt_ > kEpsilon) {
                biasTerm = (contactSolverConfig_.penetrationBiasFactor * penetrationError) / currentSubstepDt_;
            }

            return -(1.0f + restitution) * separatingVelocity + std::max(biasTerm, 0.0f);
        };

        const float rhs0 = computeRhs(c0, vn0);
        const float rhs1 = computeRhs(c1, vn1);

        const int slot0 = FindBlockSlot(manifold, c0.key);
        const int slot1 = FindBlockSlot(manifold, c1.key);
        if (slot0 < 0 || slot1 < 0 || slot0 == slot1) {
            return false;
        }

        const float old0 = manifold.blockNormalImpulseSum[slot0];
        const float old1 = manifold.blockNormalImpulseSum[slot1];
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
        if (both0 >= -lcpEpsilon && both1 >= -lcpEpsilon) {
            const auto w = residualW(both0, both1);
            if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
                new0 = std::max(0.0f, both0);
                new1 = std::max(0.0f, both1);
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

        if (!solved || !std::isfinite(new0) || !std::isfinite(new1)) {
            return false;
        }

        manifold.blockNormalImpulseSum[slot0] = new0;
        manifold.blockNormalImpulseSum[slot1] = new1;
        c0.normalImpulseSum = new0;
        c1.normalImpulseSum = new1;
        const float delta0 = new0 - old0;
        const float delta1 = new1 - old1;
        ApplyImpulse(a, b, invIA, invIB, ra0, rb0, delta0 * manifoldNormal);
        ApplyImpulse(a, b, invIA, invIB, ra1, rb1, delta1 * manifoldNormal);
        return true;
    }

    void SolveContactsInManifold(Manifold& manifold) {
        if (manifold.contacts.size() == 2) {
            if (!SolveNormalBlock2(manifold)) {
                SolveNormalScalar(manifold.contacts[0]);
                SolveNormalScalar(manifold.contacts[1]);
            }
        } else {
            for (Contact& c : manifold.contacts) {
                SolveNormalScalar(c);
            }
        }

        if (manifold.contacts.size() == 2) {
            for (Contact& c : manifold.contacts) {
                const int slot = FindBlockSlot(manifold, c.key);
                if (slot >= 0) {
                    manifold.blockNormalImpulseSum[slot] = c.normalImpulseSum;
                }
            }
        }

        for (Contact& c : manifold.contacts) {
            Body& a = bodies_[c.a];
            Body& b = bodies_[c.b];

            const Mat3 invIA = a.InvInertiaWorld();
            const Mat3 invIB = b.InvInertiaWorld();
            const Vec3 ra = c.point - a.position;
            const Vec3 rb = c.point - b.position;

            const Vec3 va2 = a.velocity + Cross(a.angularVelocity, ra);
            const Vec3 vb2 = b.velocity + Cross(b.angularVelocity, rb);
            const Vec3 rv2 = vb2 - va2;
            Vec3 tangent = rv2 - Dot(rv2, c.normal) * c.normal;
            const float tangentLenSq = LengthSquared(tangent);
            if (tangentLenSq <= kEpsilon) {
                continue;
            }
            tangent = tangent / std::sqrt(tangentLenSq);

            const Vec3 raCrossT = Cross(ra, tangent);
            const Vec3 rbCrossT = Cross(rb, tangent);
            const float tangentMass = a.invMass + b.invMass
                + Dot(raCrossT, invIA * raCrossT)
                + Dot(rbCrossT, invIB * rbCrossT);
            if (tangentMass <= kEpsilon) {
                continue;
            }

            float lambdaT = -Dot(rv2, tangent) / tangentMass;
            const float muS = 0.5f * (a.staticFriction + b.staticFriction);
            const float muD = 0.5f * (a.dynamicFriction + b.dynamicFriction);
            float mu = muS;
            if (contactSolverConfig_.staticToDynamicTransitionSpeed > kEpsilon) {
                const float slipSpeed = std::sqrt(tangentLenSq);
                const float transitionT = SmoothStep01(slipSpeed / contactSolverConfig_.staticToDynamicTransitionSpeed);
                mu = muS + (muD - muS) * transitionT;
            }
            const float maxFriction = std::max(mu, 0.0f) * c.normalImpulseSum;
            const float oldTangentImpulse = c.tangentImpulseSum;
            c.tangentImpulseSum = std::clamp(c.tangentImpulseSum + lambdaT, -maxFriction, maxFriction);
            lambdaT = c.tangentImpulseSum - oldTangentImpulse;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambdaT * tangent);
        }
    }

    void SolveDistanceJoint(DistanceJoint& j) {
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

    void SolveHingeJoint(HingeJoint& j) {
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

    void SolveIslands() {
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

    void SolveJointPositions() {
        for (DistanceJoint& j : joints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (a.invMass + b.invMass <= kEpsilon) {
                continue;
            }

            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 pa = a.position + ra;
            const Vec3 pb = b.position + rb;
            const Vec3 delta = pb - pa;
            const float len = Length(delta);
            if (len <= kEpsilon) {
                continue;
            }

            const Vec3 n = delta / len;
            const float error = len - j.restLength;
            const float invMassSum = a.invMass + b.invMass;
            if (invMassSum <= kEpsilon) {
                continue;
            }
            const Vec3 correction = (0.15f * error / invMassSum) * n;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }

        for (HingeJoint& j : hingeJoints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (a.invMass + b.invMass <= kEpsilon) {
                continue;
            }

            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 pa = a.position + ra;
            const Vec3 pb = b.position + rb;
            const Vec3 error = pb - pa;
            const float invMassSum = a.invMass + b.invMass;
            const Vec3 correction = (0.2f / std::max(invMassSum, kEpsilon)) * error;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }
    }

    void ApplyImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& ra,
        const Vec3& rb,
        const Vec3& impulse) {
        if (!a.isSleeping) {
            a.velocity -= impulse * a.invMass;
            a.angularVelocity -= invIA * Cross(ra, impulse);
        }
        if (!b.isSleeping) {
            b.velocity += impulse * b.invMass;
            b.angularVelocity += invIB * Cross(rb, impulse);
        }
    }

    void ApplyAngularImpulse(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& angularImpulse) {
        if (!a.isSleeping) {
            a.angularVelocity -= invIA * angularImpulse;
        }
        if (!b.isSleeping) {
            b.angularVelocity += invIB * angularImpulse;
        }
    }

    void PositionalCorrection() {
        if (contactSolverConfig_.useSplitImpulse) {
            return;
        }

        for (const Manifold& manifold : manifolds_) {
            for (const Contact& c : manifold.contacts) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];

                const float invMassSum = a.invMass + b.invMass;
                if (invMassSum <= kEpsilon) {
                    continue;
                }

                const float correctionMagnitude = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f)
                    * contactSolverConfig_.positionalCorrectionPercent / invMassSum;
                const Vec3 correction = correctionMagnitude * c.normal;

                if (!a.isSleeping) {
                    a.position -= correction * a.invMass;
                }
                if (!b.isSleeping) {
                    b.position += correction * b.invMass;
                }
            }
        }
    }

    void UpdateSleeping() {
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

    void ClearAccumulators() {
        for (Body& body : bodies_) {
            body.force = {0.0f, 0.0f, 0.0f};
            body.torque = {0.0f, 0.0f, 0.0f};
        }
    }

private:
    Vec3 gravity_{};
    ContactSolverConfig contactSolverConfig_{};
    float currentSubstepDt_ = 1.0f / 60.0f;
    std::vector<Body> bodies_;
    std::vector<BroadphaseProxy> proxies_;
    std::vector<TreeNode> treeNodes_;
    std::int32_t rootNode_ = -1;
    std::int32_t freeNode_ = -1;
    std::size_t lastBroadphaseMovedProxyCount_ = 0;
    std::vector<Contact> contacts_;
    std::vector<Contact> previousContacts_;
    std::vector<Manifold> manifolds_;
    std::vector<Manifold> previousManifolds_;
    std::vector<Island> islands_;
    std::vector<DistanceJoint> joints_;
    std::vector<HingeJoint> hingeJoints_;
};


} // namespace minphys3d
