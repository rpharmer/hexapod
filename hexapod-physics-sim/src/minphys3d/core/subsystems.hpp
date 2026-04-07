#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace minphys3d::core_internal {

struct BroadphaseContext {
    std::function<std::vector<Pair>()> computePotentialPairs;
};

class BroadphaseSystem {
public:
    std::vector<Pair> ComputePotentialPairs(const BroadphaseContext& context) const {
        return context.computePotentialPairs ? context.computePotentialPairs() : std::vector<Pair>{};
    }
};

struct NarrowphaseContext {
    const std::vector<Body>& bodies;
    const std::vector<Pair>& pairs;
    std::function<void(std::uint32_t, std::uint32_t)> sphereSphere;
    std::function<void(std::uint32_t, std::uint32_t)> sphereCapsule;
    std::function<void(std::uint32_t, std::uint32_t)> capsuleCapsule;
    std::function<void(std::uint32_t, std::uint32_t)> capsulePlane;
    std::function<void(std::uint32_t, std::uint32_t)> capsuleBox;
    std::function<void(std::uint32_t, std::uint32_t)> spherePlane;
    std::function<void(std::uint32_t, std::uint32_t)> boxPlane;
    std::function<void(std::uint32_t, std::uint32_t)> sphereBox;
    std::function<void(std::uint32_t, std::uint32_t)> boxBox;
};

class NarrowphaseSystem {
public:
    void GenerateContacts(const NarrowphaseContext& context) const {
        for (const Pair& pair : context.pairs) {
            const Body& a = context.bodies[pair.a];
            const Body& b = context.bodies[pair.b];

            if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
                context.sphereSphere(pair.a, pair.b);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Capsule) {
                context.sphereCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Sphere) {
                context.sphereCapsule(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Capsule) {
                context.capsuleCapsule(pair.a, pair.b);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Plane) {
                context.capsulePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Capsule) {
                context.capsulePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Box) {
                context.capsuleBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Capsule) {
                context.capsuleBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
                context.spherePlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
                context.spherePlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Plane) {
                context.boxPlane(pair.a, pair.b);
            } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Box) {
                context.boxPlane(pair.b, pair.a);
            } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Box) {
                context.sphereBox(pair.a, pair.b);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Sphere) {
                context.sphereBox(pair.b, pair.a);
            } else if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) {
                context.boxBox(pair.a, pair.b);
            }
        }
    }
};

struct ContactSolverContext {
    std::vector<Body>& bodies;
    std::vector<Contact>& contacts;
    std::vector<Manifold>& manifolds;
    const std::vector<Manifold>& previousManifolds;
    std::function<bool(const PersistentPointKey&, float&, float&, std::uint16_t&)> tryGetPersistentImpulseState;
    std::function<void(std::vector<Contact>&)> sortManifoldContacts;
    std::function<void(Manifold&)> refreshManifoldBlockCache;
    std::function<void(Manifold&)> selectBlockSolvePair;
    std::function<void(const Manifold&)> recordSelectedPairHistory;
    std::function<void(Manifold&)> solveManifoldNormalImpulses;
    std::function<int(const Manifold&, std::uint64_t)> findBlockSlot;
    std::function<void(Body&, Body&, const Mat3&, const Mat3&, const Vec3&, const Vec3&, const Vec3&)> applyImpulse;
    const ContactSolverConfig& config;
};

class ContactSolver {
public:
    void BuildManifolds(const ContactSolverContext& context) const {
        context.manifolds.clear();
        for (const Contact& c : context.contacts) {
            bool found = false;
            for (Manifold& m : context.manifolds) {
                if ((m.a == c.a && m.b == c.b) || (m.a == c.b && m.b == c.a)) {
                    m.contacts.push_back(c);
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
                m.manifoldType = static_cast<std::uint8_t>(c.featureKey >> 56u);
                m.contacts.push_back(c);
                context.manifolds.push_back(m);
            }
        }

        for (Manifold& m : context.manifolds) {
            if (!m.contacts.empty()) {
                m.manifoldType = static_cast<std::uint8_t>(m.contacts.front().featureKey >> 56u);
            }
            m.lowQuality = false;
            m.blockSolveEligible = false;
            m.usedBlockSolve = false;
            context.sortManifoldContacts(m.contacts);

            const Manifold* previous = nullptr;
            for (const Manifold& old : context.previousManifolds) {
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

            const ManifoldKey manifoldId{std::min(m.a, m.b), std::max(m.a, m.b), m.manifoldType};
            std::unordered_map<std::uint64_t, std::uint8_t> currentFeatureOrdinal;
            currentFeatureOrdinal.reserve(m.contacts.size());
            for (Contact& c : m.contacts) {
                const std::uint8_t ordinal = currentFeatureOrdinal[c.featureKey]++;
                const PersistentPointKey pointKey{manifoldId, c.featureKey, ordinal};
                float normalImpulse = 0.0f;
                float tangentImpulse = 0.0f;
                std::uint16_t persistenceAge = 0;
                if (context.tryGetPersistentImpulseState(pointKey, normalImpulse, tangentImpulse, persistenceAge)) {
                    c.normalImpulseSum = normalImpulse;
                    c.tangentImpulseSum = tangentImpulse;
                    c.persistenceAge = persistenceAge;
                } else {
                    c.normalImpulseSum = 0.0f;
                    c.tangentImpulseSum = 0.0f;
                    c.persistenceAge = 0;
                }
            }

            context.refreshManifoldBlockCache(m);
            context.selectBlockSolvePair(m);
            context.recordSelectedPairHistory(m);
        }
    }

    void SolveContactsInManifold(const ContactSolverContext& context, Manifold& manifold) const {
        if (manifold.contacts.size() >= 2) {
            const int selected0 = manifold.selectedBlockContactIndices[0];
            const int selected1 = manifold.selectedBlockContactIndices[1];
            const bool selectedIndicesValid = selected0 >= 0 && selected1 >= 0 && selected0 != selected1
                && static_cast<std::size_t>(std::max(selected0, selected1)) < manifold.contacts.size();
            const bool selectedKeysPopulated = manifold.selectedBlockContactKeys[0] != 0u && manifold.selectedBlockContactKeys[1] != 0u;
            const bool blockSlotsPopulated = manifold.blockSlotValid[0] && manifold.blockSlotValid[1];
            const bool blockKeysPopulated = manifold.blockContactKeys[0] != 0u && manifold.blockContactKeys[1] != 0u;
            (void)selectedIndicesValid;
            (void)selectedKeysPopulated;
            (void)blockSlotsPopulated;
            (void)blockKeysPopulated;
            assert(selectedIndicesValid);
            assert(selectedKeysPopulated);
            assert(blockSlotsPopulated);
            assert(blockKeysPopulated);
        }

        context.solveManifoldNormalImpulses(manifold);
        if (manifold.contacts.size() >= 2) {
            for (Contact& c : manifold.contacts) {
                const int slot = context.findBlockSlot(manifold, c.key);
                if (slot >= 0) {
                    manifold.blockNormalImpulseSum[slot] = c.normalImpulseSum;
                }
            }
        }

        for (Contact& c : manifold.contacts) {
            Body& a = context.bodies[c.a];
            Body& b = context.bodies[c.b];

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
            const float tangentMass = a.invMass + b.invMass + Dot(raCrossT, invIA * raCrossT) + Dot(rbCrossT, invIB * rbCrossT);
            if (tangentMass <= kEpsilon) {
                continue;
            }

            float lambdaT = -Dot(rv2, tangent) / tangentMass;
            const float muS = 0.5f * (a.staticFriction + b.staticFriction);
            const float muD = 0.5f * (a.dynamicFriction + b.dynamicFriction);
            const float oldTangentImpulse = c.tangentImpulseSum;
            const float slipSpeed = std::sqrt(tangentLenSq);
            const float normalImpulseMagnitude = std::max(c.normalImpulseSum, 0.0f);
            const float staticLimit = std::max(muS, 0.0f) * normalImpulseMagnitude;
            const float candidateImpulse = c.tangentImpulseSum + lambdaT;
            const bool stickPhase = slipSpeed <= std::max(context.config.staticFrictionSpeedThreshold, 0.0f);
            if (stickPhase && std::abs(candidateImpulse) <= staticLimit) {
                c.tangentImpulseSum = candidateImpulse;
            } else {
                float mu = muS;
                if (context.config.staticToDynamicTransitionSpeed > kEpsilon) {
                    const float t = std::clamp(slipSpeed / context.config.staticToDynamicTransitionSpeed, 0.0f, 1.0f);
                    const float smooth = t * t * (3.0f - 2.0f * t);
                    mu = muS + (muD - muS) * smooth;
                }
                const float maxFriction = std::max(mu, 0.0f) * normalImpulseMagnitude;
                c.tangentImpulseSum = std::clamp(candidateImpulse, -maxFriction, maxFriction);
            }
            lambdaT = c.tangentImpulseSum - oldTangentImpulse;
            context.applyImpulse(a, b, invIA, invIB, ra, rb, lambdaT * tangent);
        }
    }
};

struct JointSolverContext {
    std::vector<Body>& bodies;
    std::vector<DistanceJoint>& joints;
    std::vector<HingeJoint>& hingeJoints;
};

class JointSolver {
public:
    void SolveJointPositions(JointSolverContext& context) const {
        for (DistanceJoint& j : context.joints) {
            Body& a = context.bodies[j.a];
            Body& b = context.bodies[j.b];
            if (a.invMass + b.invMass <= kEpsilon) continue;
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 delta = (b.position + rb) - (a.position + ra);
            const float len = Length(delta);
            if (len <= kEpsilon) continue;
            const Vec3 correction = (0.15f * (len - j.restLength) / (a.invMass + b.invMass)) * (delta / len);
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }

        for (HingeJoint& j : context.hingeJoints) {
            Body& a = context.bodies[j.a];
            Body& b = context.bodies[j.b];
            if (a.invMass + b.invMass <= kEpsilon) continue;
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 error = (b.position + rb) - (a.position + ra);
            const Vec3 correction = (0.2f / std::max(a.invMass + b.invMass, kEpsilon)) * error;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;
        }
    }
};

struct SleepSystemContext {
    std::vector<Body>& bodies;
    const std::vector<Manifold>& manifolds;
    const std::vector<DistanceJoint>& joints;
    const std::vector<HingeJoint>& hingeJoints;
    float linearThreshold;
    float angularThreshold;
    std::uint32_t framesThreshold;
};

class SleepSystem {
public:
    void UpdateSleeping(const SleepSystemContext& context) const {
        std::vector<bool> visited(context.bodies.size(), false);
        for (std::uint32_t start = 0; start < context.bodies.size(); ++start) {
            if (visited[start] || context.bodies[start].invMass == 0.0f) continue;

            std::vector<std::uint32_t> islandBodies;
            std::vector<std::uint32_t> stack{start};
            visited[start] = true;
            bool islandNearlyStill = true;

            while (!stack.empty()) {
                const std::uint32_t id = stack.back();
                stack.pop_back();
                Body& body = context.bodies[id];
                islandBodies.push_back(id);

                const bool nearlyStill = LengthSquared(body.velocity) < (context.linearThreshold * context.linearThreshold)
                    && LengthSquared(body.angularVelocity) < (context.angularThreshold * context.angularThreshold);
                islandNearlyStill = islandNearlyStill && nearlyStill;

                auto enqueueConnectedBodies = [&](std::uint32_t other) {
                    if (other < context.bodies.size() && context.bodies[other].invMass != 0.0f && !visited[other]) {
                        visited[other] = true;
                        stack.push_back(other);
                    }
                };

                for (const Manifold& m : context.manifolds) {
                    if (m.a == id) enqueueConnectedBodies(m.b);
                    else if (m.b == id) enqueueConnectedBodies(m.a);
                }
                for (const DistanceJoint& j : context.joints) {
                    if (j.a == id) enqueueConnectedBodies(j.b);
                    else if (j.b == id) enqueueConnectedBodies(j.a);
                }
                for (const HingeJoint& j : context.hingeJoints) {
                    if (j.a == id) enqueueConnectedBodies(j.b);
                    else if (j.b == id) enqueueConnectedBodies(j.a);
                }
            }

            for (std::uint32_t id : islandBodies) {
                Body& body = context.bodies[id];
                if (islandNearlyStill) {
                    ++body.sleepCounter;
                    if (body.sleepCounter >= static_cast<int>(context.framesThreshold)) {
                        body.isSleeping = true;
                        body.velocity = {0.0f, 0.0f, 0.0f};
                        body.angularVelocity = {0.0f, 0.0f, 0.0f};
                    }
                } else {
                    body.isSleeping = false;
                    body.sleepCounter = 0;
                }
            }
        }
    }
};

} // namespace minphys3d::core_internal
