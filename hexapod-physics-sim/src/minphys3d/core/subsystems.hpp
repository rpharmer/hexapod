#pragma once

#include <algorithm>
#include <array>
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
    std::function<ConvexDispatchRoute(ShapeType, ShapeType)> selectConvexDispatchRoute;
    std::function<bool(std::uint32_t, std::uint32_t)> genericConvexOverlap;
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
    static bool IsConvexShape(const ShapeType shape) {
        return shape == ShapeType::Sphere || shape == ShapeType::Box || shape == ShapeType::Capsule;
    }
    static void DispatchSpecialized(const NarrowphaseContext& context, const Pair& pair, const Body& a, const Body& b) {
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

    void GenerateContacts(const NarrowphaseContext& context) const {
        for (const Pair& pair : context.pairs) {
            const Body& a = context.bodies[pair.a];
            const Body& b = context.bodies[pair.b];
            const bool eligibleConvex = IsConvexShape(a.shape) && IsConvexShape(b.shape);
            const ConvexDispatchRoute route = (eligibleConvex && context.selectConvexDispatchRoute)
                ? context.selectConvexDispatchRoute(a.shape, b.shape)
                : ConvexDispatchRoute::SpecializedOnly;

            const bool genericOverlap = (!eligibleConvex || !context.genericConvexOverlap)
                ? true
                : context.genericConvexOverlap(pair.a, pair.b);

            if (route == ConvexDispatchRoute::GenericOnly) {
                if (!genericOverlap) {
                    continue;
                }
                DispatchSpecialized(context, pair, a, b);
                continue;
            }

            if (route == ConvexDispatchRoute::GenericGateThenSpecialized && !genericOverlap) {
                continue;
            }
            DispatchSpecialized(context, pair, a, b);
        }
    }
};

struct ContactSolverContext {
    std::vector<Body>& bodies;
    std::vector<Contact>& contacts;
    std::vector<Manifold>& manifolds;
    const std::vector<Manifold>& previousManifolds;
    std::function<bool(const ManifoldKey&, const Contact&, float&, std::array<float, 2>&, std::uint16_t&)> tryGetPersistentImpulseState;
    std::function<void(Manifold&, const Manifold*)> manageManifoldContacts;
    std::function<void(Manifold&)> refreshManifoldBlockCache;
    std::function<void(Manifold&)> selectBlockSolvePair;
    std::function<void(const Manifold&)> recordSelectedPairHistory;
    std::function<void(Manifold&)> solveManifoldNormalImpulses;
    std::function<int(const Manifold&, std::uint64_t)> findBlockSlot;
    std::function<void(Body&, Body&, const Mat3&, const Mat3&, const Vec3&, const Vec3&, const Vec3&)> applyImpulse;
    std::function<void(bool)> recordTangentBasisState;
#ifndef NDEBUG
    std::function<void(FrictionBudgetNormalSupportSource)> recordFrictionBudgetSaturation;
    std::function<void(bool)> recordManifoldTangentReprojection;
#endif
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
                m.manifoldType = c.manifoldType;
                m.contacts.push_back(c);
                context.manifolds.push_back(m);
            }
        }

        if (context.config.enableDeterministicOrdering) {
            for (Manifold& m : context.manifolds) {
                std::stable_sort(m.contacts.begin(), m.contacts.end(), [](const Contact& lhs, const Contact& rhs) {
                    if (lhs.key != rhs.key) {
                        return lhs.key < rhs.key;
                    }
                    if (lhs.featureKey != rhs.featureKey) {
                        return lhs.featureKey < rhs.featureKey;
                    }
                    if (lhs.penetration != rhs.penetration) {
                        return lhs.penetration > rhs.penetration;
                    }
                    if (lhs.point.y != rhs.point.y) {
                        return lhs.point.y < rhs.point.y;
                    }
                    if (lhs.point.x != rhs.point.x) {
                        return lhs.point.x < rhs.point.x;
                    }
                    return lhs.point.z < rhs.point.z;
                });
            }
            std::stable_sort(context.manifolds.begin(), context.manifolds.end(), [](const Manifold& lhs, const Manifold& rhs) {
                if (lhs.pairKey() != rhs.pairKey()) {
                    return lhs.pairKey() < rhs.pairKey();
                }
                if (lhs.manifoldType != rhs.manifoldType) {
                    return lhs.manifoldType < rhs.manifoldType;
                }
                return lhs.contacts.size() < rhs.contacts.size();
            });
        }

        for (Manifold& m : context.manifolds) {
            if (!m.contacts.empty()) {
                m.manifoldType = m.contacts.front().manifoldType;
            }
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
            m.lowQuality = false;
            m.blockSolveEligible = false;
            m.usedBlockSolve = false;
            context.manageManifoldContacts(m, previous);

            if (previous == nullptr) {
                m.blockNormalImpulseSum = {0.0f, 0.0f};
                m.blockContactKeys = {0u, 0u};
                m.blockSlotValid = {false, false};
                m.selectedBlockContactKeys = {0u, 0u};
                m.t0 = {};
                m.t1 = {};
                m.tangentBasisValid = false;
                m.manifoldTangentImpulseSum = {0.0f, 0.0f};
                m.manifoldTangentImpulseValid = false;
                m.stickConstraintActive = false;
                m.stickConstraintAge = 0;
            } else {
                m.t0 = previous->t0;
                m.t1 = previous->t1;
                m.tangentBasisValid = previous->tangentBasisValid;
                m.manifoldTangentImpulseSum = previous->manifoldTangentImpulseSum;
                m.manifoldTangentImpulseValid = previous->manifoldTangentImpulseValid;
                m.stickConstraintActive = previous->stickConstraintActive;
                m.stickConstraintAge = previous->stickConstraintAge;
                m.cachedImpulseByContactKey = previous->cachedImpulseByContactKey;
            }

            const ManifoldKey manifoldId{std::min(m.a, m.b), std::max(m.a, m.b), m.manifoldType};
            for (Contact& c : m.contacts) {
                float normalImpulse = 0.0f;
                std::array<float, 2> tangentImpulse{0.0f, 0.0f};
                std::uint16_t persistenceAge = 0;
                if (context.tryGetPersistentImpulseState(manifoldId, c, normalImpulse, tangentImpulse, persistenceAge)) {
                    c.normalImpulseSum = normalImpulse;
                    c.tangentImpulseSum0 = tangentImpulse[0];
                    c.tangentImpulseSum1 = tangentImpulse[1];
                    c.tangentImpulseSum = c.tangentImpulseSum0;
                    c.persistenceAge = persistenceAge;
                } else {
                    c.normalImpulseSum = 0.0f;
                    c.tangentImpulseSum0 = 0.0f;
                    c.tangentImpulseSum1 = 0.0f;
                    c.tangentImpulseSum = 0.0f;
                    c.persistenceAge = 0;
                }
            }

            if (previous != nullptr && previous->manifoldTangentImpulseValid && previous->tangentBasisValid) {
                const float old0 = previous->manifoldTangentImpulseSum[0];
                const float old1 = previous->manifoldTangentImpulseSum[1];
                m.manifoldTangentImpulseSum = {old0, old1};
                m.manifoldTangentImpulseValid = true;
#ifndef NDEBUG
                if (context.recordManifoldTangentReprojection) {
                    context.recordManifoldTangentReprojection(true);
                }
#endif
            } else {
                m.manifoldTangentImpulseSum = {0.0f, 0.0f};
                m.manifoldTangentImpulseValid = false;
#ifndef NDEBUG
                if (context.recordManifoldTangentReprojection) {
                    context.recordManifoldTangentReprojection(false);
                }
#endif
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

        Vec3 manifoldRelativeVelocity = context.bodies[manifold.b].velocity - context.bodies[manifold.a].velocity;
        if (!manifold.contacts.empty()) {
            manifoldRelativeVelocity = {};
            for (const Contact& c : manifold.contacts) {
                const Body& a = context.bodies[c.a];
                const Body& b = context.bodies[c.b];
                const Vec3 ra = c.point - a.position;
                const Vec3 rb = c.point - b.position;
                const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
                const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
                manifoldRelativeVelocity += (vb - va);
            }
            manifoldRelativeVelocity = manifoldRelativeVelocity / static_cast<float>(manifold.contacts.size());
        }

        Vec3 manifoldT0{};
        Vec3 manifoldT1{};
        const bool reuseBasisHint = manifold.tangentBasisValid;
        const Vec3 previousT0 = manifold.t0;
        const Vec3 previousT1 = manifold.t1;
        const std::array<float, 2> previousManifoldImpulse = manifold.manifoldTangentImpulseSum;
        const bool previousImpulseValid = manifold.manifoldTangentImpulseValid && manifold.tangentBasisValid;
        const Vec3* preferredTangent = manifold.tangentBasisValid ? &manifold.t0 : nullptr;
        const bool basisValid = World::ComputeStableTangentFrame(
            manifold.normal, manifoldRelativeVelocity, manifoldT0, manifoldT1, preferredTangent);
        manifold.t0 = manifoldT0;
        manifold.t1 = manifoldT1;
        manifold.tangentBasisValid = basisValid;
        if (context.recordTangentBasisState) {
            context.recordTangentBasisState(basisValid && reuseBasisHint);
        }
        if (basisValid && previousImpulseValid) {
            const Vec3 worldImpulse = previousManifoldImpulse[0] * previousT0 + previousManifoldImpulse[1] * previousT1;
            manifold.manifoldTangentImpulseSum = {Dot(worldImpulse, manifold.t0), Dot(worldImpulse, manifold.t1)};
            manifold.manifoldTangentImpulseValid = true;
        }
        if (!manifold.tangentBasisValid) {
            manifold.manifoldTangentImpulseSum = {0.0f, 0.0f};
            manifold.manifoldTangentImpulseValid = false;
            for (Contact& c : manifold.contacts) {
                c.tangentImpulseSum0 = 0.0f;
                c.tangentImpulseSum1 = 0.0f;
                c.tangentImpulseSum = 0.0f;
            }
            return;
        }

        const auto sumAllContactSupport = [&]() {
            float total = 0.0f;
            for (const Contact& c : manifold.contacts) {
                total += std::max(c.normalImpulseSum, 0.0f);
            }
            return total;
        };
        const auto sumSelectedPairSupport = [&]() {
            float total = 0.0f;
            for (int idx : manifold.selectedBlockContactIndices) {
                if (idx >= 0 && static_cast<std::size_t>(idx) < manifold.contacts.size()) {
                    total += std::max(manifold.contacts[static_cast<std::size_t>(idx)].normalImpulseSum, 0.0f);
                }
            }
            return total;
        };

        const float allContactSupport = sumAllContactSupport();
        const float selectedPairSupport = sumSelectedPairSupport();
        float totalNormalSupport = allContactSupport;
        switch (context.config.frictionBudgetNormalSupportSource) {
            case FrictionBudgetNormalSupportSource::SelectedBlockPairOnly:
                totalNormalSupport = selectedPairSupport > kEpsilon ? selectedPairSupport : allContactSupport;
                break;
            case FrictionBudgetNormalSupportSource::AllManifoldContacts:
                totalNormalSupport = allContactSupport;
                break;
            case FrictionBudgetNormalSupportSource::BlendedSelectedPairAndManifold: {
                const float selectedWeight = std::max(context.config.frictionBudgetSelectedPairBlendWeight, 0.0f);
                const float manifoldWeight = 1.0f;
                const float weightedSum = selectedWeight * selectedPairSupport + manifoldWeight * allContactSupport;
                const float denom = selectedWeight + manifoldWeight;
                totalNormalSupport = denom > kEpsilon ? (weightedSum / denom) : allContactSupport;
                break;
            }
        }
        const Body& firstA = context.bodies[manifold.contacts.front().a];
        const Body& firstB = context.bodies[manifold.contacts.front().b];
        const float muS = 0.5f * (firstA.staticFriction + firstB.staticFriction);
        const float muD = 0.5f * (firstA.dynamicFriction + firstB.dynamicFriction);
        const float mu = std::max(std::max(muS, muD), 0.0f);
        const float manifoldBudget = std::max(0.0f, context.config.manifoldFrictionBudgetScale) * mu * totalNormalSupport;
        const bool useManifoldBudget = context.config.enableManifoldFrictionBudget && manifoldBudget > 0.0f;
        const bool canUseStick = context.config.enablePersistentStickConstraints
            && manifold.manifoldTangentImpulseValid
            && manifold.contacts.size() >= 2;

        std::array<float, 2> accumulatedTangent{0.0f, 0.0f};
        if (manifold.manifoldTangentImpulseValid) {
            accumulatedTangent = manifold.manifoldTangentImpulseSum;
        }
        float meanSlipSpeed = 0.0f;
        std::uint32_t slipSamples = 0;

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
            const float slipSpeed = std::sqrt(
                Dot(rv2, manifold.t0) * Dot(rv2, manifold.t0)
                + Dot(rv2, manifold.t1) * Dot(rv2, manifold.t1));
            meanSlipSpeed += slipSpeed;
            ++slipSamples;
            const Vec3 raCrossT0 = Cross(ra, manifold.t0);
            const Vec3 rbCrossT0 = Cross(rb, manifold.t0);
            const Vec3 raCrossT1 = Cross(ra, manifold.t1);
            const Vec3 rbCrossT1 = Cross(rb, manifold.t1);
            const float tangentMass0 = a.invMass + b.invMass + Dot(raCrossT0, invIA * raCrossT0) + Dot(rbCrossT0, invIB * rbCrossT0);
            const float tangentMass1 = context.config.enableTwoAxisFrictionSolve
                ? (a.invMass + b.invMass + Dot(raCrossT1, invIA * raCrossT1) + Dot(rbCrossT1, invIB * rbCrossT1))
                : std::numeric_limits<float>::infinity();
            if (tangentMass0 <= kEpsilon || tangentMass1 <= kEpsilon) {
                continue;
            }

            const std::array<float, 2> oldT{c.tangentImpulseSum0, c.tangentImpulseSum1};
            std::array<float, 2> newT{
                c.tangentImpulseSum0 - Dot(rv2, manifold.t0) / tangentMass0,
                c.tangentImpulseSum1 - Dot(rv2, manifold.t1) / tangentMass1};
            if (!context.config.enableTwoAxisFrictionSolve) {
                newT[1] = 0.0f;
            }
            if (canUseStick && manifold.stickConstraintActive) {
                const float retention = std::clamp(context.config.stickImpulseRetention, 0.0f, 1.0f);
                newT[0] = retention * oldT[0] + (1.0f - retention) * newT[0];
                newT[1] = retention * oldT[1] + (1.0f - retention) * newT[1];
            }

            const float perContactLimit = mu * std::max(c.normalImpulseSum, 0.0f);
            const float contactLenSq = newT[0] * newT[0] + newT[1] * newT[1];
            if (contactLenSq > perContactLimit * perContactLimit && perContactLimit > kEpsilon) {
                const float scale = perContactLimit / std::sqrt(contactLenSq);
                newT[0] *= scale;
                newT[1] *= scale;
            } else if (perContactLimit <= kEpsilon) {
                newT = {0.0f, 0.0f};
            }

            std::array<float, 2> candidateAccumulated{
                accumulatedTangent[0] - oldT[0] + newT[0],
                accumulatedTangent[1] - oldT[1] + newT[1]};
            if (useManifoldBudget) {
                if (context.config.frictionBudgetUseRadialClamp) {
                    const float lenSq = candidateAccumulated[0] * candidateAccumulated[0]
                        + candidateAccumulated[1] * candidateAccumulated[1];
                    const float maxLenSq = manifoldBudget * manifoldBudget;
                    if (lenSq > maxLenSq && manifoldBudget > kEpsilon) {
                        const float scale = manifoldBudget / std::sqrt(lenSq);
                        newT[0] = oldT[0] + (newT[0] - oldT[0]) * scale;
                        newT[1] = oldT[1] + (newT[1] - oldT[1]) * scale;
#ifndef NDEBUG
                        if (context.recordFrictionBudgetSaturation) {
                            context.recordFrictionBudgetSaturation(context.config.frictionBudgetNormalSupportSource);
                        }
#endif
                    }
                } else {
                    newT[0] = std::clamp(newT[0], -manifoldBudget, manifoldBudget);
                    newT[1] = std::clamp(newT[1], -manifoldBudget, manifoldBudget);
                }
            }

            c.tangentImpulseSum0 = newT[0];
            c.tangentImpulseSum1 = newT[1];
            c.tangentImpulseSum = c.tangentImpulseSum0;
            const float lambdaT0 = c.tangentImpulseSum0 - oldT[0];
            const float lambdaT1 = c.tangentImpulseSum1 - oldT[1];
            const Vec3 tangentImpulse = lambdaT0 * manifold.t0 + lambdaT1 * manifold.t1;
            context.applyImpulse(a, b, invIA, invIB, ra, rb, tangentImpulse);
            accumulatedTangent[0] += lambdaT0;
            accumulatedTangent[1] += lambdaT1;
        }
        manifold.manifoldTangentImpulseSum = accumulatedTangent;
        manifold.manifoldTangentImpulseValid = true;
        const float averageSlip = (slipSamples > 0) ? (meanSlipSpeed / static_cast<float>(slipSamples)) : std::numeric_limits<float>::infinity();
        const bool stableAgedContacts = std::all_of(
            manifold.contacts.begin(),
            manifold.contacts.end(),
            [&](const Contact& c) { return c.persistenceAge >= context.config.stickMinPersistenceAge; });
        const bool shouldStick = canUseStick
            && stableAgedContacts
            && averageSlip <= context.config.stickVelocityThreshold;
        if (shouldStick) {
            manifold.stickConstraintActive = true;
            manifold.stickConstraintAge = static_cast<std::uint16_t>(std::min<std::uint32_t>(
                static_cast<std::uint32_t>(manifold.stickConstraintAge) + 1u,
                std::numeric_limits<std::uint16_t>::max()));
        } else {
            manifold.stickConstraintActive = false;
            manifold.stickConstraintAge = 0;
        }
    }
};

struct JointSolverContext {
    std::vector<Body>& bodies;
    std::vector<DistanceJoint>& joints;
    std::vector<HingeJoint>& hingeJoints;
    std::vector<BallSocketJoint>& ballSocketJoints;
    std::vector<FixedJoint>& fixedJoints;
    std::vector<PrismaticJoint>& prismaticJoints;
    std::vector<ServoJoint>& servoJoints;
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

        for (BallSocketJoint& j : context.ballSocketJoints) {
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

        for (FixedJoint& j : context.fixedJoints) {
            Body& a = context.bodies[j.a];
            Body& b = context.bodies[j.b];
            if (a.invMass + b.invMass <= kEpsilon) continue;
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 anchorError = (b.position + rb) - (a.position + ra);
            const Vec3 correction = (0.2f / std::max(a.invMass + b.invMass, kEpsilon)) * anchorError;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;

            const Quat target = Normalize(Normalize(a.orientation) * j.referenceRotation);
            Quat dq = Normalize(Conjugate(Normalize(b.orientation)) * target);
            if (dq.w < 0.0f) {
                dq = {-dq.w, -dq.x, -dq.y, -dq.z};
            }
            Vec3 angularError{dq.x, dq.y, dq.z};
            if (LengthSquared(angularError) > kEpsilon * kEpsilon) {
                angularError = 0.3f * angularError;
                if (!a.isSleeping) {
                    a.orientation = Normalize(a.orientation * Quat{1.0f, -angularError.x * a.invMass, -angularError.y * a.invMass, -angularError.z * a.invMass});
                }
                if (!b.isSleeping) {
                    b.orientation = Normalize(b.orientation * Quat{1.0f, angularError.x * b.invMass, angularError.y * b.invMass, angularError.z * b.invMass});
                }
            }
        }

        for (PrismaticJoint& j : context.prismaticJoints) {
            Body& a = context.bodies[j.a];
            Body& b = context.bodies[j.b];
            if (a.invMass + b.invMass <= kEpsilon) continue;
            const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
            const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 axis = Normalize(Rotate(a.orientation, j.localAxisA));
            Vec3 error = (b.position + rb) - (a.position + ra);
            const float along = Dot(error, axis);
            error -= along * axis;
            const Vec3 correction = (0.2f / std::max(a.invMass + b.invMass, kEpsilon)) * error;
            if (!a.isSleeping) a.position += correction * a.invMass;
            if (!b.isSleeping) b.position -= correction * b.invMass;

            if (j.limitsEnabled) {
                float limitError = 0.0f;
                if (along < j.lowerTranslation) {
                    limitError = along - j.lowerTranslation;
                } else if (along > j.upperTranslation) {
                    limitError = along - j.upperTranslation;
                }
                if (std::abs(limitError) > kEpsilon) {
                    const Vec3 limitCorrection = (0.15f * limitError / std::max(a.invMass + b.invMass, kEpsilon)) * axis;
                    if (!a.isSleeping) a.position += limitCorrection * a.invMass;
                    if (!b.isSleeping) b.position -= limitCorrection * b.invMass;
                }
            }
        }

        for (ServoJoint& j : context.servoJoints) {
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
    const std::vector<BallSocketJoint>& ballSocketJoints;
    const std::vector<FixedJoint>& fixedJoints;
    const std::vector<PrismaticJoint>& prismaticJoints;
    const std::vector<ServoJoint>& servoJoints;
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
                for (const BallSocketJoint& j : context.ballSocketJoints) {
                    if (j.a == id) enqueueConnectedBodies(j.b);
                    else if (j.b == id) enqueueConnectedBodies(j.a);
                }
                for (const FixedJoint& j : context.fixedJoints) {
                    if (j.a == id) enqueueConnectedBodies(j.b);
                    else if (j.b == id) enqueueConnectedBodies(j.a);
                }
                for (const PrismaticJoint& j : context.prismaticJoints) {
                    if (j.a == id) enqueueConnectedBodies(j.b);
                    else if (j.b == id) enqueueConnectedBodies(j.a);
                }
                for (const ServoJoint& j : context.servoJoints) {
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
