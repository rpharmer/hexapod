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

#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/core/world.hpp"

namespace minphys3d::core_internal {

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
    std::function<void(std::uint32_t, std::uint32_t)> convexPlane;
    std::function<void(std::uint32_t, std::uint32_t)> convexConvexEpa;
    std::function<void(std::uint32_t, std::uint32_t)> sphereCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> cylinderBox;
    std::function<void(std::uint32_t, std::uint32_t)> cylinderCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> capsuleCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> sphereHalfCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> halfCylinderBox;
    std::function<void(std::uint32_t, std::uint32_t)> halfCylinderCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> halfCylinderHalfCylinder;
    std::function<void(std::uint32_t, std::uint32_t)> capsuleHalfCylinder;
};

class NarrowphaseSystem {
public:
    static bool IsConvexShape(const ShapeType shape) {
        return IsConvexPrimitiveShape(shape);
    }

    static bool IsBoxLikeShape(const ShapeType shape) {
        return shape == ShapeType::Box;
    }

    static bool IsCylinderFamily(const ShapeType shape) {
        return shape == ShapeType::Cylinder || shape == ShapeType::HalfCylinder;
    }

    static bool IsFullCylinderShape(const ShapeType shape) {
        return shape == ShapeType::Cylinder;
    }

    static bool IsHalfCylinderShape(const ShapeType shape) {
        return shape == ShapeType::HalfCylinder;
    }

    static void DispatchSpecialized(const NarrowphaseContext& context, const Pair& pair, const Body& a, const Body& b) {
        const bool aBoxLike = IsBoxLikeShape(a.shape);
        const bool bBoxLike = IsBoxLikeShape(b.shape);

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
        } else if (a.shape == ShapeType::Capsule && bBoxLike) {
            context.capsuleBox(pair.a, pair.b);
        } else if (aBoxLike && b.shape == ShapeType::Capsule) {
            context.capsuleBox(pair.b, pair.a);
        } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
            context.spherePlane(pair.a, pair.b);
        } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
            context.spherePlane(pair.b, pair.a);
        } else if (aBoxLike && b.shape == ShapeType::Plane) {
            context.boxPlane(pair.a, pair.b);
        } else if (a.shape == ShapeType::Plane && bBoxLike) {
            context.boxPlane(pair.b, pair.a);
        } else if (a.shape == ShapeType::Sphere && bBoxLike) {
            context.sphereBox(pair.a, pair.b);
        } else if (aBoxLike && b.shape == ShapeType::Sphere) {
            context.sphereBox(pair.b, pair.a);
        } else if (IsCylinderFamily(a.shape) && b.shape == ShapeType::Plane) {
            context.convexPlane(pair.a, pair.b);
        } else if (a.shape == ShapeType::Plane && IsCylinderFamily(b.shape)) {
            context.convexPlane(pair.b, pair.a);
        } else if (aBoxLike && bBoxLike) {
            context.boxBox(pair.a, pair.b);
        } else if (a.shape == ShapeType::Sphere && IsFullCylinderShape(b.shape)) {
            context.sphereCylinder(pair.a, pair.b);
        } else if (IsFullCylinderShape(a.shape) && b.shape == ShapeType::Sphere) {
            context.sphereCylinder(pair.b, pair.a);
        } else if (IsFullCylinderShape(a.shape) && bBoxLike) {
            context.cylinderBox(pair.a, pair.b);
        } else if (aBoxLike && IsFullCylinderShape(b.shape)) {
            context.cylinderBox(pair.b, pair.a);
        } else if (IsFullCylinderShape(a.shape) && IsFullCylinderShape(b.shape)) {
            context.cylinderCylinder(pair.a, pair.b);
        } else if (a.shape == ShapeType::Capsule && IsFullCylinderShape(b.shape)) {
            context.capsuleCylinder(pair.a, pair.b);
        } else if (IsFullCylinderShape(a.shape) && b.shape == ShapeType::Capsule) {
            context.capsuleCylinder(pair.b, pair.a);
        } else if (a.shape == ShapeType::Sphere && IsHalfCylinderShape(b.shape)) {
            context.sphereHalfCylinder(pair.a, pair.b);
        } else if (IsHalfCylinderShape(a.shape) && b.shape == ShapeType::Sphere) {
            context.sphereHalfCylinder(pair.b, pair.a);
        } else if (IsHalfCylinderShape(a.shape) && bBoxLike) {
            context.halfCylinderBox(pair.a, pair.b);
        } else if (aBoxLike && IsHalfCylinderShape(b.shape)) {
            context.halfCylinderBox(pair.b, pair.a);
        } else if (IsHalfCylinderShape(a.shape) && IsFullCylinderShape(b.shape)) {
            context.halfCylinderCylinder(pair.a, pair.b);
        } else if (IsFullCylinderShape(a.shape) && IsHalfCylinderShape(b.shape)) {
            context.halfCylinderCylinder(pair.b, pair.a);
        } else if (IsHalfCylinderShape(a.shape) && IsHalfCylinderShape(b.shape)) {
            context.halfCylinderHalfCylinder(pair.a, pair.b);
        } else if (a.shape == ShapeType::Capsule && IsHalfCylinderShape(b.shape)) {
            context.capsuleHalfCylinder(pair.a, pair.b);
        } else if (IsHalfCylinderShape(a.shape) && b.shape == ShapeType::Capsule) {
            context.capsuleHalfCylinder(pair.b, pair.a);
        } else if (
            (IsCylinderFamily(a.shape) && IsConvexPrimitiveShape(b.shape) && b.shape != ShapeType::Plane)
            || (IsCylinderFamily(b.shape) && IsConvexPrimitiveShape(a.shape) && a.shape != ShapeType::Plane)) {
            context.convexConvexEpa(pair.a, pair.b);
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
    /// Mirrors `JointSolverConfig::servoPositionPasses`.
    std::uint8_t servoPositionPasses = 4;
    /// World gravity; used to strip vertical anchor snap for static-backed servos (see servo position pass).
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    /// Substep duration — used to convert angular position corrections into velocity biases so
    /// they interact correctly with servo PD damping instead of injecting phantom energy.
    float substepDt = 1.0f / 360.0f;
};

inline float WrapJointAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline Vec3 JointReferenceFallback(const Vec3& axis) {
    const Vec3 fallback = (std::abs(axis.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
    Vec3 reference = fallback - Dot(fallback, axis) * axis;
    if (!TryNormalize(reference, reference)) {
        reference = {1.0f, 0.0f, 0.0f};
    }
    return reference;
}

inline Vec3 ResolveJointReference(const Quat& orientation, const Vec3& localReference, const Vec3& axis) {
    Vec3 reference = Rotate(orientation, localReference);
    reference = reference - Dot(reference, axis) * axis;
    if (!TryNormalize(reference, reference)) {
        return JointReferenceFallback(axis);
    }
    return reference;
}

inline float SignedAngleAroundAxis(const Vec3& from, const Vec3& to, const Vec3& axis) {
    Vec3 fromN = from;
    Vec3 toN = to;
    if (!TryNormalize(fromN, fromN) || !TryNormalize(toN, toN)) {
        return 0.0f;
    }
    return std::atan2(Dot(Cross(fromN, toN), axis), Dot(fromN, toN));
}

/// Strongly biased axis-alignment snap for servo joints.
///
/// This keeps the correction directly focused on the axis mismatch instead of averaging it down
/// through the high-fanout hub path.
inline Vec3 ComputeServoAxisAlignmentCorrection(const ServoJoint& j, const Vec3& axisA, const Vec3& axisB) {
    const float stab = std::clamp(j.angleStabilizationScale, 0.0f, 1.0f);
    if (stab <= 1e-7f) {
        return {};
    }

    Vec3 axisError = Cross(axisA, axisB);
    const float axisErrorMagnitude = Length(axisError);
    if (axisErrorMagnitude <= 1e-6f) {
        return {};
    }

    constexpr float kAxisAlignmentGain = 0.45f;
    constexpr float kAxisAlignmentMaxCorrection = 0.30f;
    const float correctionMagnitude = std::min(kAxisAlignmentMaxCorrection * stab,
                                               kAxisAlignmentGain * stab * axisErrorMagnitude);
    return axisError * (correctionMagnitude / axisErrorMagnitude);
}

/// Small Baumgarte-style orientation correction for joint position passes.
/// Uses the same **scalar** effective rotational compliance as axis torques in `SolveServoJoint`
/// (`wa = u·invIA·u`, `wb = u·invIB·u`) to split the quaternion step between bodies, instead of
/// `invMass` weights (which over-rotate light links on a heavy chassis and destabilize multi-pass
/// servo chains). Keeps the legacy opposite-sign convention on body B.
inline void ApplyAngularPositionCorrection(Body& a, Body& b, const Vec3& correction) {
    if (LengthSquared(correction) <= 1e-18f) {
        return;
    }
    const Vec3 u = correction * (1.0f / Length(correction));
    const Mat3 invIA = a.InvInertiaWorld();
    const Mat3 invIB = b.InvInertiaWorld();
    const float wa = Dot(u, invIA * u);
    const float wb = Dot(u, invIB * u);
    const float denom = wa + wb;
    float weightA;
    float weightB;
    if (denom > kEpsilon) {
        weightA = wa / denom;
        weightB = wb / denom;
    } else {
        const float invMassSum = std::max(a.invMass + b.invMass, kEpsilon);
        weightA = a.invMass / invMassSum;
        weightB = b.invMass / invMassSum;
    }
    if (!a.isSleeping && a.invMass > 0.0f) {
        a.orientation =
            Normalize(a.orientation * Quat{1.0f, correction.x * weightA, correction.y * weightA, correction.z * weightA});
    }
    if (!b.isSleeping && b.invMass > 0.0f) {
        b.orientation = Normalize(
            b.orientation * Quat{1.0f, -correction.x * weightB, -correction.y * weightB, -correction.z * weightB});
    }
}

/// Compute the per-body angular correction weight for a correction vector, split by inverse inertia.
/// Returns {weightA, weightB}. Does not modify the bodies.
inline std::pair<float, float> ComputeAngularCorrectionWeights(const Body& a, const Body& b, const Vec3& correction) {
    if (LengthSquared(correction) <= 1e-18f) {
        return {0.0f, 0.0f};
    }
    const Vec3 u = correction * (1.0f / Length(correction));
    const Mat3 invIA = a.InvInertiaWorld();
    const Mat3 invIB = b.InvInertiaWorld();
    const float wa = Dot(u, invIA * u);
    const float wb = Dot(u, invIB * u);
    const float denom = wa + wb;
    if (denom > kEpsilon) {
        return {wa / denom, wb / denom};
    }
    const float invMassSum = std::max(a.invMass + b.invMass, kEpsilon);
    return {a.invMass / invMassSum, b.invMass / invMassSum};
}

/// Apply a pre-computed angular correction directly to a body's orientation.
inline void ApplyAngularCorrectionToBody(Body& body, const Vec3& correction) {
    if (!body.isSleeping && body.invMass > 0.0f && LengthSquared(correction) > 1e-18f) {
        body.orientation = Normalize(
            body.orientation * Quat{1.0f, correction.x, correction.y, correction.z});
    }
}

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

        const int servoPositionPasses = std::max(0, static_cast<int>(context.servoPositionPasses));
        if (servoPositionPasses > 0 && !context.servoJoints.empty()) {
            // Hybrid angular correction: direct position snap for low-fanout bodies, Jacobi-
            // averaged velocity biases for high-fanout bodies.
            //
            // Direct position corrections rotate bodies without changing angular velocity. For
            // simple chains (1-2 joints per body) this gives stiff snap. But on star topologies
            // (hexapod chassis with 6 servos) the corrections accumulate on the hub, injecting
            // phantom energy that feeds back through contacts closing the kinematic loop,
            // producing exponential angular velocity divergence.
            //
            // Velocity biases avoid this: the PD damping (dampingGain × omega) provides negative
            // feedback. The Jacobi average ensures the hub receives one averaged correction
            // regardless of joint count. The bias is computed ONCE (not per-pass) to avoid
            // amplification by the pass count.
            constexpr std::uint16_t kFanoutThreshold = 2;
            const float invDt = (context.substepDt > 1e-8f) ? (1.0f / context.substepDt) : 0.0f;

            std::vector<std::uint16_t> servoFanout(context.bodies.size(), 0);
            std::uint16_t maxFanout = 0;
            for (const ServoJoint& sj : context.servoJoints) {
                ++servoFanout[sj.a];
                ++servoFanout[sj.b];
            }
            for (auto f : servoFanout) maxFanout = std::max(maxFanout, f);

            const bool useVelocityBiases = maxFanout > kFanoutThreshold;

            if (useVelocityBiases) {
                std::vector<Vec3> angularAccum(context.bodies.size(), Vec3{0.0f, 0.0f, 0.0f});
                std::vector<std::uint16_t> angularCount(context.bodies.size(), 0);

                for (const ServoJoint& j : context.servoJoints) {
                    Body& a = context.bodies[j.a];
                    Body& b = context.bodies[j.b];
                    if (a.invMass + b.invMass <= kEpsilon) continue;
                    const float stab = std::clamp(j.angleStabilizationScale, 0.0f, 1.0f);
                    if (stab <= 1e-7f) continue;

                    Vec3 axisA = Rotate(a.orientation, j.localAxisA);
                    Vec3 axisB = Rotate(b.orientation, j.localAxisB);
                    if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) continue;

                    const Vec3 axisCorrection = ComputeServoAxisAlignmentCorrection(j, axisA, axisB);
                    if (LengthSquared(axisCorrection) > 0.0f) {
                        ApplyAngularPositionCorrection(a, b, axisCorrection);
                        axisA = Rotate(a.orientation, j.localAxisA);
                        axisB = Rotate(b.orientation, j.localAxisB);
                        if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) continue;
                    }

                    const Vec3 refA = ResolveJointReference(a.orientation, j.localReferenceA, axisA);
                    const Vec3 refB = ResolveJointReference(b.orientation, j.localReferenceB, axisA);
                    const float hingeAngle = SignedAngleAroundAxis(refA, refB, axisA);
                    const float targetError = WrapJointAngle(hingeAngle - j.targetAngle);
                    const float clampedAngle = std::clamp(
                        0.15f * stab * targetError, -0.06f * stab, 0.06f * stab);
                    if (std::abs(clampedAngle) > 1e-5f) {
                        const Vec3 hc = clampedAngle * axisA;
                        const auto [wA, wB] = ComputeAngularCorrectionWeights(a, b, hc);
                        angularAccum[j.a] = angularAccum[j.a] + hc * wA;
                        angularAccum[j.b] = angularAccum[j.b] + hc * (-wB);
                    }
                    ++angularCount[j.a];
                    ++angularCount[j.b];
                }

                for (std::size_t i = 0; i < context.bodies.size(); ++i) {
                    if (angularCount[i] > 0) {
                        Body& body = context.bodies[i];
                        if (body.isSleeping || body.invMass <= 0.0f) continue;
                        const Vec3 avg = angularAccum[i] * (1.0f / static_cast<float>(angularCount[i]));
                        body.angularVelocity = body.angularVelocity + avg * invDt;
                    }
                }
            }

            for (int pass = 0; pass < servoPositionPasses; ++pass) {
                for (ServoJoint& j : context.servoJoints) {
                    Body& a = context.bodies[j.a];
                    Body& b = context.bodies[j.b];
                    if (a.invMass + b.invMass <= kEpsilon) continue;
                    const float stab = std::clamp(j.angleStabilizationScale, 0.0f, 1.0f);

                    const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
                    const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
                    const Vec3 error = (b.position + rb) - (a.position + ra);
                    Vec3 correction = (0.28f / std::max(a.invMass + b.invMass, kEpsilon)) * error;
                    const float gLenSq = LengthSquared(context.gravity);
                    const bool oneStatic = a.isStatic != b.isStatic;
                    if (oneStatic && gLenSq > 1e-12f) {
                        const Vec3 gHat = context.gravity * (1.0f / std::sqrt(gLenSq));
                        correction -= Dot(correction, gHat) * gHat;
                    }
                    if (!a.isSleeping) a.position += correction * a.invMass;
                    if (!b.isSleeping) b.position -= correction * b.invMass;

                    if (useVelocityBiases || stab <= 1e-7f) continue;

                    Vec3 axisA = Rotate(a.orientation, j.localAxisA);
                    Vec3 axisB = Rotate(b.orientation, j.localAxisB);
                    if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) continue;

                    const Vec3 axisCorrection = ComputeServoAxisAlignmentCorrection(j, axisA, axisB);
                    if (LengthSquared(axisCorrection) > 0.0f) {
                        ApplyAngularPositionCorrection(a, b, axisCorrection);
                        axisA = Rotate(a.orientation, j.localAxisA);
                        axisB = Rotate(b.orientation, j.localAxisB);
                        if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) continue;
                    }

                    const Vec3 refA = ResolveJointReference(a.orientation, j.localReferenceA, axisA);
                    const Vec3 refB = ResolveJointReference(b.orientation, j.localReferenceB, axisA);
                    const float hingeAngle = SignedAngleAroundAxis(refA, refB, axisA);
                    const float targetError = WrapJointAngle(hingeAngle - j.targetAngle);
                    const float clampedAngleCorrection = std::clamp(
                        0.15f * stab * targetError, -0.06f * stab, 0.06f * stab);
                    if (std::abs(clampedAngleCorrection) > 1e-5f) {
                        ApplyAngularPositionCorrection(a, b, clampedAngleCorrection * axisA);
                    }
                }
            }
        }
    }
};

} // namespace minphys3d::core_internal
