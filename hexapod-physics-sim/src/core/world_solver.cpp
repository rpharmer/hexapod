#include "minphys3d/core/constraint_solver.hpp"
#include "minphys3d/core/sleep_system.hpp"
#include "minphys3d/core/subsystems.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/solver/block2_solver.hpp"
#include "minphys3d/solver/block4_solver.hpp"
#include "minphys3d/solver/island_ordering.hpp"

#include <cmath>

namespace minphys3d {

namespace {

Vec3 JointReferenceFallback(const Vec3& axis) {
    const Vec3 fallback = (std::abs(axis.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
    Vec3 reference = fallback - Dot(fallback, axis) * axis;
    if (!TryNormalize(reference, reference)) {
        reference = {1.0f, 0.0f, 0.0f};
    }
    return reference;
}

Vec3 ResolveJointReference(const Quat& orientation, const Vec3& localReference, const Vec3& axis) {
    Vec3 reference = Rotate(orientation, localReference);
    reference = reference - Dot(reference, axis) * axis;
    if (!TryNormalize(reference, reference)) {
        return JointReferenceFallback(axis);
    }
    return reference;
}

float SignedAngleAroundAxis(const Vec3& from, const Vec3& to, const Vec3& axis) {
    Vec3 fromN = from;
    Vec3 toN = to;
    if (!TryNormalize(fromN, fromN) || !TryNormalize(toN, toN)) {
        return 0.0f;
    }
    return std::atan2(Dot(Cross(fromN, toN), axis), Dot(fromN, toN));
}

} // namespace

void World::ResolveTOIPipeline(float dt) {
        float remaining = dt;
        const int maxToiIterations = std::max(contactSolverConfig_.toi.max_iterations, 1);
        const float toiMinTimeStep = std::max(contactSolverConfig_.toi.min_time_step, 1e-9f);
        int iterations = 0;
        while (remaining > toiMinTimeStep && iterations < maxToiIterations) {
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
            if (hit.toi <= toiMinTimeStep) {
                remaining = std::max(0.0f, remaining - toiMinTimeStep);
            }
            ++iterations;
        }

        if (remaining > 0.0f) {
            AdvanceDynamicBodies(remaining);
        }
    }

void World::WarmStartContacts() {
        for (Manifold& m : manifolds_) {
            EnsureStableTwoPointOrder(m);
            std::vector<bool> skipPerContactNormal(m.contacts.size(), false);
            const int selected0 = m.selectedBlockContactIndices[0];
            const int selected1 = m.selectedBlockContactIndices[1];
            const bool selectedIndicesValid = selected0 >= 0 && selected1 >= 0 && selected0 != selected1
                && static_cast<std::size_t>(std::max(selected0, selected1)) < m.contacts.size();
            if (selectedIndicesValid && m.blockSlotValid[0] && m.blockSlotValid[1]) {
                const int slotForSelected0 = FindBlockSlot(m, m.contacts[static_cast<std::size_t>(selected0)].key);
                const int slotForSelected1 = FindBlockSlot(m, m.contacts[static_cast<std::size_t>(selected1)].key);
                const bool blockCacheMatchesSelectedPair = slotForSelected0 >= 0 && slotForSelected1 >= 0 && slotForSelected0 != slotForSelected1;
                if (blockCacheMatchesSelectedPair) {
                    const std::array<int, 2> selectedContactIndices{selected0, selected1};
                    for (int contactIndex : selectedContactIndices) {
                        Contact& c = m.contacts[static_cast<std::size_t>(contactIndex)];
                        const int slot = FindBlockSlot(m, c.key);
                        if (slot >= 0) {
                            const float cachedNormalImpulse = m.blockNormalImpulseSum[slot];
                            if (cachedNormalImpulse != 0.0f) {
                                Body& a = bodies_[c.a];
                                Body& b = bodies_[c.b];
                                const Mat3 invIA = a.InvInertiaWorld();
                                const Mat3 invIB = b.InvInertiaWorld();
                                const Vec3 ra = c.point - a.position;
                                const Vec3 rb = c.point - b.position;
                                ApplyImpulse(a, b, invIA, invIB, ra, rb, cachedNormalImpulse * m.normal);
                            }
                            c.normalImpulseSum = cachedNormalImpulse;
                        }
                        skipPerContactNormal[static_cast<std::size_t>(contactIndex)] = true;
                    }
                }
            }

            std::size_t contactIndex = 0;
            const bool useManifoldTangentWarmStart =
                m.tangentBasisValid && m.manifoldTangentImpulseValid && !m.contacts.empty();
            if (useManifoldTangentWarmStart) {
                float totalNormal = 0.0f;
                for (const Contact& c : m.contacts) {
                    totalNormal += std::max(c.normalImpulseSum, 0.0f);
                }
                const float equalWeight = 1.0f / static_cast<float>(m.contacts.size());
                for (Contact& c : m.contacts) {
                    const float w = totalNormal > kEpsilon
                        ? std::max(c.normalImpulseSum, 0.0f) / totalNormal
                        : equalWeight;
                    c.tangentImpulseSum0 = w * m.manifoldTangentImpulseSum[0];
                    c.tangentImpulseSum1 = w * m.manifoldTangentImpulseSum[1];
                    c.tangentImpulseSum = c.tangentImpulseSum0;
                }
            }
            for (Contact& c : m.contacts) {
                if (const std::array<float, 3>* cached = FindPerContactImpulseCache(m, c.key)) {
                    // Normal impulse warm-start source precedence is unchanged: per-contact cache always applies.
                    c.normalImpulseSum = std::max((*cached)[0], 0.0f);
                    // Tangent warm-start precedence:
                    // 1) Manifold tangent accumulator (when manifold basis + impulses are valid).
                    // 2) Per-contact cache fallback only when manifold tangent basis is unavailable/invalid.
                    if (!useManifoldTangentWarmStart) {
                        c.tangentImpulseSum0 = (*cached)[1];
                        c.tangentImpulseSum1 = (*cached)[2];
                        c.tangentImpulseSum = c.tangentImpulseSum0;
                    }
                }
                const bool skipNormal = skipPerContactNormal[contactIndex];
                if (!skipNormal && c.normalImpulseSum == 0.0f
                    && c.tangentImpulseSum0 == 0.0f && c.tangentImpulseSum1 == 0.0f) {
                    ++contactIndex;
                    continue;
                }
                if (skipNormal && c.tangentImpulseSum0 == 0.0f && c.tangentImpulseSum1 == 0.0f) {
                    ++contactIndex;
                    continue;
                }

                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                const Mat3 invIA = a.InvInertiaWorld();
                const Mat3 invIB = b.InvInertiaWorld();
                const Vec3 ra = c.point - a.position;
                const Vec3 rb = c.point - b.position;

                const Vec3 normalImpulse = skipNormal ? Vec3{0.0f, 0.0f, 0.0f} : c.normalImpulseSum * c.normal;
                Vec3 tangentImpulse{0.0f, 0.0f, 0.0f};
                if (m.tangentBasisValid) {
                    tangentImpulse = c.tangentImpulseSum0 * m.t0 + c.tangentImpulseSum1 * m.t1;
                } else {
                    tangentImpulse = c.tangentImpulseSum * m.t0;
                }
                const Vec3 impulse = normalImpulse + tangentImpulse;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
                std::array<float, 3>& cacheEntry = EnsurePerContactImpulseCache(m, c.key);
                cacheEntry[0] = std::max(c.normalImpulseSum, 0.0f);
                cacheEntry[1] = c.tangentImpulseSum0;
                cacheEntry[2] = c.tangentImpulseSum1;
                ++contactIndex;
            }
        }
    }

bool World::SolveNormalBlock2(Manifold& manifold, BlockSolveFallbackReason& fallbackReason, float& determinantOrConditionEstimate) {
        fallbackReason = BlockSolveFallbackReason::None;
        determinantOrConditionEstimate = std::numeric_limits<float>::quiet_NaN();
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
        Vec3 normal0 = Normalize(c0.normal);
        Vec3 normal1 = Normalize(c1.normal);
        if (Dot(c0.normal, manifoldNormal) < 0.95f || Dot(c1.normal, manifoldNormal) < 0.95f
            || LengthSquared(normal0) <= kEpsilon || LengthSquared(normal1) <= kEpsilon) {
            fallbackReason = BlockSolveFallbackReason::ContactNormalMismatch;
            return false;
        }

        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();
        const Vec3 ra0 = c0.point - a.position;
        const Vec3 rb0 = c0.point - b.position;
        const Vec3 ra1 = c1.point - a.position;
        const Vec3 rb1 = c1.point - b.position;

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

        const Vec3 ra0CrossN0 = Cross(ra0, normal0);
        const Vec3 rb0CrossN0 = Cross(rb0, normal0);
        const Vec3 ra1CrossN1 = Cross(ra1, normal1);
        const Vec3 rb1CrossN1 = Cross(rb1, normal1);
        const float k11 = a.invMass + b.invMass + Dot(ra0CrossN0, invIA * ra0CrossN0) + Dot(rb0CrossN0, invIB * rb0CrossN0);
        const float k22 = a.invMass + b.invMass + Dot(ra1CrossN1, invIA * ra1CrossN1) + Dot(rb1CrossN1, invIB * rb1CrossN1);

        const auto computeRhs = [&](Contact& c, const Vec3& contactNormal, float separatingVelocity, float contactNormalMass) {
            const float speedIntoContact = -separatingVelocity;
            const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);
            const float massRatioBoost = ComputeHighMassRatioBoost(a, b);

            float biasTerm = 0.0f;
            const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
            if (contactSolverConfig_.useSplitImpulse && !solverRelaxationPassActive_) {
                if (penetrationError > 0.0f && contactNormalMass > kEpsilon) {
                    const float boostedFactor = contactSolverConfig_.splitImpulseCorrectionFactor
                        * (1.0f + contactSolverConfig_.highMassRatioSplitImpulseBoost * (massRatioBoost - 1.0f));
                    const float correctionMagnitude = boostedFactor * penetrationError / contactNormalMass;
                    const Vec3 correction = correctionMagnitude * contactNormal;
                    AccumulateSplitImpulseCorrection(c.a, -correction * a.invMass, {0.0f, 0.0f, 0.0f});
                    AccumulateSplitImpulseCorrection(c.b, correction * b.invMass, {0.0f, 0.0f, 0.0f});
                }
            } else if (currentSubstepDt_ > kEpsilon && !solverRelaxationPassActive_) {
                const float maxSafeSeparatingSpeed = penetrationError / currentSubstepDt_;
                if (separatingVelocity <= maxSafeSeparatingSpeed) {
                    const float boostedBias = contactSolverConfig_.penetrationBiasFactor
                        * (1.0f + contactSolverConfig_.highMassRatioBiasBoost * (massRatioBoost - 1.0f));
                    biasTerm = (boostedBias * penetrationError) / currentSubstepDt_;
                    biasTerm = std::min(biasTerm, std::max(contactSolverConfig_.penetrationBiasMaxSpeed, 0.0f));
                }
            }
            return -(1.0f + restitution) * separatingVelocity + std::max(biasTerm, 0.0f);
        };

        solver_internal::Block2SolveInput solveInput{};
        solveInput.invIA = invIA;
        solveInput.invIB = invIB;
        solveInput.invMassSum = a.invMass + b.invMass;
        solveInput.blockDiagonalMinimum = contactSolverConfig_.blockDiagonalMinimum;
        solveInput.blockDeterminantEpsilon = contactSolverConfig_.blockDeterminantEpsilon;
        solveInput.blockConditionEstimateMax = contactSolverConfig_.blockConditionEstimateMax;
        solveInput.contacts[0] = {normal0, ra0, rb0, std::max(c0.normalImpulseSum, 0.0f), computeRhs(c0, normal0, vn0, k11)};
        solveInput.contacts[1] = {normal1, ra1, rb1, std::max(c1.normalImpulseSum, 0.0f), computeRhs(c1, normal1, vn1, k22)};

        const solver_internal::Block2SolveResult result = solver_internal::SolveBlock2NormalLcp(solveInput);
        determinantOrConditionEstimate = result.conditionEstimate;
        fallbackReason = static_cast<BlockSolveFallbackReason>(result.fallbackReason);
        if (!result.success) {
            return false;
        }

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        manifold.blockSolveDebug.selectedPreNormalImpulses = {solveInput.contacts[0].oldImpulse, solveInput.contacts[1].oldImpulse};
        manifold.blockSolveDebug.selectedPairPenetrationStep = c0.penetration + c1.penetration;
#endif

        manifold.blockNormalImpulseSum[slot0] = result.solvedImpulses[0];
        manifold.blockNormalImpulseSum[slot1] = result.solvedImpulses[1];
        c0.normalImpulseSum = result.solvedImpulses[0];
        c1.normalImpulseSum = result.solvedImpulses[1];
        EnsurePerContactImpulseCache(manifold, c0.key)[0] = c0.normalImpulseSum;
        EnsurePerContactImpulseCache(manifold, c1.key)[0] = c1.normalImpulseSum;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        manifold.blockSolveDebug.selectedPostNormalImpulses = {c0.normalImpulseSum, c1.normalImpulseSum};
#endif
        ApplyImpulse(a, b, invIA, invIB, ra0, rb0, result.impulseDeltas[0] * normal0);
        ApplyImpulse(a, b, invIA, invIB, ra1, rb1, result.impulseDeltas[1] * normal1);
        return true;
    }

bool World::SolveNormalProjected4(Manifold& manifold, BlockSolveFallbackReason& fallbackReason, float& conditionEstimate) {
        fallbackReason = BlockSolveFallbackReason::None;
        conditionEstimate = std::numeric_limits<float>::quiet_NaN();
        if (manifold.contacts.size() != 4 || manifold.manifoldType != 9) {
            fallbackReason = BlockSolveFallbackReason::TypePolicy;
            return false;
        }

        Body& a = bodies_[manifold.contacts[0].a];
        Body& b = bodies_[manifold.contacts[0].b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();
        const Vec3 manifoldNormal = Normalize(manifold.normal);
        if (LengthSquared(manifoldNormal) <= kEpsilon) {
            fallbackReason = BlockSolveFallbackReason::InvalidManifoldNormal;
            return false;
        }

        solver_internal::Block4SolveInput solveInput{};
        solveInput.invIA = invIA;
        solveInput.invIB = invIB;
        solveInput.invMassSum = a.invMass + b.invMass;
        solveInput.blockDiagonalMinimum = contactSolverConfig_.blockDiagonalMinimum;
        solveInput.blockConditionEstimateMax = contactSolverConfig_.blockConditionEstimateMax;
        solveInput.face4ConditionEstimateMax = contactSolverConfig_.face4ConditionEstimateMax;
        solveInput.face4Iterations = std::max<int>(contactSolverConfig_.face4Iterations, 1);
        solveInput.face4ProjectedGaussSeidelEpsilon = contactSolverConfig_.face4ProjectedGaussSeidelEpsilon;
        solveInput.face4MinSpreadSq = contactSolverConfig_.face4MinSpreadSq;
        solveInput.face4MinArea = contactSolverConfig_.face4MinArea;
        solveInput.symmetryTolerance = contactSolverConfig_.block4.symmetry_tolerance;

        for (int i = 0; i < 4; ++i) {
            Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            const Vec3 normal = Normalize(c.normal);
            if (Dot(normal, manifoldNormal) < 0.95f) {
                fallbackReason = BlockSolveFallbackReason::ContactNormalMismatch;
                return false;
            }
            const Vec3 ra = c.point - a.position;
            const Vec3 rb = c.point - b.position;
            const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
            const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
            const float vn = Dot(vb - va, normal);
            const float speedIntoContact = -vn;
            const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);
            float biasTerm = 0.0f;
            const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
            if (currentSubstepDt_ > kEpsilon && !contactSolverConfig_.useSplitImpulse && !solverRelaxationPassActive_) {
                const float maxSafeSeparatingSpeed = penetrationError / currentSubstepDt_;
                if (vn <= maxSafeSeparatingSpeed) {
                    const float massRatioBoost = ComputeHighMassRatioBoost(a, b);
                    const float boostedBias = contactSolverConfig_.penetrationBiasFactor
                        * (1.0f + contactSolverConfig_.highMassRatioBiasBoost * (massRatioBoost - 1.0f));
                    biasTerm = (boostedBias * penetrationError) / currentSubstepDt_;
                    biasTerm = std::min(biasTerm, std::max(contactSolverConfig_.penetrationBiasMaxSpeed, 0.0f));
                }
            }
            solveInput.contacts[static_cast<std::size_t>(i)] = {
                c.point,
                normal,
                ra,
                rb,
                std::max(c.normalImpulseSum, 0.0f),
                -(1.0f + restitution) * vn + std::max(biasTerm, 0.0f),
            };
        }

        const solver_internal::Block4SolveResult result = solver_internal::SolveBlock4ProjectedGaussSeidel(solveInput);
        conditionEstimate = result.conditionEstimate;
        fallbackReason = static_cast<BlockSolveFallbackReason>(result.fallbackReason);
        if (!result.success) {
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            c.normalImpulseSum = result.solvedImpulses[static_cast<std::size_t>(i)];
            EnsurePerContactImpulseCache(manifold, c.key)[0] = c.normalImpulseSum;
            ApplyImpulse(a,
                         b,
                         invIA,
                         invIB,
                         solveInput.contacts[static_cast<std::size_t>(i)].ra,
                         solveInput.contacts[static_cast<std::size_t>(i)].rb,
                         result.impulseDeltas[static_cast<std::size_t>(i)] * solveInput.contacts[static_cast<std::size_t>(i)].normal);
            const int slot = FindBlockSlot(manifold, c.key);
            if (slot >= 0) {
                manifold.blockNormalImpulseSum[slot] = c.normalImpulseSum;
            }
        }
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

bool World::SolveHingeAnchorBlock3x3(
        Body& a,
        Body& b,
        const Mat3& invIA,
        const Mat3& invIB,
        const Vec3& ra,
        const Vec3& rb,
        const Vec3& error,
        const Vec3& relVel,
        Vec3& outLambda,
        JointBlockFallbackReason& outReason) const {
        outReason = JointBlockFallbackReason::None;
        const Vec3 basis[3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };
        float K[3][3]{};
        const float invMassSum = a.invMass + b.invMass;
        for (int i = 0; i < 3; ++i) {
            const Vec3 raCrossNi = Cross(ra, basis[i]);
            const Vec3 rbCrossNi = Cross(rb, basis[i]);
            for (int j = 0; j < 3; ++j) {
                const Vec3 raCrossNj = Cross(ra, basis[j]);
                const Vec3 rbCrossNj = Cross(rb, basis[j]);
                K[i][j] = invMassSum * Dot(basis[i], basis[j])
                    + Dot(raCrossNi, invIA * raCrossNj)
                    + Dot(rbCrossNi, invIB * rbCrossNj);
            }
        }

        for (int i = 0; i < 3; ++i) {
            if (K[i][i] <= jointSolverConfig_.blockDiagonalMinimum) {
                outReason = JointBlockFallbackReason::DegenerateMassMatrix;
                return false;
            }
        }

        const float det =
            K[0][0] * (K[1][1] * K[2][2] - K[1][2] * K[2][1])
            - K[0][1] * (K[1][0] * K[2][2] - K[1][2] * K[2][0])
            + K[0][2] * (K[1][0] * K[2][1] - K[1][1] * K[2][0]);
        if (std::abs(det) <= jointSolverConfig_.blockDeterminantEpsilon) {
            outReason = JointBlockFallbackReason::DegenerateMassMatrix;
            return false;
        }

        if (jointSolverConfig_.blockConditionEstimateMax > 0.0f) {
            const float rowNorm = std::max({
                std::abs(K[0][0]) + std::abs(K[0][1]) + std::abs(K[0][2]),
                std::abs(K[1][0]) + std::abs(K[1][1]) + std::abs(K[1][2]),
                std::abs(K[2][0]) + std::abs(K[2][1]) + std::abs(K[2][2]),
            });
            float adj[3][3]{};
            adj[0][0] = K[1][1] * K[2][2] - K[1][2] * K[2][1];
            adj[0][1] = -(K[1][0] * K[2][2] - K[1][2] * K[2][0]);
            adj[0][2] = K[1][0] * K[2][1] - K[1][1] * K[2][0];
            adj[1][0] = -(K[0][1] * K[2][2] - K[0][2] * K[2][1]);
            adj[1][1] = K[0][0] * K[2][2] - K[0][2] * K[2][0];
            adj[1][2] = -(K[0][0] * K[2][1] - K[0][1] * K[2][0]);
            adj[2][0] = K[0][1] * K[1][2] - K[0][2] * K[1][1];
            adj[2][1] = -(K[0][0] * K[1][2] - K[0][2] * K[1][0]);
            adj[2][2] = K[0][0] * K[1][1] - K[0][1] * K[1][0];
            const float invAbsDet = 1.0f / std::abs(det);
            const float invNorm = std::max({
                (std::abs(adj[0][0]) + std::abs(adj[0][1]) + std::abs(adj[0][2])) * invAbsDet,
                (std::abs(adj[1][0]) + std::abs(adj[1][1]) + std::abs(adj[1][2])) * invAbsDet,
                (std::abs(adj[2][0]) + std::abs(adj[2][1]) + std::abs(adj[2][2])) * invAbsDet,
            });
            const float conditionEstimate = rowNorm * invNorm;
            if (!std::isfinite(conditionEstimate)
                || conditionEstimate > jointSolverConfig_.blockConditionEstimateMax) {
                outReason = JointBlockFallbackReason::ConditionEstimateExceeded;
                return false;
            }
        }

        const Vec3 rhs{
            jointSolverConfig_.hingeAnchorBiasFactor * error.x + jointSolverConfig_.hingeAnchorDampingFactor * relVel.x,
            jointSolverConfig_.hingeAnchorBiasFactor * error.y + jointSolverConfig_.hingeAnchorDampingFactor * relVel.y,
            jointSolverConfig_.hingeAnchorBiasFactor * error.z + jointSolverConfig_.hingeAnchorDampingFactor * relVel.z,
        };
        const Vec3 negRhs = -1.0f * rhs;

        float adj[3][3]{};
        adj[0][0] = K[1][1] * K[2][2] - K[1][2] * K[2][1];
        adj[0][1] = -(K[1][0] * K[2][2] - K[1][2] * K[2][0]);
        adj[0][2] = K[1][0] * K[2][1] - K[1][1] * K[2][0];
        adj[1][0] = -(K[0][1] * K[2][2] - K[0][2] * K[2][1]);
        adj[1][1] = K[0][0] * K[2][2] - K[0][2] * K[2][0];
        adj[1][2] = -(K[0][0] * K[2][1] - K[0][1] * K[2][0]);
        adj[2][0] = K[0][1] * K[1][2] - K[0][2] * K[1][1];
        adj[2][1] = -(K[0][0] * K[1][2] - K[0][2] * K[1][0]);
        adj[2][2] = K[0][0] * K[1][1] - K[0][1] * K[1][0];
        const float invDet = 1.0f / det;
        const Mat3 invK{
            {
                {adj[0][0] * invDet, adj[1][0] * invDet, adj[2][0] * invDet},
                {adj[0][1] * invDet, adj[1][1] * invDet, adj[2][1] * invDet},
                {adj[0][2] * invDet, adj[1][2] * invDet, adj[2][2] * invDet},
            }
        };
        outLambda = invK * negRhs;
        if (!std::isfinite(outLambda.x) || !std::isfinite(outLambda.y) || !std::isfinite(outLambda.z)) {
            outReason = JointBlockFallbackReason::NonFiniteResult;
            return false;
        }
        return true;
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

        bool usedBlockSolve = false;
        if (jointSolverConfig_.useBlockSolver) {
            Vec3 lambda{};
            JointBlockFallbackReason fallbackReason = JointBlockFallbackReason::None;
            if (SolveHingeAnchorBlock3x3(a, b, invIA, invIB, ra, rb, error, relVel, lambda, fallbackReason)) {
                j.impulseX += lambda.x;
                j.impulseY += lambda.y;
                j.impulseZ += lambda.z;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda);
                usedBlockSolve = true;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                ++solverTelemetry_.jointBlockSolveUsed;
#endif
            } else {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                switch (fallbackReason) {
                    case JointBlockFallbackReason::DegenerateMassMatrix:
                        ++solverTelemetry_.jointBlockFallbackDegenerate;
                        break;
                    case JointBlockFallbackReason::ConditionEstimateExceeded:
                        ++solverTelemetry_.jointBlockFallbackConditionEstimate;
                        break;
                    case JointBlockFallbackReason::NonFiniteResult:
                        ++solverTelemetry_.jointBlockFallbackNonFinite;
                        break;
                    case JointBlockFallbackReason::None:
                        break;
                }
#endif
            }
        }

        if (!usedBlockSolve) {
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

                const float bias = jointSolverConfig_.hingeAnchorBiasFactor * Dot(error, n)
                    + jointSolverConfig_.hingeAnchorDampingFactor * Dot(relVel, n);
                const float lambda = -bias / effMass;
                *impulseSums[i] += lambda;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
            }
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
            float lambda = -bias / effMass;
            if (j.maxMotorTorque > 0.0f) {
                const float oldAxisImpulse = *angImpulseSums[i];
                *angImpulseSums[i] = std::clamp(oldAxisImpulse + lambda, -j.maxMotorTorque, j.maxMotorTorque);
                lambda = *angImpulseSums[i] - oldAxisImpulse;
            } else {
                *angImpulseSums[i] += lambda;
            }
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * n);
        }

        const Vec3 refA = ResolveJointReference(a.orientation, j.localReferenceA, axisA);
        const Vec3 refB = ResolveJointReference(b.orientation, j.localReferenceB, axisA);
        float hingeAngle = SignedAngleAroundAxis(refA, refB, axisA);
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

void World::SolveBallSocketJoint(BallSocketJoint& j) {
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
        if (Length(error) > kWakeContactPenetrationThreshold || Length(relVel) > kWakeJointRelativeSpeedThreshold) {
            WakeConnectedBodies(j.a);
            WakeConnectedBodies(j.b);
        }

        Vec3 lambda{};
        JointBlockFallbackReason fallbackReason = JointBlockFallbackReason::None;
        if (!SolveHingeAnchorBlock3x3(a, b, invIA, invIB, ra, rb, error, relVel, lambda, fallbackReason)) {
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
                const float bias = jointSolverConfig_.hingeAnchorBiasFactor * Dot(error, n)
                    + jointSolverConfig_.hingeAnchorDampingFactor * Dot(relVel, n);
                const float dl = -bias / effMass;
                *impulseSums[i] += dl;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, dl * n);
            }
            return;
        }

        j.impulseX += lambda.x;
        j.impulseY += lambda.y;
        j.impulseZ += lambda.z;
        ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda);
    }

void World::SolveFixedJoint(FixedJoint& j) {
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

        Vec3 linearLambda{};
        JointBlockFallbackReason fallbackReason = JointBlockFallbackReason::None;
        if (SolveHingeAnchorBlock3x3(a, b, invIA, invIB, ra, rb, error, relVel, linearLambda, fallbackReason)) {
            j.impulseX += linearLambda.x;
            j.impulseY += linearLambda.y;
            j.impulseZ += linearLambda.z;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, linearLambda);
        }

        const Quat target = Normalize(Normalize(a.orientation) * j.referenceRotation);
        Quat dq = Normalize(Conjugate(Normalize(b.orientation)) * target);
        if (dq.w < 0.0f) {
            dq = {-dq.w, -dq.x, -dq.y, -dq.z};
        }
        const Vec3 angularError{dq.x, dq.y, dq.z};
        const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
        const Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        float* angularSums[3] = {&j.angularImpulseX, &j.angularImpulseY, &j.angularImpulseZ};
        for (int i = 0; i < 3; ++i) {
            const Vec3 n = axes[i];
            const float effMass = Dot(n, invIA * n) + Dot(n, invIB * n);
            if (effMass <= kEpsilon) {
                continue;
            }
            const float bias = 0.3f * Dot(angularError, n) + 0.08f * Dot(relAngVel, n);
            const float lambda = -bias / effMass;
            *angularSums[i] += lambda;
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * n);
        }
    }

void World::SolvePrismaticJoint(PrismaticJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3 invIA = a.InvInertiaWorld();
        const Mat3 invIB = b.InvInertiaWorld();

        const Vec3 ra = Rotate(a.orientation, j.localAnchorA);
        const Vec3 rb = Rotate(b.orientation, j.localAnchorB);
        const Vec3 pa = a.position + ra;
        const Vec3 pb = b.position + rb;
        const Vec3 rawError = pb - pa;
        const Vec3 va = a.velocity + Cross(a.angularVelocity, ra);
        const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb);
        const Vec3 relVel = vb - va;

        Vec3 axis = Normalize(Rotate(a.orientation, j.localAxisA));
        Vec3 t1 = Cross(axis, {0.0f, 1.0f, 0.0f});
        if (LengthSquared(t1) <= 1e-5f) t1 = Cross(axis, {0.0f, 0.0f, 1.0f});
        t1 = Normalize(t1);
        const Vec3 t2 = Normalize(Cross(axis, t1));

        const Vec3 errorPerp = rawError - Dot(rawError, axis) * axis;
        const Vec3 relVelPerp = relVel - Dot(relVel, axis) * axis;
        if (Length(errorPerp) > kWakeContactPenetrationThreshold || Length(relVel) > kWakeJointRelativeSpeedThreshold) {
            WakeConnectedBodies(j.a);
            WakeConnectedBodies(j.b);
        }

        const Vec3 perpAxes[2] = {t1, t2};
        float* perpSums[2] = {&j.impulseT1, &j.impulseT2};
        for (int i = 0; i < 2; ++i) {
            const Vec3 n = perpAxes[i];
            const Vec3 raCrossN = Cross(ra, n);
            const Vec3 rbCrossN = Cross(rb, n);
            const float effMass = a.invMass + b.invMass
                + Dot(raCrossN, invIA * raCrossN)
                + Dot(rbCrossN, invIB * rbCrossN);
            if (effMass <= kEpsilon) {
                continue;
            }
            const float bias = jointSolverConfig_.hingeAnchorBiasFactor * Dot(errorPerp, n)
                + jointSolverConfig_.hingeAnchorDampingFactor * Dot(relVelPerp, n);
            const float lambda = -bias / effMass;
            *perpSums[i] += lambda;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * n);
        }

        const float translation = Dot(rawError, axis);
        float axisBias = 0.0f;
        if (j.limitsEnabled) {
            if (translation < j.lowerTranslation) {
                axisBias = jointSolverConfig_.hingeAnchorBiasFactor * (translation - j.lowerTranslation);
            } else if (translation > j.upperTranslation) {
                axisBias = jointSolverConfig_.hingeAnchorBiasFactor * (translation - j.upperTranslation);
            }
        }
        if (axisBias != 0.0f) {
            const Vec3 raCrossAxis = Cross(ra, axis);
            const Vec3 rbCrossAxis = Cross(rb, axis);
            const float effMass = a.invMass + b.invMass
                + Dot(raCrossAxis, invIA * raCrossAxis)
                + Dot(rbCrossAxis, invIB * rbCrossAxis);
            if (effMass > kEpsilon) {
                const float lambda = -(axisBias + jointSolverConfig_.hingeAnchorDampingFactor * Dot(relVel, axis)) / effMass;
                j.impulseAxis += lambda;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * axis);
            }
        }

        if (j.motorEnabled) {
            const Vec3 raCrossAxis = Cross(ra, axis);
            const Vec3 rbCrossAxis = Cross(rb, axis);
            const float effMass = a.invMass + b.invMass
                + Dot(raCrossAxis, invIA * raCrossAxis)
                + Dot(rbCrossAxis, invIB * rbCrossAxis);
            if (effMass > kEpsilon) {
                float lambda = -(Dot(relVel, axis) - j.motorSpeed) / effMass;
                const float oldImpulse = j.motorImpulseSum;
                j.motorImpulseSum = std::clamp(j.motorImpulseSum + lambda, -j.maxMotorForce, j.maxMotorForce);
                lambda = j.motorImpulseSum - oldImpulse;
                ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda * axis);
            }
        }
    }

void World::SolveServoJoint(ServoJoint& j) {
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
            || relAngSpeed > kWakeJointRelativeSpeedThreshold) {
            WakeConnectedBodies(j.a);
            WakeConnectedBodies(j.b);
        }

        Vec3 lambda{};
        JointBlockFallbackReason fallbackReason = JointBlockFallbackReason::None;
        if (SolveHingeAnchorBlock3x3(a, b, invIA, invIB, ra, rb, error, relVel, lambda, fallbackReason)) {
            j.impulseX += lambda.x;
            j.impulseY += lambda.y;
            j.impulseZ += lambda.z;
            ApplyImpulse(a, b, invIA, invIB, ra, rb, lambda);
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
        const float dt = currentSubstepDt_;
        // Keep the shared axis-alignment solve stiffer than the hinge-angle target so identical
        // leg servos hold their shape under load instead of behaving like loose ball joints.
        constexpr float kAxisAlignOmega = 260.0f;
        constexpr float kAxisAlignZeta  = 1.12f;
        const float axisDenom = 2.0f * kAxisAlignZeta + dt * kAxisAlignOmega;
        for (int i = 0; i < 2; ++i) {
            const Vec3 n = angAxes[i];
            const float w = Dot(n, invIA * n) + Dot(n, invIB * n);
            if (w <= kEpsilon) {
                continue;
            }
            const float axisGamma = w / (dt * kAxisAlignOmega * std::max(axisDenom, kEpsilon));
            const float axisBiasVel = kAxisAlignOmega * Dot(angularError, n)
                                    / std::max(axisDenom, kEpsilon);
            float angLambda = -(Dot(relAngVel, n) + axisBiasVel) / (w + axisGamma);
            const float oldAxisImpulse = *angImpulseSums[i];
            *angImpulseSums[i] = std::clamp(oldAxisImpulse + angLambda, -j.maxServoTorque, j.maxServoTorque);
            angLambda = *angImpulseSums[i] - oldAxisImpulse;
            ApplyAngularImpulse(a, b, invIA, invIB, angLambda * n);
        }

        const Vec3 refA = ResolveJointReference(a.orientation, j.localReferenceA, axisA);
        const Vec3 refB = ResolveJointReference(b.orientation, j.localReferenceB, axisA);
        const float hingeAngle = SignedAngleAroundAxis(refA, refB, axisA);
        const float rawPositionError = core_internal::WrapJointAngle(hingeAngle - j.targetAngle);
        const float positionError = (j.positionErrorSmoothing > 0.0f) ? j.smoothedAngleError : rawPositionError;
        const float omegaAxis = Dot(relAngVel, axisA);
        const float w = Dot(axisA, invIA * axisA) + Dot(axisA, invIB * axisA);
        if (w > kEpsilon) {
            const float omega = j.positionGain;
            const float zeta = j.dampingGain;
            const float denom = 2.0f * zeta + dt * omega;
            const float gamma = w / (dt * omega * std::max(denom, kEpsilon));
            const float clampedError = std::clamp(positionError, -j.maxCorrectionAngle, j.maxCorrectionAngle);
            float biasVel = omega * clampedError / std::max(denom, kEpsilon)
                          + j.integralGain * j.integralAccum;
            if (j.maxServoSpeed > 0.0f && std::isfinite(j.maxServoSpeed)) {
                biasVel = std::clamp(biasVel, -j.maxServoSpeed, j.maxServoSpeed);
            }
            float servoLambda = -(omegaAxis + biasVel) / (w + gamma);
            const float oldImpulse = j.servoImpulseSum;
            j.servoImpulseSum = std::clamp(j.servoImpulseSum + servoLambda, -j.maxServoTorque, j.maxServoTorque);
            servoLambda = j.servoImpulseSum - oldImpulse;
            ApplyAngularImpulse(a, b, invIA, invIB, servoLambda * axisA);
            if (j.maxServoSpeed > 0.0f && std::isfinite(j.maxServoSpeed)) {
                const float postOmegaAxis = Dot(b.angularVelocity - a.angularVelocity, axisA);
                const float clampedOmegaAxis = std::clamp(postOmegaAxis, -j.maxServoSpeed, j.maxServoSpeed);
                if (std::abs(clampedOmegaAxis - postOmegaAxis) > 1e-6f) {
                    float speedLambda = (clampedOmegaAxis - postOmegaAxis) / std::max(w, kEpsilon);
                    const float speedImpulse = std::clamp(j.servoImpulseSum + speedLambda, -j.maxServoTorque, j.maxServoTorque);
                    speedLambda = speedImpulse - j.servoImpulseSum;
                    j.servoImpulseSum = speedImpulse;
                    ApplyAngularImpulse(a, b, invIA, invIB, speedLambda * axisA);
                }
            }
        }
    }


void World::SolveContactsInManifold(Manifold& manifold) {
        core_internal::ContactSolver solver;
        std::unordered_map<ManifoldKey, std::unordered_set<PersistentPointKey, PersistentPointKeyHash>, ManifoldKeyHash> warmStartUsedKeys;
        core_internal::ContactSolverContext context{
            bodies_,
            contacts_,
            manifolds_,
            previousManifolds_,
            [this, &warmStartUsedKeys](const ManifoldKey& manifoldId, const Contact& contact, float& normal, std::array<float, 2>& tangent, std::uint16_t& age) {
                PersistentPointMatchCandidate match{};
                std::unordered_set<PersistentPointKey, PersistentPointKeyHash>& usedKeys = warmStartUsedKeys[manifoldId];
                if (!TryMatchPersistentPoint(persistentPointImpulses_, manifoldId, contact, usedKeys, match)) {
                    return false;
                }
                usedKeys.insert(match.key);
                normal = match.state.normalImpulseSum;
                tangent = {match.state.tangentImpulseSum0, match.state.tangentImpulseSum1};
                age = match.state.persistenceAge;
                return true;
            },
            [this](Manifold& manifold, const Manifold* previous) { ManageManifoldContacts(manifold, previous); },
            [](Manifold& manifold) { RefreshManifoldBlockCache(manifold); },
            [this](Manifold& manifold) { SelectBlockSolvePair(manifold); },
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            [this](const Manifold& manifold) { RecordSelectedPairHistory(manifold); },
#else
            [](const Manifold&) {},
#endif
            [this](Manifold& m) { SolveManifoldNormalImpulses(m); },
            [](const Manifold& m, std::uint64_t key) { return FindBlockSlot(m, key); },
            [this](Body& a, Body& b, const Mat3& invIA, const Mat3& invIB, const Vec3& ra, const Vec3& rb, const Vec3& impulse) {
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
            },
            [this](bool reusedBasis) {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                if (reusedBasis) {
                    ++solverTelemetry_.tangentBasisReused;
                } else {
                    ++solverTelemetry_.tangentBasisResets;
                }
#else
                (void)reusedBasis;
#endif
            },
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            [this](FrictionBudgetNormalSupportSource source) {
                ++solverTelemetry_.manifoldFrictionBudgetSaturated;
                switch (source) {
                    case FrictionBudgetNormalSupportSource::SelectedBlockPairOnly:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedSelectedPair;
                        break;
                    case FrictionBudgetNormalSupportSource::AllManifoldContacts:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedAllContacts;
                        break;
                    case FrictionBudgetNormalSupportSource::BlendedSelectedPairAndManifold:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedBlended;
                        break;
                }
            },
            [this](bool reprojected) {
                if (reprojected) {
                    ++solverTelemetry_.tangentImpulseReprojected;
                } else {
                    ++solverTelemetry_.tangentImpulseReset;
                }
            },
#endif
            contactSolverConfig_,
        };
        solver.SolveContactsInManifold(context, manifold);
    }

void World::SolveJointPositions() {
        core_internal::JointSolver jointSolver;
        core_internal::JointSolverContext context{
            bodies_,
            joints_,
            hingeJoints_,
            ballSocketJoints_,
            fixedJoints_,
            prismaticJoints_,
            servoJoints_,
            jointSolverConfig_.servoPositionPasses,
            gravity_,
            currentSubstepDt_,
        };
        jointSolver.SolveJointPositions(context);
    }

void World::BeginSplitImpulseSubstep() {
        splitLinearPositionDelta_.assign(bodies_.size(), Vec3{0.0f, 0.0f, 0.0f});
        splitAngularPositionDelta_.assign(bodies_.size(), Vec3{0.0f, 0.0f, 0.0f});
    }

void World::AccumulateSplitImpulseCorrection(std::uint32_t bodyId, const Vec3& linearDelta, const Vec3& angularDelta) {
        if (bodyId >= splitLinearPositionDelta_.size()) {
            return;
        }
        if (bodies_[bodyId].isSleeping) {
            return;
        }
        splitLinearPositionDelta_[bodyId] += linearDelta;
        splitAngularPositionDelta_[bodyId] += angularDelta;
    }

void World::ApplySplitStabilization() {
        if (!contactSolverConfig_.useSplitImpulse) {
            return;
        }
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            Body& body = bodies_[i];
            if (body.invMass == 0.0f || body.isSleeping) {
                continue;
            }
            body.position += splitLinearPositionDelta_[i];
        }
    }

bool World::TryComputeAnchorSeparation(const Contact& c, float& outPenetration) const {
        if (!c.anchorsValid || c.a >= bodies_.size() || c.b >= bodies_.size()) {
            return false;
        }
        const Body& a = bodies_[c.a];
        const Body& b = bodies_[c.b];
        const Vec3 worldAnchorA = a.position + Rotate(a.orientation, c.localAnchorA);
        const Vec3 worldAnchorB = b.position + Rotate(b.orientation, c.localAnchorB);
        const float separation = Dot(worldAnchorB - worldAnchorA, c.normal);
        const float penetration = std::max(0.0f, c.referenceSeparation - separation);
        if (!std::isfinite(penetration)) {
            return false;
        }
        outPenetration = penetration;
        return true;
    }

void World::PrepareIslandOrders() {
    islandOrders_.clear();
    islandOrders_.reserve(islands_.size());
    for (const Island& island : islands_) {
        islandOrders_.push_back(
            solver_internal::ComputeIslandOrder(island, bodies_, manifolds_, contactSolverConfig_));
    }
}

void World::SolveIslands() {
        std::unordered_map<ManifoldKey, std::unordered_set<PersistentPointKey, PersistentPointKeyHash>, ManifoldKeyHash> warmStartUsedKeys;
        core_internal::ContactSolverContext contactContext{
            bodies_,
            contacts_,
            manifolds_,
            previousManifolds_,
            [this, &warmStartUsedKeys](const ManifoldKey& manifoldId, const Contact& contact, float& normal, std::array<float, 2>& tangent, std::uint16_t& age) {
                PersistentPointMatchCandidate match{};
                std::unordered_set<PersistentPointKey, PersistentPointKeyHash>& usedKeys = warmStartUsedKeys[manifoldId];
                if (!TryMatchPersistentPoint(persistentPointImpulses_, manifoldId, contact, usedKeys, match)) {
                    return false;
                }
                usedKeys.insert(match.key);
                normal = match.state.normalImpulseSum;
                tangent = {match.state.tangentImpulseSum0, match.state.tangentImpulseSum1};
                age = match.state.persistenceAge;
                return true;
            },
            [this](Manifold& manifold, const Manifold* previous) { ManageManifoldContacts(manifold, previous); },
            [](Manifold& manifold) { RefreshManifoldBlockCache(manifold); },
            [this](Manifold& manifold) { SelectBlockSolvePair(manifold); },
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            [this](const Manifold& manifold) { RecordSelectedPairHistory(manifold); },
#else
            [](const Manifold&) {},
#endif
            [this](Manifold& manifold) { SolveManifoldNormalImpulses(manifold); },
            [](const Manifold& manifold, std::uint64_t contactKey) { return FindBlockSlot(manifold, contactKey); },
            [this](Body& a, Body& b, const Mat3& invIA, const Mat3& invIB, const Vec3& ra, const Vec3& rb, const Vec3& impulse) {
                ApplyImpulse(a, b, invIA, invIB, ra, rb, impulse);
            },
            [this](bool reusedBasis) {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                if (reusedBasis) {
                    ++solverTelemetry_.tangentBasisReused;
                } else {
                    ++solverTelemetry_.tangentBasisResets;
                }
#else
                (void)reusedBasis;
#endif
            },
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            [this](FrictionBudgetNormalSupportSource source) {
                ++solverTelemetry_.manifoldFrictionBudgetSaturated;
                switch (source) {
                    case FrictionBudgetNormalSupportSource::SelectedBlockPairOnly:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedSelectedPair;
                        break;
                    case FrictionBudgetNormalSupportSource::AllManifoldContacts:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedAllContacts;
                        break;
                    case FrictionBudgetNormalSupportSource::BlendedSelectedPairAndManifold:
                        ++solverTelemetry_.manifoldFrictionBudgetSaturatedBlended;
                        break;
                }
            },
            [this](bool reprojected) {
                if (reprojected) {
                    ++solverTelemetry_.tangentImpulseReprojected;
                } else {
                    ++solverTelemetry_.tangentImpulseReset;
                }
            },
#endif
            contactSolverConfig_,
        };

        const core_internal::ConstraintSolver constraintSolver;
        const core_internal::ConstraintSolverContext context{
            bodies_,
            manifolds_,
            islands_,
            joints_,
            hingeJoints_,
            ballSocketJoints_,
            fixedJoints_,
            prismaticJoints_,
            servoJoints_,
            contactContext,
            contactSolverConfig_,
            islandOrders_.empty() ? nullptr : &islandOrders_,
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            &solverTelemetry_,
#endif
            [](core_internal::ContactSolver& solver, const core_internal::ContactSolverContext& context, Manifold& manifold) {
                solver.SolveContactsInManifold(context, manifold);
            },
            [this](DistanceJoint& joint) { SolveDistanceJoint(joint); },
            [this](HingeJoint& joint) { SolveHingeJoint(joint); },
            [this](BallSocketJoint& joint) { SolveBallSocketJoint(joint); },
            [this](FixedJoint& joint) { SolveFixedJoint(joint); },
            [this](PrismaticJoint& joint) { SolvePrismaticJoint(joint); },
            [this](ServoJoint& joint) { SolveServoJoint(joint); },
        };

        constraintSolver.SolveIslands(context);
    }

void World::UpdateSleeping() {
        const core_internal::SleepSystem sleepSystem;
        const core_internal::SleepSystemContext context{
            bodies_,
            manifolds_,
            joints_,
            hingeJoints_,
            ballSocketJoints_,
            fixedJoints_,
            prismaticJoints_,
            servoJoints_,
            kSleepLinearThreshold,
            kSleepAngularThreshold,
            kSleepFramesThreshold,
        };
        sleepSystem.UpdateSleeping(context);
    }


} // namespace minphys3d
