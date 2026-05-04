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

// Octant-reduction atan2 approximation.
// Max error ~0.005 rad — adequate for physics Baumgarte bias (multiplied by ~0.2).
float FastAtan2(float y, float x) {
    constexpr float kPiOver2 = 1.5707963f;
    constexpr float kPi      = 3.1415927f;
    const float absX = std::abs(x);
    const float absY = std::abs(y);
    const float mn = absX < absY ? absX : absY;
    const float mx = absX > absY ? absX : absY;
    if (mx < 1e-10f) return 0.0f;
    const float t  = mn / mx;
    const float t2 = t * t;
    // Degree-5 minimax polynomial for atan(t) on [0, 1]
    float r = t * (0.9997878f + t2 * (-0.3258560f + t2 * 0.1520524f));
    if (absY > absX) r = kPiOver2 - r;
    if (x  < 0.0f ) r = kPi      - r;
    if (y  < 0.0f ) r = -r;
    return r;
}

float SignedAngleAroundAxis(const Vec3& from, const Vec3& to, const Vec3& axis) {
    Vec3 fromN = from;
    Vec3 toN = to;
    if (!TryNormalize(fromN, fromN) || !TryNormalize(toN, toN)) {
        return 0.0f;
    }
    return FastAtan2(Dot(Cross(fromN, toN), axis), Dot(fromN, toN));
}

bool ComputeUseVelocityBiasesForServoPosition(
    const std::vector<ServoJoint>& servoJoints,
    std::size_t bodyCount,
    std::vector<std::uint16_t>& fanoutScratch) {
    constexpr std::uint16_t kFanoutThreshold = 2;
    fanoutScratch.assign(bodyCount, 0);
    std::uint16_t maxFanout = 0;
    for (const ServoJoint& sj : servoJoints) {
        ++fanoutScratch[sj.a];
        ++fanoutScratch[sj.b];
    }
    for (const std::uint16_t f : fanoutScratch) {
        maxFanout = std::max(maxFanout, f);
    }
    return maxFanout > kFanoutThreshold;
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
                                const Mat3& invIA = bodyInvInertiaWorld_[c.a];
                                const Mat3& invIB = bodyInvInertiaWorld_[c.b];
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
                const Mat3& invIA = bodyInvInertiaWorld_[c.a];
                const Mat3& invIB = bodyInvInertiaWorld_[c.b];
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
        const std::uint32_t bodyAIndex = c0.a;
        const std::uint32_t bodyBIndex = c0.b;

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

        const Mat3& invIA = bodyInvInertiaWorld_[bodyAIndex];
        const Mat3& invIB = bodyInvInertiaWorld_[bodyBIndex];
        // Look up cached per-contact kinematics (ra, rb, normalMass) computed once per
        // substep by PrepareContactSolves. The block solver internals still rebuild K
        // from invIA/invIB/ra/rb (separate optimisation target); here we only avoid
        // recomputing the rhs effective masses on every PGS iteration.
        const std::size_t mIdx = static_cast<std::size_t>(&manifold - manifolds_.data());
        const ManifoldPrep& mPrep = manifoldPreps_[mIdx];
        const ContactPrep& prep0 = mPrep.contacts[static_cast<std::size_t>(idx0)];
        const ContactPrep& prep1 = mPrep.contacts[static_cast<std::size_t>(idx1)];
        const Vec3& ra0 = prep0.ra;
        const Vec3& rb0 = prep0.rb;
        const Vec3& ra1 = prep1.ra;
        const Vec3& rb1 = prep1.rb;

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

        const float k11 = prep0.normalMass;
        const float k22 = prep1.normalMass;

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

        const std::uint32_t bodyAIndex = manifold.contacts[0].a;
        const std::uint32_t bodyBIndex = manifold.contacts[0].b;
        Body& a = bodies_[bodyAIndex];
        Body& b = bodies_[bodyBIndex];
        const Mat3& invIA = bodyInvInertiaWorld_[bodyAIndex];
        const Mat3& invIB = bodyInvInertiaWorld_[bodyBIndex];
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

        const std::size_t mIdx = static_cast<std::size_t>(&manifold - manifolds_.data());
        const ManifoldPrep& mPrep = manifoldPreps_[mIdx];
        for (int i = 0; i < 4; ++i) {
            Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            const Vec3 normal = Normalize(c.normal);
            if (Dot(normal, manifoldNormal) < 0.95f) {
                fallbackReason = BlockSolveFallbackReason::ContactNormalMismatch;
                return false;
            }
            const ContactPrep& cPrep = mPrep.contacts[static_cast<std::size_t>(i)];
            const Vec3& ra = cPrep.ra;
            const Vec3& rb = cPrep.rb;
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
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];

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
        float K[3][3]{};
        const float invMassSum = a.invMass + b.invMass;
        const Vec3 axes[3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };
        Vec3 raCrossAxis[3]{};
        Vec3 rbCrossAxis[3]{};
        for (int i = 0; i < 3; ++i) {
            raCrossAxis[i] = Cross(ra, axes[i]);
            rbCrossAxis[i] = Cross(rb, axes[i]);
        }
        for (int i = 0; i < 3; ++i) {
            const Vec3 invIaCrossI = invIA * raCrossAxis[i];
            const Vec3 invIbCrossI = invIB * rbCrossAxis[i];
            for (int j = 0; j < 3; ++j) {
                K[i][j] = (i == j ? invMassSum : 0.0f)
                    + Dot(invIaCrossI, raCrossAxis[j])
                    + Dot(invIbCrossI, rbCrossAxis[j]);
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
        // All geometry (ra, rb, axisA, t1, t2, invK, effective masses, hingeAngle) is
        // precomputed once per substep in PrepareHingeJointSolves and cached in the prep.
        // This function is called N_iter times per substep; only velocity-dependent terms
        // (relVel, relAngVel) are recomputed here.
        const std::size_t idx = static_cast<std::size_t>(&j - hingeJoints_.data());
        const HingeJointPrep& prep = hingeJointPreps_[idx];

        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];

        // ---- Anchor 3D constraint ----
        if (jointSolverConfig_.useBlockSolver && prep.anchorValid) {
            const Vec3 va = a.velocity + Cross(a.angularVelocity, prep.ra);
            const Vec3 vb = b.velocity + Cross(b.angularVelocity, prep.rb);
            const Vec3 relVel = vb - va;
            const Vec3 rhs = jointSolverConfig_.hingeAnchorDampingFactor * relVel + prep.anchorBias;
            const Vec3 lambda = -1.0f * (prep.invK * rhs);
            if (std::isfinite(lambda.x) && std::isfinite(lambda.y) && std::isfinite(lambda.z)) {
                j.impulseX += lambda.x;
                j.impulseY += lambda.y;
                j.impulseZ += lambda.z;
                ApplyImpulse(a, b, invIA, invIB, prep.ra, prep.rb, lambda);
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                ++solverTelemetry_.jointBlockSolveUsed;
#endif
            }
        } else {
            // Per-axis scalar fallback (useBlockSolver==false or degenerate K).
            const Vec3 va = a.velocity + Cross(a.angularVelocity, prep.ra);
            const Vec3 vb = b.velocity + Cross(b.angularVelocity, prep.rb);
            const Vec3 relVel = vb - va;
            const Vec3 axes[3] = {{1,0,0},{0,1,0},{0,0,1}};
            float* impulseSums[3] = {&j.impulseX, &j.impulseY, &j.impulseZ};
            for (int i = 0; i < 3; ++i) {
                if (prep.fallbackEffMass[i] <= kEpsilon) continue;
                const float bias = prep.fallbackPosBias[i]
                    + jointSolverConfig_.hingeAnchorDampingFactor * Dot(relVel, axes[i]);
                const float lambda = -bias / prep.fallbackEffMass[i];
                *impulseSums[i] += lambda;
                ApplyImpulse(a, b, invIA, invIB, prep.ra, prep.rb, lambda * axes[i]);
            }
        }

        // ---- Angular alignment (t1, t2 ⊥ axisA) ----
        if (prep.t1Active) {
            const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
            const float bias = prep.angBiasT1 + 0.1f * Dot(relAngVel, prep.t1);
            float lambda = -bias * prep.invDenomT1;
            if (j.maxMotorTorque > 0.0f) {
                const float old = j.angularImpulse1;
                j.angularImpulse1 = std::clamp(old + lambda, -j.maxMotorTorque, j.maxMotorTorque);
                lambda = j.angularImpulse1 - old;
            } else {
                j.angularImpulse1 += lambda;
            }
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * prep.t1);
        }
        if (prep.t2Active) {
            const Vec3 relAngVel = b.angularVelocity - a.angularVelocity; // re-read after t1 impulse
            const float bias = prep.angBiasT2 + 0.1f * Dot(relAngVel, prep.t2);
            float lambda = -bias * prep.invDenomT2;
            if (j.maxMotorTorque > 0.0f) {
                const float old = j.angularImpulse2;
                j.angularImpulse2 = std::clamp(old + lambda, -j.maxMotorTorque, j.maxMotorTorque);
                lambda = j.angularImpulse2 - old;
            } else {
                j.angularImpulse2 += lambda;
            }
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * prep.t2);
        }

        if (!prep.hingeEffMassActive) {
            return;
        }

        // ---- Hinge angle limits ----
        if (j.limitsEnabled) {
            float limitError = 0.0f;
            if (prep.hingeAngle < j.lowerAngle) {
                limitError = prep.hingeAngle - j.lowerAngle;
            } else if (prep.hingeAngle > j.upperAngle) {
                limitError = prep.hingeAngle - j.upperAngle;
            }
            if (std::abs(limitError) > kEpsilon) {
                const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
                const float lambda = -(0.2f * limitError + 0.05f * Dot(relAngVel, prep.axisA))
                                     * prep.invEffMassHinge;
                ApplyAngularImpulse(a, b, invIA, invIB, lambda * prep.axisA);
            }
        }

        // ---- Motor ----
        if (j.motorEnabled) {
            const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
            float lambda = -(Dot(relAngVel, prep.axisA) - j.motorSpeed) * prep.invEffMassHinge;
            const float oldImpulse = j.motorImpulseSum;
            j.motorImpulseSum = std::clamp(j.motorImpulseSum + lambda, -j.maxMotorTorque, j.maxMotorTorque);
            lambda = j.motorImpulseSum - oldImpulse;
            ApplyAngularImpulse(a, b, invIA, invIB, lambda * prep.axisA);
        }
    }

void World::SolveBallSocketJoint(BallSocketJoint& j) {
        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];

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
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];

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
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];

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

void World::PrepareContactSolves() {
        const std::size_t numManifolds = manifolds_.size();
        if (manifoldPreps_.size() < numManifolds) {
            manifoldPreps_.resize(numManifolds);
        }
        for (std::size_t mi = 0; mi < numManifolds; ++mi) {
            const Manifold& m = manifolds_[mi];
            ManifoldPrep& prep = manifoldPreps_[mi];
            const std::size_t numContacts = m.contacts.size();
            if (prep.contacts.size() != numContacts) {
                prep.contacts.resize(numContacts);
            }
            for (std::size_t ci = 0; ci < numContacts; ++ci) {
                const Contact& c = m.contacts[ci];
                ContactPrep& cp = prep.contacts[ci];
                const Body& a = bodies_[c.a];
                const Body& b = bodies_[c.b];
                const Mat3& invIA = bodyInvInertiaWorld_[c.a];
                const Mat3& invIB = bodyInvInertiaWorld_[c.b];
                cp.ra = c.point - a.position;
                cp.rb = c.point - b.position;
                cp.raCrossN = Cross(cp.ra, c.normal);
                cp.rbCrossN = Cross(cp.rb, c.normal);
                const float angularTermA = Dot(cp.raCrossN, invIA * cp.raCrossN);
                const float angularTermB = Dot(cp.rbCrossN, invIB * cp.rbCrossN);
                float sideA = a.invMass + angularTermA;
                float sideB = b.invMass + angularTermB;
                if (c.a < artChainIndexForLeafBody_.size()) {
                    const std::uint32_t ac = artChainIndexForLeafBody_[c.a];
                    if (ac != kInvalidArticulationChain && ac < articulationChains_.size()) {
                        const ArtChain& ch = articulationChains_[ac];
                        if (!ch.Ia.empty() && !ch.links.empty()) {
                            const Vec3  refRoot   = ch.links[0].jointAnchorWorld;
                            const Vec3  raFromRef = c.point - refRoot;
                            const SpatialVec h{Cross(raFromRef, c.normal), c.normal};
                            const float comp = SpatialMotionCompliance(ch.Ia[0], h);
                            if (std::isfinite(comp) && comp > kEpsilon && comp < 1e8f) {
                                sideA = comp;
                            }
                        }
                    }
                }
                if (c.b < artChainIndexForLeafBody_.size()) {
                    const std::uint32_t bc = artChainIndexForLeafBody_[c.b];
                    if (bc != kInvalidArticulationChain && bc < articulationChains_.size()) {
                        const ArtChain& ch = articulationChains_[bc];
                        if (!ch.Ia.empty() && !ch.links.empty()) {
                            const Vec3  refRoot   = ch.links[0].jointAnchorWorld;
                            const Vec3  rbFromRef = c.point - refRoot;
                            const SpatialVec h{Cross(rbFromRef, c.normal), c.normal};
                            const float comp = SpatialMotionCompliance(ch.Ia[0], h);
                            if (std::isfinite(comp) && comp > kEpsilon && comp < 1e8f) {
                                sideB = comp;
                            }
                        }
                    }
                }
                cp.normalMass = sideA + sideB;
                cp.invNormalMass = (cp.normalMass > kEpsilon) ? (1.0f / cp.normalMass) : 0.0f;
            }
        }
    }

// ---------------------------------------------------------------------------
// BuildArticulationChains
//   Detect serial kinematic chains (trees of ServoJoints) in the scene and
//   populate articulationChains_.  Called once per substep after BuildIslands().
//
//   Algorithm:
//   1. Build a child→parent map from the ServoJoint graph:
//        childBody[j.b] = {j.a, joint_index}
//   2. Root bodies are bodies that appear as j.a but never as j.b in any
//      ServoJoint (i.e. they have outgoing joints but no incoming joint).
//   3. DFS from each root, tracing every root→leaf path.  Each distinct path
//      becomes one ArtChain.  Branching bodies (e.g. a chassis with 6 legs)
//      spawn one chain per child branch.
//   4. Mark every joint in a detected chain with inArticulationChain = true.
//
//   Zero behaviour impact in Phase 1a: inArticulationChain is set but no
//   solver code reads it yet.
// ---------------------------------------------------------------------------

void World::BuildArticulationChains() {
    // Reset — rebuild from scratch each substep (topology changes are rare but
    // must be handled correctly, e.g. after a body is added mid-simulation).
    articulationChains_.clear();

    const std::size_t nJoints = servoJoints_.size();
    if (nJoints == 0) {
        return;
    }

    // Step 1: clear stale articulation flags, build child→parent adjacency.
    for (ServoJoint& j : servoJoints_) {
        j.inArticulationChain = false;
    }

    // childParent[body_b] = joint index  (only if body_b appears exactly once as b)
    // Use max-value as "not set" sentinel.
    constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();
    const std::size_t nBodies = bodies_.size();
    // childJoint[bodyIdx] = index of the ServoJoint that has bodyIdx as j.b,
    //                        or kNone if bodyIdx never appears as j.b (i.e. it is a root or unconnected).
    std::vector<std::uint32_t> childJoint(nBodies, kNone);
    // outgoing[bodyIdx] = list of ServoJoint indices with j.a == bodyIdx
    std::vector<std::vector<std::uint32_t>> outgoing(nBodies);

    for (std::uint32_t ji = 0; ji < static_cast<std::uint32_t>(nJoints); ++ji) {
        const ServoJoint& j = servoJoints_[ji];
        // If body_b already appears as b in another joint, it is not a simple serial chain.
        // Mark as kNone-1 (any non-kNone value) to indicate ambiguity; we won't include it.
        constexpr std::uint32_t kAmbiguous = kNone - 1u;
        if (j.b < nBodies) {
            if (childJoint[j.b] == kNone) {
                childJoint[j.b] = ji;
            } else {
                // body_b has more than one incoming joint — not a simple tree child
                childJoint[j.b] = kAmbiguous;
            }
        }
        if (j.a < nBodies) {
            outgoing[j.a].push_back(ji);
        }
    }

    // Step 2: find root bodies (have outgoing joints, are never a j.b in a simple chain).
    // Stack-based DFS to trace paths and build chains.
    struct DFSFrame {
        std::uint32_t bodyIdx;  // current body
        int           parentLinkIdx;  // index in chain.links of parent; -1 for root
        std::uint32_t incomingJointIdx;  // joint from parent to this body; kNone for root
        std::size_t   chainIdx;  // which ArtChain this path belongs to
    };

    std::vector<DFSFrame> stack;
    stack.reserve(32);

    for (std::uint32_t bodyIdx = 0; bodyIdx < static_cast<std::uint32_t>(nBodies); ++bodyIdx) {
        // Root: has at least one outgoing joint AND never appears as a j.b in any joint.
        if (outgoing[bodyIdx].empty()) continue;
        if (childJoint[bodyIdx] != kNone) continue;  // has an incoming joint → not a root

        // Start a DFS from this root body for each of its outgoing joints.
        for (const std::uint32_t firstJointIdx : outgoing[bodyIdx]) {
            const ServoJoint& firstJoint = servoJoints_[firstJointIdx];
            // Skip if the child body has ambiguous parentage.
            constexpr std::uint32_t kAmbiguous = kNone - 1u;
            if (childJoint[firstJoint.b] == kAmbiguous) continue;

            // Begin a new chain with root at bodyIdx.
            const std::size_t chainIdx = articulationChains_.size();
            articulationChains_.emplace_back();
            ArtChain& chain = articulationChains_.back();

            // Root link (no parent joint).
            ArtLink rootLink{};
            rootLink.bodyIdx            = bodyIdx;
            rootLink.jointIdx           = ArtLink::kNoJoint;
            rootLink.parent             = -1;
            rootLink.jointAxisLocal     = {};
            // Spatial inertia reference for the floating base: first joint anchor on this branch
            // (matches the child link's incoming joint pivot so ABI merges share a common frame).
            {
                const ServoJoint& fj = servoJoints_[firstJointIdx];
                const Body&      raBody = bodies_[fj.a];
                rootLink.jointAnchorWorld =
                    raBody.position + Rotate(raBody.orientation, fj.localAnchorA);
            }
            chain.links.push_back(rootLink);

            // Push the first child joint onto the DFS stack.
            stack.push_back({firstJoint.b, /*parentLinkIdx=*/0, firstJointIdx, chainIdx});

            while (!stack.empty()) {
                const DFSFrame frame = stack.back();
                stack.pop_back();

                ArtChain& c = articulationChains_[frame.chainIdx];
                const ServoJoint& joint = servoJoints_[frame.incomingJointIdx];

                // Compute world-space anchor (body_a position + rotated local anchor)
                const Body& bodyA = bodies_[joint.a];
                const Vec3 anchorWorld =
                    bodyA.position + Rotate(bodyA.orientation, joint.localAnchorA);

                const int thisLinkIdx = static_cast<int>(c.links.size());
                ArtLink link{};
                link.bodyIdx          = frame.bodyIdx;
                link.jointIdx         = frame.incomingJointIdx;
                link.parent           = frame.parentLinkIdx;
                link.jointAxisLocal   = joint.localAxisA;
                link.jointAnchorWorld = anchorWorld;
                c.links.push_back(link);

                // Mark joint as belonging to an articulation chain.
                servoJoints_[frame.incomingJointIdx].inArticulationChain = true;

                // Push outgoing joints from this body (branches become separate entries
                // on the same chain for now; for a hexapod each link has at most one child).
                if (frame.bodyIdx < nBodies) {
                    for (const std::uint32_t childJointIdx : outgoing[frame.bodyIdx]) {
                        const ServoJoint& cj = servoJoints_[childJointIdx];
                        if (childJoint[cj.b] == kAmbiguous) continue;
                        stack.push_back({cj.b, thisLinkIdx, childJointIdx, frame.chainIdx});
                    }
                }
            }

            // Finalise: resize per-substep caches.
            articulationChains_.back().resizeCaches();
        }
    }
}

// ---------------------------------------------------------------------------
// PrepareArticulatedInertias
//   Tip→root articulated-body inertia pass; updates ServoJointPrep hinge
//   denominators for in-chain servos and fills artChainIndexForLeafBody_ for contacts.
// ---------------------------------------------------------------------------

void World::PrepareArticulatedInertias() {
    const std::size_t nBodies = bodies_.size();
    artChainIndexForLeafBody_.assign(nBodies, kInvalidArticulationChain);

    if (servoJointPreps_.size() != servoJoints_.size()) {
        return;
    }

    // Forward world inertia (local inverse → local forward → world); only used by this pass
    // so we do not alter the global inverse-inertia cache behaviour for non-articulation scenes.
    bodyInertiaWorld_.resize(nBodies);
    for (std::size_t i = 0; i < nBodies; ++i) {
        const Body& body = bodies_[i];
        if (body.invMass <= 0.0f) {
            bodyInertiaWorld_[i] = Mat3{};
            continue;
        }
        const Mat3 R = RotationMatrix(body.orientation);
        Mat3       Ilocal{};
        if (!InvertMat3(body.invInertiaLocal, Ilocal)) {
            bodyInertiaWorld_[i] = Mat3{};
        } else {
            bodyInertiaWorld_[i] = R * Ilocal * Transpose(R);
        }
    }

    const float dt = currentSubstepDt_;
    const SpatialVec spatialGravity{Vec3{0.0f, 0.0f, 0.0f}, gravity_};
    const bool       velocityPreCorr = articulationConfig_.enableVelocityPreCorrection;

    for (std::size_t ci = 0; ci < articulationChains_.size(); ++ci) {
        ArtChain& chain = articulationChains_[ci];
        const std::size_t N = chain.links.size();
        if (N < 2) {
            continue;
        }
        chain.resizeCaches();

        for (std::size_t li = 0; li < N; ++li) {
            const ArtLink& L = chain.links[li];
            const Body&    bd = bodies_[L.bodyIdx];
            if (bd.invMass <= 0.0f || !(bd.mass > kEpsilon)) {
                chain.Ia[li] = SpatialInertia{};
                continue;
            }
            chain.Ia[li] = SpatialInertiaFromBody(
                bd.mass,
                bodyInertiaWorld_[L.bodyIdx],
                bd.position,
                L.jointAnchorWorld);
        }

        if (velocityPreCorr) {
            for (std::size_t li = 0; li < N; ++li) {
                chain.rigidSpatialI[li] = chain.Ia[li];
            }

            // Pass A — spatial motion at each link joint anchor (no body writes).
            {
                const Body& rootBd = bodies_[chain.links[0].bodyIdx];
                const Vec3  anchor0 = chain.links[0].jointAnchorWorld;
                const Vec3  rCom0   = anchor0 - rootBd.position;
                chain.vel[0].ang = rootBd.angularVelocity;
                chain.vel[0].lin = rootBd.velocity + Cross(rootBd.angularVelocity, rCom0);
            }
            for (std::size_t li = 1; li < N; ++li) {
                const ArtLink& L  = chain.links[li];
                const ArtLink& Lp = chain.links[li - 1u];
                const std::uint32_t jidx = L.jointIdx;
                if (jidx == ArtLink::kNoJoint || jidx >= servoJoints_.size()) {
                    chain.vel[li] = SpatialVec{};
                    continue;
                }
                const ServoJoint& sj       = servoJoints_[jidx];
                const Body&       parentBd = bodies_[sj.a];
                const Body&       childBd  = bodies_[sj.b];
                Vec3              axis     = Rotate(parentBd.orientation, sj.localAxisA);
                if (!TryNormalize(axis, axis)) {
                    chain.vel[li] = SpatialVec{};
                    continue;
                }
                const float dq = Dot(childBd.angularVelocity - parentBd.angularVelocity, axis);
                const SpatialVec& vParent = chain.vel[li - 1u];
                const Vec3        offset  = L.jointAnchorWorld - Lp.jointAnchorWorld;
                const Vec3        vAtChild = vParent.lin + Cross(vParent.ang, offset);
                chain.vel[li].ang = childBd.angularVelocity;
                chain.vel[li].lin = vAtChild + axis * dq;
            }

            for (std::size_t li = 0; li < N; ++li) {
                chain.Pa[li] = SpatialBias(chain.rigidSpatialI[li], chain.vel[li])
                    - SpatialMul(chain.rigidSpatialI[li], spatialGravity);
            }
        }

        for (int childLink = static_cast<int>(N) - 1; childLink >= 1; --childLink) {
            const std::size_t cIdx = static_cast<std::size_t>(childLink);
            const std::size_t pIdx = cIdx - 1u;
            const std::uint32_t jidx = chain.links[cIdx].jointIdx;
            if (jidx == ArtLink::kNoJoint || jidx >= servoJoints_.size()) {
                continue;
            }
            if (velocityPreCorr && articulationConfig_.enableVelocityPreCorrectionChassisArticulatedTree
                && N >= 4 && cIdx == 1u) {
                chain.legSubtreeIaAtCoxaAnchor = chain.Ia[1];
                chain.legSubtreePaAtCoxaAnchor = chain.Pa[1];
            }
            const ServoJoint& sj = servoJoints_[jidx];
            const Body&      parentBody = bodies_[sj.a];
            Vec3             axis = Rotate(parentBody.orientation, sj.localAxisA);
            if (!TryNormalize(axis, axis)) {
                continue;
            }
            float D = 0.0f;
            chain.iaPreJoint[cIdx] = chain.Ia[cIdx];
            if (velocityPreCorr) {
                chain.paJointSnapshot[cIdx] = chain.Pa[cIdx];
            }
            const SpatialInertia reduced = PropagateInertia(chain.Ia[cIdx], axis, D);
            chain.D[cIdx] = D;
            const ServoJointPrep& prepJoint = servoJointPreps_[jidx];
            const float          sPc    = velocityPreCorr ? Dot(axis, chain.Pa[cIdx].ang) : 0.0f;
            float                  uJoint = sPc;
            if (articulationConfig_.includeServoPdBiasInArticulationU && prepJoint.hingeActive) {
                uJoint += D * prepJoint.servoBias;
            }
            uJoint += sj.articulationDriveTorque;
            chain.u[cIdx] = uJoint;
            if (velocityPreCorr) {
                chain.Pa[pIdx] += PropagateBias(chain.Pa[cIdx], chain.Ia[cIdx], axis, D, uJoint);
            }
            const Vec3 off =
                chain.links[pIdx].jointAnchorWorld - chain.links[cIdx].jointAnchorWorld;
            chain.Ia[pIdx] += SpatialInertiaPluckerTranslateChildToParent(reduced, off);
        }

        const std::uint32_t leafBody = chain.links.back().bodyIdx;
        // Articulated contact mass: any detected chain with a leaf body (N>=2). If two chains
        // shared a leaf body, last writer wins (not used by built-in scenes).
        if (N >= 2 && leafBody < artChainIndexForLeafBody_.size()) {
            artChainIndexForLeafBody_[leafBody] = static_cast<std::uint32_t>(ci);
        }
    }

    // NOTE: ABI D[i] = s^T * Ioo_accumulated * s includes the parallel-axis term
    // (I_com + m*|r×s|²) and downstream chain coupling.  SolveServoJoint applies a
    // *pure angular impulse* (only angularVelocity is updated, not linearVelocity), so
    // the correct effective mass for that constraint is the COM-frame angular inertia —
    // exactly what PrepareServoJointSolves already computes as wHinge = invIA + invIB.
    // Overriding invDenomHinge with Dabi/denomScale would make the impulse ~Dabi/I_com
    // times too large (≈19× for a typical arm), causing velocity divergence.
    //
    // ABI data (D, Ia, Pa) is retained for Phase 1c (contact effective mass override)
    // and Phase 2 (velocity pre-correction forward pass) where spatial impulses ARE used.

    // Pass C — velocity pre-correction: either full spatial forward pass (optional) or legacy
    // child-only angular projection (no root writes so six legs sharing a chassis do not apply
    // six independent base deltas).
    const bool fullForwardPass = velocityPreCorr && !articulationConfig_.enableVelocityPreCorrectionKinematicsOnly
        && articulationConfig_.enableVelocityPreCorrectionFullForwardPass;
    const bool forwardCoriolis =
        fullForwardPass && articulationConfig_.enableVelocityPreCorrectionForwardCoriolis;
    if (fullForwardPass) {
        for (ArtChain& chain : articulationChains_) {
            const std::size_t N = chain.links.size();
            if (N < 2) {
                continue;
            }
            chain.spatialAcc.assign(N, SpatialVec{});
            // Floating base: zero spatial acceleration at root anchor (no extra gravity here;
            // gravity is already in IntegrateForces / Pa bias).
            chain.spatialAcc[0] = SpatialVec{};
            for (std::size_t linkIdx = 1; linkIdx < N; ++linkIdx) {
                const ArtLink& L = chain.links[linkIdx];
                if (L.jointIdx == ArtLink::kNoJoint || L.jointIdx >= servoJoints_.size()) {
                    continue;
                }
                const ServoJoint& sj = servoJoints_[L.jointIdx];
                const Body&       parentBody = bodies_[sj.a];
                Vec3              axis       = Rotate(parentBody.orientation, sj.localAxisA);
                if (!TryNormalize(axis, axis)) {
                    continue;
                }
                const float D = chain.D[linkIdx];
                if (!(D > kEpsilon)) {
                    continue;
                }
                const ArtLink& Lp    = chain.links[linkIdx - 1u];
                const Vec3     off   = L.jointAnchorWorld - Lp.jointAnchorWorld;
                const SpatialVec& ap = chain.spatialAcc[linkIdx - 1u];
                SpatialVec          aPred{ap.ang, ap.lin + Cross(ap.ang, off)};
                if (forwardCoriolis) {
                    const SpatialVec& vParent = chain.vel[linkIdx - 1u];
                    aPred.lin += Cross(vParent.ang, Cross(vParent.ang, off));
                }
                const SpatialVec  eta =
                    SpatialMul(chain.iaPreJoint[linkIdx], aPred) + chain.paJointSnapshot[linkIdx];
                const float qdd = (chain.u[linkIdx] - Dot(axis, eta.ang)) / D;
                chain.spatialAcc[linkIdx] =
                    SpatialVec{aPred.ang + axis * qdd, aPred.lin};
                Body& child = bodies_[sj.b];
                if (child.invMass > 0.0f && !child.isSleeping) {
                    const Vec3 rCom = child.position - L.jointAnchorWorld;
                    const Vec3 aLinCom =
                        chain.spatialAcc[linkIdx].lin + Cross(chain.spatialAcc[linkIdx].ang, rCom);
                    child.velocity += aLinCom * dt;
                    child.angularVelocity += chain.spatialAcc[linkIdx].ang * dt;
                }
            }
        }
    } else if (velocityPreCorr && !articulationConfig_.enableVelocityPreCorrectionKinematicsOnly) {
        for (ArtChain& chain : articulationChains_) {
            const std::size_t N = chain.links.size();
            if (N < 2) {
                continue;
            }
            for (std::size_t linkIdx = 1; linkIdx < N; ++linkIdx) {
                const ArtLink& L = chain.links[linkIdx];
                if (L.jointIdx == ArtLink::kNoJoint || L.jointIdx >= servoJoints_.size()) {
                    continue;
                }
                const ServoJoint& sj = servoJoints_[L.jointIdx];
                Body&             child = bodies_[sj.b];
                if (child.invMass <= 0.0f) {
                    continue;
                }
                const Body& parentBody = bodies_[sj.a];
                Vec3        axis = Rotate(parentBody.orientation, sj.localAxisA);
                if (!TryNormalize(axis, axis)) {
                    continue;
                }
                const float D = chain.D[linkIdx];
                if (!(D > kEpsilon)) {
                    continue;
                }
                const float sPa = Dot(axis, chain.paJointSnapshot[linkIdx].ang);
                const float qdd = (chain.u[linkIdx] - sPa) / D;
                child.angularVelocity += axis * (qdd * dt);
            }
        }
    }

    // Optional chassis coupling: MVP (sum of post-backward Pa[0] at COM) or multi-tree aggregate
    // inertia at COM + single 6×6 solve (tree path). Runs after child Pass C; default off.
    const bool chassisTree = velocityPreCorr && !articulationConfig_.enableVelocityPreCorrectionKinematicsOnly
        && articulationConfig_.enableVelocityPreCorrectionChassisArticulatedTree;
    const bool chassisMvp = velocityPreCorr && !articulationConfig_.enableVelocityPreCorrectionKinematicsOnly
        && articulationConfig_.enableVelocityPreCorrectionChassisCoupling && !chassisTree;
    if (chassisMvp || chassisTree) {
        constexpr float kChassisMaxDw = 10.0f;
        constexpr float kChassisMaxDv = 8.0f;
        const float     gain = std::clamp(articulationConfig_.chassisCouplingGain, 0.0f, 4.0f);

        chassisCouplingWrenchSum_.assign(nBodies, SpatialVec{});
        chassisCouplingChainCount_.assign(nBodies, 0u);
        if (chassisTree) {
            chassisArticulatedInertiaSum_.assign(nBodies, SpatialInertia{});
            chassisArticulatedBaseInertiaAdded_.assign(nBodies, 0u);
        }

        if (chassisMvp) {
            for (ArtChain& chain : articulationChains_) {
                const std::size_t N = chain.links.size();
                if (N < 2) {
                    continue;
                }
                const std::uint32_t rootBody = chain.links[0].bodyIdx;
                if (rootBody >= nBodies) {
                    continue;
                }
                Body& root = bodies_[rootBody];
                if (root.invMass <= 0.0f || root.isSleeping) {
                    continue;
                }
                const Vec3 com   = root.position;
                const Vec3 anchor = chain.links[0].jointAnchorWorld;
                const SpatialVec wCom =
                    SpatialForceTranslateByOffset(chain.Pa[0], com - anchor);
                chassisCouplingWrenchSum_[rootBody] += wCom;
                ++chassisCouplingChainCount_[rootBody];
            }

            for (std::size_t bi = 0; bi < nBodies; ++bi) {
                if (chassisCouplingChainCount_[bi] == 0u) {
                    continue;
                }
                Body&       b   = bodies_[bi];
                SpatialVec& sum = chassisCouplingWrenchSum_[bi];
                if (b.invMass <= 0.0f || b.isSleeping) {
                    continue;
                }
                Vec3 dw{};
                if (!articulationConfig_.chassisCouplingAngularOnly) {
                    const Vec3 dv = gain * dt * b.invMass * sum.lin;
                    const Vec3 dvClamped{
                        std::clamp(dv.x, -kChassisMaxDv, kChassisMaxDv),
                        std::clamp(dv.y, -kChassisMaxDv, kChassisMaxDv),
                        std::clamp(dv.z, -kChassisMaxDv, kChassisMaxDv),
                    };
                    b.velocity += dvClamped;
                }
                const Mat3 invI = b.InvInertiaWorld();
                dw                     = gain * dt * (invI * sum.ang);
                const Vec3 dwClamped = {
                    std::clamp(dw.x, -kChassisMaxDw, kChassisMaxDw),
                    std::clamp(dw.y, -kChassisMaxDw, kChassisMaxDw),
                    std::clamp(dw.z, -kChassisMaxDw, kChassisMaxDw),
                };
                b.angularVelocity += dwClamped;
            }
        } else if (chassisTree) {
            for (ArtChain& chain : articulationChains_) {
                const std::size_t N = chain.links.size();
                if (N < 4) {
                    continue;
                }
                const std::uint32_t rootBody = chain.links[0].bodyIdx;
                if (rootBody >= nBodies) {
                    continue;
                }
                Body& root = bodies_[rootBody];
                if (root.invMass <= 0.0f || root.isSleeping) {
                    continue;
                }
                const Vec3 com = root.position;
                if (chassisArticulatedBaseInertiaAdded_[rootBody] == 0u) {
                    const Mat3&          Iw       = bodyInertiaWorld_[rootBody];
                    const SpatialInertia IbaseCom =
                        SpatialInertiaFromBody(root.mass, Iw, com, com);
                    chassisArticulatedInertiaSum_[rootBody] += IbaseCom;
                    const SpatialVec vCom{root.angularVelocity, root.velocity};
                    chassisCouplingWrenchSum_[rootBody] +=
                        SpatialBias(IbaseCom, vCom) - SpatialMul(IbaseCom, spatialGravity);
                    chassisArticulatedBaseInertiaAdded_[rootBody] = 1u;
                }
                const Vec3 hip = chain.links[1].jointAnchorWorld;
                chassisArticulatedInertiaSum_[rootBody] += SpatialInertiaPluckerTranslateChildToParent(
                    chain.legSubtreeIaAtCoxaAnchor, com - hip);
                chassisCouplingWrenchSum_[rootBody] += SpatialForceTranslateByOffset(
                    chain.legSubtreePaAtCoxaAnchor, com - hip);
            }

            for (std::size_t bi = 0; bi < nBodies; ++bi) {
                if (chassisArticulatedBaseInertiaAdded_[bi] == 0u) {
                    continue;
                }
                Body& b = bodies_[bi];
                if (b.invMass <= 0.0f || b.isSleeping) {
                    continue;
                }
                const SpatialInertia& Iagg = chassisArticulatedInertiaSum_[bi];
                if (!(Iagg.mass > kEpsilon)) {
                    continue;
                }
                SpatialVec a{};
                const SpatialVec rhs{-chassisCouplingWrenchSum_[bi].ang, -chassisCouplingWrenchSum_[bi].lin};
                if (!SpatialInertiaSolveMotion(Iagg, rhs, a)) {
                    continue;
                }
                Vec3 dv = gain * dt * a.lin;
                Vec3 dw = gain * dt * a.ang;
                if (articulationConfig_.chassisCouplingAngularOnly) {
                    dv = Vec3{};
                }
                dv = {std::clamp(dv.x, -kChassisMaxDv, kChassisMaxDv),
                    std::clamp(dv.y, -kChassisMaxDv, kChassisMaxDv),
                    std::clamp(dv.z, -kChassisMaxDv, kChassisMaxDv)};
                dw = {std::clamp(dw.x, -kChassisMaxDw, kChassisMaxDw),
                    std::clamp(dw.y, -kChassisMaxDw, kChassisMaxDw),
                    std::clamp(dw.z, -kChassisMaxDw, kChassisMaxDw)};
                b.velocity += dv;
                b.angularVelocity += dw;
            }
        }
    }
}

void World::PrepareHingeJointSolves() {
        const std::size_t n = hingeJoints_.size();
        if (hingeJointPreps_.size() != n) {
            hingeJointPreps_.assign(n, HingeJointPrep{});
        }
        if (n == 0) {
            return;
        }

        const float anchorBiasFactor = jointSolverConfig_.hingeAnchorBiasFactor;
        const float blockDiagMin     = jointSolverConfig_.blockDiagonalMinimum;
        const float blockDetEps      = jointSolverConfig_.blockDeterminantEpsilon;
        const float blockCondMax     = jointSolverConfig_.blockConditionEstimateMax;
        const bool  useBlock         = jointSolverConfig_.useBlockSolver;

        for (std::size_t idx = 0; idx < n; ++idx) {
            const HingeJoint& j = hingeJoints_[idx];
            HingeJointPrep& prep = hingeJointPreps_[idx];
            prep = HingeJointPrep{};

            const Body& a = bodies_[j.a];
            const Body& b = bodies_[j.b];
            const Mat3& invIA = bodyInvInertiaWorld_[j.a];
            const Mat3& invIB = bodyInvInertiaWorld_[j.b];

            // ---- Anchor kinematics ----
            prep.ra = Rotate(a.orientation, j.localAnchorA);
            prep.rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 error = (b.position + prep.rb) - (a.position + prep.ra);
            const Vec3 va0   = a.velocity + Cross(a.angularVelocity, prep.ra);
            const Vec3 vb0   = b.velocity + Cross(b.angularVelocity, prep.rb);
            const Vec3 relVel0 = vb0 - va0;
            const float relAngSpeed = Length(b.angularVelocity - a.angularVelocity);

            // Wake check once per substep (not per iteration).
            if (Length(error) > kWakeContactPenetrationThreshold
                || Length(relVel0) > kWakeJointRelativeSpeedThreshold
                || relAngSpeed > kWakeJointRelativeSpeedThreshold
                || j.motorEnabled) {
                WakeConnectedBodies(j.a);
                WakeConnectedBodies(j.b);
            }

            prep.anchorBias = anchorBiasFactor * error;

            // ---- Anchor mass matrix ----
            const float invMassSum = a.invMass + b.invMass;
            const Vec3 axes[3] = {{1,0,0},{0,1,0},{0,0,1}};
            Vec3 raCrossAxis[3]{}, rbCrossAxis[3]{};
            for (int i = 0; i < 3; ++i) {
                raCrossAxis[i] = Cross(prep.ra, axes[i]);
                rbCrossAxis[i] = Cross(prep.rb, axes[i]);
            }

            if (useBlock) {
                float K[3][3]{};
                for (int i = 0; i < 3; ++i) {
                    const Vec3 invIaCrossI = invIA * raCrossAxis[i];
                    const Vec3 invIbCrossI = invIB * rbCrossAxis[i];
                    for (int kk = 0; kk < 3; ++kk) {
                        K[i][kk] = (i == kk ? invMassSum : 0.0f)
                            + Dot(invIaCrossI, raCrossAxis[kk])
                            + Dot(invIbCrossI, rbCrossAxis[kk]);
                    }
                }
                bool ok = true;
                for (int i = 0; i < 3; ++i) {
                    if (K[i][i] <= blockDiagMin) { ok = false; break; }
                }
                float det = 0.0f;
                if (ok) {
                    det = K[0][0] * (K[1][1]*K[2][2] - K[1][2]*K[2][1])
                        - K[0][1] * (K[1][0]*K[2][2] - K[1][2]*K[2][0])
                        + K[0][2] * (K[1][0]*K[2][1] - K[1][1]*K[2][0]);
                    if (std::abs(det) <= blockDetEps) ok = false;
                }
                float adj[3][3]{};
                if (ok) {
                    adj[0][0]=K[1][1]*K[2][2]-K[1][2]*K[2][1]; adj[0][1]=-(K[1][0]*K[2][2]-K[1][2]*K[2][0]); adj[0][2]=K[1][0]*K[2][1]-K[1][1]*K[2][0];
                    adj[1][0]=-(K[0][1]*K[2][2]-K[0][2]*K[2][1]); adj[1][1]=K[0][0]*K[2][2]-K[0][2]*K[2][0]; adj[1][2]=-(K[0][0]*K[2][1]-K[0][1]*K[2][0]);
                    adj[2][0]=K[0][1]*K[1][2]-K[0][2]*K[1][1]; adj[2][1]=-(K[0][0]*K[1][2]-K[0][2]*K[1][0]); adj[2][2]=K[0][0]*K[1][1]-K[0][1]*K[1][0];
                    if (blockCondMax > 0.0f) {
                        const float rowNorm = std::max({
                            std::abs(K[0][0])+std::abs(K[0][1])+std::abs(K[0][2]),
                            std::abs(K[1][0])+std::abs(K[1][1])+std::abs(K[1][2]),
                            std::abs(K[2][0])+std::abs(K[2][1])+std::abs(K[2][2]),
                        });
                        const float invAbsDet = 1.0f / std::abs(det);
                        const float invNorm = std::max({
                            (std::abs(adj[0][0])+std::abs(adj[0][1])+std::abs(adj[0][2]))*invAbsDet,
                            (std::abs(adj[1][0])+std::abs(adj[1][1])+std::abs(adj[1][2]))*invAbsDet,
                            (std::abs(adj[2][0])+std::abs(adj[2][1])+std::abs(adj[2][2]))*invAbsDet,
                        });
                        if (!std::isfinite(rowNorm*invNorm) || rowNorm*invNorm > blockCondMax) ok = false;
                    }
                }
                if (ok) {
                    const float invDet = 1.0f / det;
                    prep.invK = Mat3{{
                        {adj[0][0]*invDet, adj[1][0]*invDet, adj[2][0]*invDet},
                        {adj[0][1]*invDet, adj[1][1]*invDet, adj[2][1]*invDet},
                        {adj[0][2]*invDet, adj[1][2]*invDet, adj[2][2]*invDet},
                    }};
                    prep.anchorValid = true;
                }
            }

            // Per-axis scalar fallback (also used when !useBlock or !anchorValid).
            for (int i = 0; i < 3; ++i) {
                prep.fallbackEffMass[i] = invMassSum
                    + Dot(raCrossAxis[i], invIA * raCrossAxis[i])
                    + Dot(rbCrossAxis[i], invIB * rbCrossAxis[i]);
                prep.fallbackPosBias[i] = anchorBiasFactor * Dot(error, axes[i]);
            }

            // ---- Axis-alignment ----
            prep.axisA = Normalize(Rotate(a.orientation, j.localAxisA));
            const Vec3 axisB = Normalize(Rotate(b.orientation, j.localAxisB));
            prep.t1 = Cross(prep.axisA, {1.0f, 0.0f, 0.0f});
            if (LengthSquared(prep.t1) <= 1e-5f) prep.t1 = Cross(prep.axisA, {0.0f, 0.0f, 1.0f});
            prep.t1 = Normalize(prep.t1);
            prep.t2 = Cross(prep.axisA, prep.t1); // axisA ⊥ t1, both unit → cross is unit

            const Vec3 angularError = Cross(prep.axisA, axisB);
            const float effMassT1 = Dot(prep.t1, invIA * prep.t1) + Dot(prep.t1, invIB * prep.t1);
            if (effMassT1 > kEpsilon) {
                prep.invDenomT1 = 1.0f / effMassT1;
                prep.angBiasT1  = 0.2f * Dot(angularError, prep.t1);
                prep.t1Active   = true;
            }
            const float effMassT2 = Dot(prep.t2, invIA * prep.t2) + Dot(prep.t2, invIB * prep.t2);
            if (effMassT2 > kEpsilon) {
                prep.invDenomT2 = 1.0f / effMassT2;
                prep.angBiasT2  = 0.2f * Dot(angularError, prep.t2);
                prep.t2Active   = true;
            }

            // ---- Limit + motor effective mass ----
            const float effMassHinge = Dot(prep.axisA, invIA * prep.axisA)
                                     + Dot(prep.axisA, invIB * prep.axisA);
            if (effMassHinge > kEpsilon) {
                prep.invEffMassHinge    = 1.0f / effMassHinge;
                prep.hingeEffMassActive = true;
                // Precompute hinge angle once — avoids ResolveJointReference + FastAtan2 per
                // PGS iteration in the (N_iter - 1) redundant calls.
                const Vec3 refA = ResolveJointReference(a.orientation, j.localReferenceA, prep.axisA);
                const Vec3 refB = ResolveJointReference(b.orientation, j.localReferenceB, prep.axisA);
                prep.hingeAngle = SignedAngleAroundAxis(refA, refB, prep.axisA);
            }
        }
    }

void World::PrepareServoJointSolves() {
        const std::size_t n = servoJoints_.size();
        if (servoJointPreps_.size() != n) {
            servoJointPreps_.assign(n, ServoJointPrep{});
        }
        if (n == 0) {
            return;
        }

        const float dt = currentSubstepDt_;
        // Match the constants used in the original SolveServoJoint axis-alignment block.
        constexpr float kAxisAlignOmega = 260.0f;
        constexpr float kAxisAlignZeta  = 1.12f;
        const float axisDenom = std::max(2.0f * kAxisAlignZeta + dt * kAxisAlignOmega, kEpsilon);
        const float invAxisDenom = 1.0f / axisDenom;
        const float axisGammaCoeff = invAxisDenom / (dt * kAxisAlignOmega);
        const float axisBiasCoeff = kAxisAlignOmega * invAxisDenom;
        const float resumeScale = std::max(1.0f, jointSolverConfig_.servoEarlyOutResumeScale);
        const float impulseGuard = std::max(0.0f, jointSolverConfig_.servoEarlyOutImpulseGuardFraction);
        const float wakeErrorSq = kWakeContactPenetrationThreshold * kWakeContactPenetrationThreshold;
        const float wakeSpeedSq = kWakeJointRelativeSpeedThreshold * kWakeJointRelativeSpeedThreshold;
        const float anchorErrorEnter = jointSolverConfig_.servoAnchorEarlyOutError;
        const float anchorSpeedEnter = jointSolverConfig_.servoAnchorEarlyOutSpeed;
        const float anchorErrorEnterSq = anchorErrorEnter * anchorErrorEnter;
        const float anchorSpeedEnterSq = anchorSpeedEnter * anchorSpeedEnter;
        const float anchorErrorExitSq = (anchorErrorEnter * resumeScale) * (anchorErrorEnter * resumeScale);
        const float anchorSpeedExitSq = (anchorSpeedEnter * resumeScale) * (anchorSpeedEnter * resumeScale);
        const float angularErrorEnter = jointSolverConfig_.servoAngularEarlyOutError;
        const float angularSpeedEnter = jointSolverConfig_.servoAngularEarlyOutSpeed;
        const float angularErrorExit = angularErrorEnter * resumeScale;
        const float angularSpeedExit = angularSpeedEnter * resumeScale;
        const float hingeErrorEnter = jointSolverConfig_.servoHingeEarlyOutError;
        const float hingeSpeedEnter = jointSolverConfig_.servoHingeEarlyOutSpeed;
        const float hingeErrorExit = hingeErrorEnter * resumeScale;
        const float hingeSpeedExit = hingeSpeedEnter * resumeScale;
        const float anchorBiasFactor = jointSolverConfig_.hingeAnchorBiasFactor;
        const float blockDiagMin = jointSolverConfig_.blockDiagonalMinimum;
        const float blockDetEps = jointSolverConfig_.blockDeterminantEpsilon;
        const float blockCondMax = jointSolverConfig_.blockConditionEstimateMax;

        // Small flat cache: when multiple joints share the same (body_id, localAxisA)
        // — e.g. all 6 hip joints use body_id=0 and localAxisA={0,1,0} — the world-space
        // axis is computed once and reused for subsequent hits. Stack-allocated; cleared
        // implicitly each PrepareServoJointSolves call.
        struct AxisCacheEntry { std::uint32_t bodyId; Vec3 localAxis; Vec3 worldAxis; };
        constexpr int kAxisCacheCapacity = 8;
        AxisCacheEntry axisCache[kAxisCacheCapacity];
        int axisCacheCount = 0;

        for (std::size_t idx = 0; idx < n; ++idx) {
            ServoJoint& j = servoJoints_[idx];
            ServoJointPrep& prep = servoJointPreps_[idx];
            prep = ServoJointPrep{};

            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            const Mat3& invIA = bodyInvInertiaWorld_[j.a];
            const Mat3& invIB = bodyInvInertiaWorld_[j.b];

            // ---- Anchor kinematics ----
            prep.ra = Rotate(a.orientation, j.localAnchorA);
            prep.rb = Rotate(b.orientation, j.localAnchorB);
            const Vec3 error = (b.position + prep.rb) - (a.position + prep.ra);
            const Vec3 va0 = a.velocity + Cross(a.angularVelocity, prep.ra);
            const Vec3 vb0 = b.velocity + Cross(b.angularVelocity, prep.rb);
            const Vec3 relVel0 = vb0 - va0;
            const Vec3 relAngVel0 = b.angularVelocity - a.angularVelocity;

            // Wake check (start-of-substep kinematics).
            if (LengthSquared(error) > wakeErrorSq
                || LengthSquared(relVel0) > wakeSpeedSq
                || LengthSquared(relAngVel0) > wakeSpeedSq) {
                WakeConnectedBodies(j.a);
                WakeConnectedBodies(j.b);
            }

            const float anchorErrorSq = LengthSquared(error);
            const float anchorSpeedSq = LengthSquared(relVel0);
            const float anchorImpulseSum = std::abs(j.impulseX) + std::abs(j.impulseY) + std::abs(j.impulseZ);
            const bool anchorImpulseSmall = anchorImpulseSum <= (j.maxServoTorque * impulseGuard);
            if (jointSolverConfig_.enableServoAnchorEarlyOut) {
                if (j.anchorEarlyOutActive) {
                    j.anchorEarlyOutActive =
                        anchorErrorSq < anchorErrorExitSq && anchorSpeedSq < anchorSpeedExitSq && anchorImpulseSmall;
                } else {
                    j.anchorEarlyOutActive =
                        anchorErrorSq < anchorErrorEnterSq && anchorSpeedSq < anchorSpeedEnterSq && anchorImpulseSmall;
                }
                prep.skipAnchor = j.anchorEarlyOutActive;
            } else {
                j.anchorEarlyOutActive = false;
            }

            // ---- Anchor 3x3 mass matrix and inverse ----
            // Same construction and degeneracy/condition checks as SolveHingeAnchorBlock3x3.
            // We compute it even when skipAnchor is set so a later iteration that re-enters the
            // active path (unlikely within a single substep, but defensive) has a valid invK.
            // Skip when neither body has mass (degenerate trivially).
            float K[3][3]{};
            const float invMassSum = a.invMass + b.invMass;
            const Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
            Vec3 raCrossAxis[3]{};
            Vec3 rbCrossAxis[3]{};
            for (int i = 0; i < 3; ++i) {
                raCrossAxis[i] = Cross(prep.ra, axes[i]);
                rbCrossAxis[i] = Cross(prep.rb, axes[i]);
            }
            for (int i = 0; i < 3; ++i) {
                const Vec3 invIaCrossI = invIA * raCrossAxis[i];
                const Vec3 invIbCrossI = invIB * rbCrossAxis[i];
                for (int kCol = 0; kCol < 3; ++kCol) {
                    K[i][kCol] = (i == kCol ? invMassSum : 0.0f)
                        + Dot(invIaCrossI, raCrossAxis[kCol])
                        + Dot(invIbCrossI, rbCrossAxis[kCol]);
                }
            }

            bool anchorOK = true;
            for (int i = 0; i < 3; ++i) {
                if (K[i][i] <= blockDiagMin) { anchorOK = false; break; }
            }
            float det = 0.0f;
            if (anchorOK) {
                det = K[0][0] * (K[1][1] * K[2][2] - K[1][2] * K[2][1])
                    - K[0][1] * (K[1][0] * K[2][2] - K[1][2] * K[2][0])
                    + K[0][2] * (K[1][0] * K[2][1] - K[1][1] * K[2][0]);
                if (std::abs(det) <= blockDetEps) {
                    anchorOK = false;
                }
            }
            float adj[3][3]{};
            if (anchorOK) {
                adj[0][0] = K[1][1] * K[2][2] - K[1][2] * K[2][1];
                adj[0][1] = -(K[1][0] * K[2][2] - K[1][2] * K[2][0]);
                adj[0][2] = K[1][0] * K[2][1] - K[1][1] * K[2][0];
                adj[1][0] = -(K[0][1] * K[2][2] - K[0][2] * K[2][1]);
                adj[1][1] = K[0][0] * K[2][2] - K[0][2] * K[2][0];
                adj[1][2] = -(K[0][0] * K[2][1] - K[0][1] * K[2][0]);
                adj[2][0] = K[0][1] * K[1][2] - K[0][2] * K[1][1];
                adj[2][1] = -(K[0][0] * K[1][2] - K[0][2] * K[1][0]);
                adj[2][2] = K[0][0] * K[1][1] - K[0][1] * K[1][0];
                if (blockCondMax > 0.0f) {
                    const float rowNorm = std::max({
                        std::abs(K[0][0]) + std::abs(K[0][1]) + std::abs(K[0][2]),
                        std::abs(K[1][0]) + std::abs(K[1][1]) + std::abs(K[1][2]),
                        std::abs(K[2][0]) + std::abs(K[2][1]) + std::abs(K[2][2]),
                    });
                    const float invAbsDet = 1.0f / std::abs(det);
                    const float invNorm = std::max({
                        (std::abs(adj[0][0]) + std::abs(adj[0][1]) + std::abs(adj[0][2])) * invAbsDet,
                        (std::abs(adj[1][0]) + std::abs(adj[1][1]) + std::abs(adj[1][2])) * invAbsDet,
                        (std::abs(adj[2][0]) + std::abs(adj[2][1]) + std::abs(adj[2][2])) * invAbsDet,
                    });
                    const float conditionEstimate = rowNorm * invNorm;
                    if (!std::isfinite(conditionEstimate) || conditionEstimate > blockCondMax) {
                        anchorOK = false;
                    }
                }
            }
            if (anchorOK) {
                const float invDet = 1.0f / det;
                prep.invK = Mat3{{
                    {adj[0][0] * invDet, adj[1][0] * invDet, adj[2][0] * invDet},
                    {adj[0][1] * invDet, adj[1][1] * invDet, adj[2][1] * invDet},
                    {adj[0][2] * invDet, adj[1][2] * invDet, adj[2][2] * invDet},
                }};
                prep.anchorBias = anchorBiasFactor * error;
                prep.anchorValid = true;
            }

            // ---- Axis-alignment kinematics ----
            // Change 2: if masterAxisJointIdx is set, copy {axisA,t1,t2} from the master
            // joint's already-prepared prep (master must have a lower index → already done).
            // Exploits the geometric invariant that the femur and tibia joints of each leg
            // share the same world-space lateral axis (femur only rotates around that axis).
            if (j.masterAxisJointIdx != ServoJoint::kNoMasterAxis
                && j.masterAxisJointIdx < static_cast<std::uint32_t>(idx)) {
                const ServoJointPrep& master = servoJointPreps_[j.masterAxisJointIdx];
                prep.axisA = master.axisA;
                prep.t1    = master.t1;
                prep.t2    = master.t2;
            } else {
                // Change 1: probe the per-call flat cache keyed by (bodyA_id, localAxisA).
                // Avoids recomputing Normalize(Rotate(...)) for joints that share a body and
                // axis — e.g. all 6 hip joints use the same body orientation and {0,1,0}.
                Vec3 worldAxis{};
                bool foundInCache = false;
                for (int c = 0; c < axisCacheCount; ++c) {
                    const AxisCacheEntry& e = axisCache[c];
                    if (e.bodyId == j.a
                        && e.localAxis.x == j.localAxisA.x
                        && e.localAxis.y == j.localAxisA.y
                        && e.localAxis.z == j.localAxisA.z) {
                        worldAxis = e.worldAxis;
                        foundInCache = true;
                        break;
                    }
                }
                if (!foundInCache) {
                    worldAxis = Normalize(Rotate(a.orientation, j.localAxisA));
                    if (axisCacheCount < kAxisCacheCapacity) {
                        axisCache[axisCacheCount++] = {j.a, j.localAxisA, worldAxis};
                    }
                }
                prep.axisA = worldAxis;
                prep.t1 = Cross(prep.axisA, {1.0f, 0.0f, 0.0f});
                if (LengthSquared(prep.t1) <= 1e-5f) prep.t1 = Cross(prep.axisA, {0.0f, 0.0f, 1.0f});
                prep.t1 = Normalize(prep.t1);
                prep.t2 = Cross(prep.axisA, prep.t1); // axisA ⊥ t1, both unit → cross is unit
            }
            const Vec3 axisB = Normalize(Rotate(b.orientation, j.localAxisB));
            const Vec3 angularError = Cross(prep.axisA, axisB);
            const float angularErrorMag = Length(angularError);
            const float relAngSpeed = Length(relAngVel0);
            const bool angularImpulseSmall =
                std::max(std::abs(j.angularImpulse1), std::abs(j.angularImpulse2))
                <= (j.maxServoTorque * impulseGuard);
            if (jointSolverConfig_.enableServoAngularEarlyOut) {
                if (j.angularEarlyOutActive) {
                    j.angularEarlyOutActive =
                        angularErrorMag < angularErrorExit && relAngSpeed < angularSpeedExit && angularImpulseSmall;
                } else {
                    j.angularEarlyOutActive =
                        angularErrorMag < angularErrorEnter && relAngSpeed < angularSpeedEnter && angularImpulseSmall;
                }
                prep.skipAngular = j.angularEarlyOutActive;
            } else {
                j.angularEarlyOutActive = false;
            }

            // Effective masses and pre-inverted denominators for the two angular axis rows.
            const Vec3 invISumT1 = invIA * prep.t1 + invIB * prep.t1;
            const Vec3 invISumT2 = invIA * prep.t2 + invIB * prep.t2;
            const float wT1 = Dot(prep.t1, invISumT1);
            const float wT2 = Dot(prep.t2, invISumT2);
            const float wT12 = Dot(prep.t1, invISumT2); // == Dot(t2, invISumT1) since invIA, invIB symmetric
            prep.t1Active = wT1 > kEpsilon;
            prep.t2Active = wT2 > kEpsilon;
            if (prep.t1Active) {
                const float gamma1 = wT1 * axisGammaCoeff;
                prep.invDenomT1 = 1.0f / (wT1 + gamma1);
                prep.axisBiasT1 = Dot(angularError, prep.t1) * axisBiasCoeff;
            }
            if (prep.t2Active) {
                const float gamma2 = wT2 * axisGammaCoeff;
                prep.invDenomT2 = 1.0f / (wT2 + gamma2);
                prep.axisBiasT2 = Dot(angularError, prep.t2) * axisBiasCoeff;
            }
            // Try the coupled 2x2 block solve. Both rows share the same Baumgarte denominator
            // structure, so the diagonals are wTi + gamma_i and the off-diagonal is wT12 (no
            // gamma — gamma only damps each row's own constraint). Falls back to the two
            // scalar rows when the system is degenerate.
            if (prep.t1Active && prep.t2Active) {
                const float gamma1 = wT1 * axisGammaCoeff;
                const float gamma2 = wT2 * axisGammaCoeff;
                const float K11 = wT1 + gamma1;
                const float K22 = wT2 + gamma2;
                const float K12 = wT12;
                const float det2 = K11 * K22 - K12 * K12;
                // Require strict positivity and well-conditioning; use the per-row fallback
                // otherwise so we never inject NaN/Inf into the velocity stream.
                if (det2 > kEpsilon * std::max(K11 * K22, 1.0f)) {
                    const float invDet2 = 1.0f / det2;
                    prep.invK2aa = K22 * invDet2;
                    prep.invK2ab = -K12 * invDet2;
                    prep.invK2bb = K11 * invDet2;
                    prep.useBlockAxisSolve = true;
                }
            }

            // ---- Hinge servo kinematics ----
            const float wHinge = Dot(prep.axisA, invIA * prep.axisA) + Dot(prep.axisA, invIB * prep.axisA);
            prep.hingeActive = wHinge > kEpsilon;
            if (prep.hingeActive) {
                const float hingeAngle = core_internal::ComputeServoJointAngle(a, b, j);
                const float rawPositionError = core_internal::WrapJointAngle(hingeAngle - j.targetAngle);
                const float positionError = (j.positionErrorSmoothing > 0.0f) ? j.smoothedAngleError : rawPositionError;

                // Hinge early-out uses end-of-substep kinematic state (positionError) and
                // start-of-substep dynamics (impulse + angular velocity sample). The original
                // code re-evaluated this every iteration; freezing once per substep matches the
                // prepare/iterate split and avoids re-running the std::abs+compares N times.
                const float omegaAxis0 = Dot(relAngVel0, prep.axisA);
                const bool hingeImpulseSmall = std::abs(j.servoImpulseSum) <= (j.maxServoTorque * impulseGuard);
                if (jointSolverConfig_.enableServoHingeEarlyOut) {
                    if (j.hingeEarlyOutActive) {
                        j.hingeEarlyOutActive =
                            std::abs(positionError) < hingeErrorExit && std::abs(omegaAxis0) < hingeSpeedExit && hingeImpulseSmall;
                    } else {
                        j.hingeEarlyOutActive =
                            std::abs(positionError) < hingeErrorEnter && std::abs(omegaAxis0) < hingeSpeedEnter && hingeImpulseSmall;
                    }
                    prep.skipHinge = j.hingeEarlyOutActive;
                } else {
                    j.hingeEarlyOutActive = false;
                }

                const float omega = j.positionGain;
                const float zeta = j.dampingGain;
                const float hingeDenom = std::max(2.0f * zeta + dt * omega, kEpsilon);
                const float invHingeDenom = 1.0f / hingeDenom;
                const float hingeGamma = wHinge * invHingeDenom / (dt * omega);
                const float clampedError = std::clamp(positionError, -j.maxCorrectionAngle, j.maxCorrectionAngle);
                float servoBias = omega * clampedError * invHingeDenom + j.integralGain * j.integralAccum;
                prep.hasSpeedClamp = (j.maxServoSpeed > 0.0f && std::isfinite(j.maxServoSpeed));
                if (prep.hasSpeedClamp) {
                    servoBias = std::clamp(servoBias, -j.maxServoSpeed, j.maxServoSpeed);
                    prep.maxServoSpeed = j.maxServoSpeed;
                }
                prep.servoBias = servoBias;
                prep.invDenomHinge = 1.0f / (wHinge + hingeGamma);
                prep.invWHingeForSpeed = 1.0f / std::max(wHinge, kEpsilon);
            }
        }
    }

void World::SolveServoJoint(ServoJoint& j) {
#if MINPHYS3D_PROFILE_INNER_LOOPS
        const auto _scope = resource_profiler_.scope(
            world_resource_monitoring::toIndex(world_resource_monitoring::Section::SolveServoJoint));
#endif
        const std::size_t idx = static_cast<std::size_t>(&j - servoJoints_.data());
        const ServoJointPrep& prep = servoJointPreps_[idx];

        Body& a = bodies_[j.a];
        Body& b = bodies_[j.b];
        const Mat3& invIA = bodyInvInertiaWorld_[j.a];
        const Mat3& invIB = bodyInvInertiaWorld_[j.b];
        const float dampingFactor = jointSolverConfig_.hingeAnchorDampingFactor;

        // ---- Anchor 3D row ----
        if (!prep.skipAnchor && prep.anchorValid) {
            const Vec3 va = a.velocity + Cross(a.angularVelocity, prep.ra);
            const Vec3 vb = b.velocity + Cross(b.angularVelocity, prep.rb);
            const Vec3 relVel = vb - va;
            const Vec3 rhs = dampingFactor * relVel + prep.anchorBias;
            const Vec3 lambda = -1.0f * (prep.invK * rhs);
            if (std::isfinite(lambda.x) && std::isfinite(lambda.y) && std::isfinite(lambda.z)) {
                j.impulseX += lambda.x;
                j.impulseY += lambda.y;
                j.impulseZ += lambda.z;
                ApplyImpulse(a, b, invIA, invIB, prep.ra, prep.rb, lambda);
            }
        }

        // ---- Axis-alignment angular rows ----
        if (!prep.skipAngular) {
            const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
            if (prep.useBlockAxisSolve) {
                // Coupled 2x2 solve: lambda = invK2 * -(rhs).
                const float r1 = Dot(relAngVel, prep.t1) + prep.axisBiasT1;
                const float r2 = Dot(relAngVel, prep.t2) + prep.axisBiasT2;
                float lambda1 = -(prep.invK2aa * r1 + prep.invK2ab * r2);
                float lambda2 = -(prep.invK2ab * r1 + prep.invK2bb * r2);
                const float old1 = j.angularImpulse1;
                const float old2 = j.angularImpulse2;
                j.angularImpulse1 = std::clamp(old1 + lambda1, -j.maxServoTorque, j.maxServoTorque);
                j.angularImpulse2 = std::clamp(old2 + lambda2, -j.maxServoTorque, j.maxServoTorque);
                lambda1 = j.angularImpulse1 - old1;
                lambda2 = j.angularImpulse2 - old2;
                if (lambda1 != 0.0f || lambda2 != 0.0f) {
                    ApplyAngularImpulse(a, b, invIA, invIB, lambda1 * prep.t1 + lambda2 * prep.t2);
                }
            } else {
                if (prep.t1Active) {
                    const float velN = Dot(relAngVel, prep.t1);
                    float angLambda = -(velN + prep.axisBiasT1) * prep.invDenomT1;
                    const float oldAxisImpulse = j.angularImpulse1;
                    j.angularImpulse1 = std::clamp(oldAxisImpulse + angLambda, -j.maxServoTorque, j.maxServoTorque);
                    angLambda = j.angularImpulse1 - oldAxisImpulse;
                    ApplyAngularImpulse(a, b, invIA, invIB, angLambda * prep.t1);
                }
                if (prep.t2Active) {
                    const Vec3 relAngVel2 = b.angularVelocity - a.angularVelocity;
                    const float velN = Dot(relAngVel2, prep.t2);
                    float angLambda = -(velN + prep.axisBiasT2) * prep.invDenomT2;
                    const float oldAxisImpulse = j.angularImpulse2;
                    j.angularImpulse2 = std::clamp(oldAxisImpulse + angLambda, -j.maxServoTorque, j.maxServoTorque);
                    angLambda = j.angularImpulse2 - oldAxisImpulse;
                    ApplyAngularImpulse(a, b, invIA, invIB, angLambda * prep.t2);
                }
            }
        }

        // ---- Hinge servo row ----
        if (!prep.hingeActive || prep.skipHinge) {
            return;
        }
        {
            const Vec3 relAngVel = b.angularVelocity - a.angularVelocity;
            const float omegaAxis = Dot(relAngVel, prep.axisA);
            float servoLambda = -(omegaAxis + prep.servoBias) * prep.invDenomHinge;
            const float oldImpulse = j.servoImpulseSum;
            j.servoImpulseSum = std::clamp(j.servoImpulseSum + servoLambda, -j.maxServoTorque, j.maxServoTorque);
            servoLambda = j.servoImpulseSum - oldImpulse;
            ApplyAngularImpulse(a, b, invIA, invIB, servoLambda * prep.axisA);

            if (prep.hasSpeedClamp) {
                const float postOmegaAxis = Dot(b.angularVelocity - a.angularVelocity, prep.axisA);
                const float clampedOmegaAxis = std::clamp(postOmegaAxis, -prep.maxServoSpeed, prep.maxServoSpeed);
                if (std::abs(clampedOmegaAxis - postOmegaAxis) > 1e-6f) {
                    float speedLambda = (clampedOmegaAxis - postOmegaAxis) * prep.invWHingeForSpeed;
                    const float speedImpulse = std::clamp(j.servoImpulseSum + speedLambda, -j.maxServoTorque, j.maxServoTorque);
                    speedLambda = speedImpulse - j.servoImpulseSum;
                    j.servoImpulseSum = speedImpulse;
                    ApplyAngularImpulse(a, b, invIA, invIB, speedLambda * prep.axisA);
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
            &bodyInvInertiaWorld_,
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

bool World::ComputeServoPositionUseVelocityBiases() const {
        if (!servoPositionUseVelocityBiasesCacheValid_) {
            servoPositionUseVelocityBiasesCached_ = ComputeUseVelocityBiasesForServoPosition(
                servoJoints_, bodies_.size(), servoPositionFanoutScratch_);
            servoPositionUseVelocityBiasesCacheValid_ = true;
        }
        return servoPositionUseVelocityBiasesCached_;
    }

void World::SolveJointPositions(const std::vector<std::uint8_t>* skipServoHingeSnapMask) {
        const auto _scope = resource_profiler_.scope(
            world_resource_monitoring::toIndex(world_resource_monitoring::Section::SolveJointPositionsServo));
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
            true,
            ComputeServoPositionUseVelocityBiases(),
            &servoPositionFanoutScratch_,
            &servoPositionAngularAccumScratch_,
            &servoPositionAngularCountScratch_,
            skipServoHingeSnapMask,
        };
        jointSolver.SolveJointPositions(context);
    }

void World::SolveArticulationChainPositions() {
    const float subDt = currentSubstepDt_;
    for (const ArtChain& chain : articulationChains_) {
        const std::size_t N = chain.links.size();
        if (N < 2) {
            continue;
        }
        for (std::size_t linkIdx = 1; linkIdx < N; ++linkIdx) {
            const std::uint32_t jidx = chain.links[linkIdx].jointIdx;
            if (jidx == ArtLink::kNoJoint || jidx >= servoJoints_.size()) {
                continue;
            }
            ServoJoint& sj = servoJoints_[jidx];
            if (!sj.inArticulationChain) {
                continue;
            }
            if (jidx < servoSkipHingeSnapMask_.size()) {
                servoSkipHingeSnapMask_[jidx] = 1u;
            }
            Body& a = bodies_[sj.a];
            Body& b = bodies_[sj.b];
            if (a.invMass + b.invMass <= kEpsilon) {
                continue;
            }
            const float stab = std::clamp(sj.angleStabilizationScale, 0.0f, 1.0f);
            if (stab <= 1e-7f) {
                continue;
            }

            Vec3 axisA = Rotate(a.orientation, sj.localAxisA);
            Vec3 axisB = Rotate(b.orientation, sj.localAxisB);
            if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) {
                continue;
            }

            const Vec3 axisCorrection =
                core_internal::ComputeServoAxisAlignmentCorrection(sj, axisA, axisB);
            if (LengthSquared(axisCorrection) > 0.0f) {
                core_internal::ApplyAngularPositionCorrection(a, b, axisCorrection);
                axisA = Rotate(a.orientation, sj.localAxisA);
                axisB = Rotate(b.orientation, sj.localAxisB);
                if (!TryNormalize(axisA, axisA) || !TryNormalize(axisB, axisB)) {
                    continue;
                }
            }

            const float hingeAngle =
                core_internal::ComputeServoJointAngle(a, b, sj);
            const float targetError =
                core_internal::WrapJointAngle(hingeAngle - sj.targetAngle);
            const float maxServoSpeed = std::max(0.0f, sj.maxServoSpeed);
            const float maxAngleCorrection = maxServoSpeed > 0.0f
                ? std::min(0.06f * stab, maxServoSpeed * subDt)
                : 0.06f * stab;
            const float Dji = chain.D[linkIdx];
            const float dScale = std::min(1.0f, 0.25f / std::max(Dji, kEpsilon));
            const float raw = 0.15f * stab * targetError * dScale;
            const float clampedAngle = std::clamp(raw, -maxAngleCorrection, maxAngleCorrection);
            if (std::abs(clampedAngle) > 1e-5f) {
                core_internal::ApplyAngularPositionCorrection(a, b, clampedAngle * axisA);
            }
        }
    }
}

void World::SolveArticulationPositions() {
    const std::vector<std::uint8_t>* maskPtr = nullptr;
    if (articulationConfig_.enableChainPositionSolve) {
        servoSkipHingeSnapMask_.assign(servoJoints_.size(), 0u);
        SolveArticulationChainPositions();
        maskPtr = &servoSkipHingeSnapMask_;
    }
    SolveJointPositions(maskPtr);
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
            &bodyInvInertiaWorld_,
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
            &resource_profiler_,
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
