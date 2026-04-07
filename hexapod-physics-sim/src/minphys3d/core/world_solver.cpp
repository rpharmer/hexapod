#include "minphys3d/core/world.hpp"
#include "subsystems.hpp"

namespace minphys3d {
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
            if (m.tangentBasisValid && m.manifoldTangentImpulseValid && !m.contacts.empty()) {
                float totalNormal = 0.0f;
                for (const Contact& c : m.contacts) {
                    totalNormal += std::max(c.normalImpulseSum, 0.0f);
                }
                if (totalNormal > kEpsilon) {
                    for (Contact& c : m.contacts) {
                        const float w = std::max(c.normalImpulseSum, 0.0f) / totalNormal;
                        c.tangentImpulseSum0 = w * m.manifoldTangentImpulseSum[0];
                        c.tangentImpulseSum1 = w * m.manifoldTangentImpulseSum[1];
                        c.tangentImpulseSum = c.tangentImpulseSum0;
                    }
                }
            }
            for (Contact& c : m.contacts) {
                if (const std::array<float, 3>* cached = FindPerContactImpulseCache(m, c.key)) {
                    c.normalImpulseSum = std::max((*cached)[0], 0.0f);
                    c.tangentImpulseSum0 = (*cached)[1];
                    c.tangentImpulseSum1 = (*cached)[2];
                    c.tangentImpulseSum = c.tangentImpulseSum0;
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
        determinantOrConditionEstimate = det;
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
            determinantOrConditionEstimate = conditionEstimate;
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
        EnsurePerContactImpulseCache(manifold, c0.key)[0] = new0;
        EnsurePerContactImpulseCache(manifold, c1.key)[0] = new1;
#ifndef NDEBUG
        manifold.blockSolveDebug.selectedPostNormalImpulses = {new0, new1};
#endif
        const float delta0 = new0 - old0;
        const float delta1 = new1 - old1;
        ApplyImpulse(a, b, invIA, invIB, ra0, rb0, delta0 * normal0);
        ApplyImpulse(a, b, invIA, invIB, ra1, rb1, delta1 * normal1);
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

        std::array<Vec3, 4> normals{};
        std::array<Vec3, 4> ra{};
        std::array<Vec3, 4> rb{};
        std::array<Vec3, 4> raCrossN{};
        std::array<Vec3, 4> rbCrossN{};
        std::array<float, 4> diagonal{};
        std::array<float, 4> rhs{};
        std::array<float, 4> oldLambda{};
        std::array<std::array<float, 4>, 4> K{};

        const float invMassSum = a.invMass + b.invMass;
        Vec3 centroid{0.0f, 0.0f, 0.0f};
        for (const Contact& c : manifold.contacts) {
            centroid += c.point;
        }
        centroid = centroid / 4.0f;
        float spreadSq = 0.0f;
        float areaProxy = 0.0f;
        for (int i = 0; i < 4; ++i) {
            const Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            spreadSq = std::max(spreadSq, LengthSquared(c.point - centroid));
            for (int j = i + 1; j < 4; ++j) {
                areaProxy = std::max(areaProxy, LengthSquared(Cross(c.point - centroid, manifold.contacts[static_cast<std::size_t>(j)].point - centroid)));
            }
            normals[static_cast<std::size_t>(i)] = Normalize(c.normal);
            if (Dot(normals[static_cast<std::size_t>(i)], manifoldNormal) < 0.95f) {
                fallbackReason = BlockSolveFallbackReason::ContactNormalMismatch;
                return false;
            }
            ra[static_cast<std::size_t>(i)] = c.point - a.position;
            rb[static_cast<std::size_t>(i)] = c.point - b.position;
            raCrossN[static_cast<std::size_t>(i)] = Cross(ra[static_cast<std::size_t>(i)], normals[static_cast<std::size_t>(i)]);
            rbCrossN[static_cast<std::size_t>(i)] = Cross(rb[static_cast<std::size_t>(i)], normals[static_cast<std::size_t>(i)]);
            oldLambda[static_cast<std::size_t>(i)] = std::max(c.normalImpulseSum, 0.0f);
        }
        if (spreadSq < contactSolverConfig_.face4MinSpreadSq || areaProxy < contactSolverConfig_.face4MinArea) {
            fallbackReason = BlockSolveFallbackReason::QualityGate;
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                const float nd = Dot(normals[static_cast<std::size_t>(i)], normals[static_cast<std::size_t>(j)]);
                K[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    invMassSum * nd
                    + Dot(raCrossN[static_cast<std::size_t>(i)], invIA * raCrossN[static_cast<std::size_t>(j)])
                    + Dot(rbCrossN[static_cast<std::size_t>(i)], invIB * rbCrossN[static_cast<std::size_t>(j)]);
            }
            diagonal[static_cast<std::size_t>(i)] = K[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
            if (diagonal[static_cast<std::size_t>(i)] <= contactSolverConfig_.blockDiagonalMinimum) {
                fallbackReason = BlockSolveFallbackReason::DegenerateMassMatrix;
                return false;
            }
        }

        const float diagMin = *std::min_element(diagonal.begin(), diagonal.end());
        const float diagMax = *std::max_element(diagonal.begin(), diagonal.end());
        conditionEstimate = (diagMin > kEpsilon) ? (diagMax / diagMin) : std::numeric_limits<float>::infinity();
        const float conditionCap =
            (contactSolverConfig_.face4ConditionEstimateMax > 0.0f)
                ? contactSolverConfig_.face4ConditionEstimateMax
                : contactSolverConfig_.blockConditionEstimateMax;
        if (conditionCap > 0.0f && (!std::isfinite(conditionEstimate) || conditionEstimate > conditionCap)) {
            fallbackReason = BlockSolveFallbackReason::ConditionEstimateExceeded;
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            const Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            const Vec3 va = a.velocity + Cross(a.angularVelocity, ra[static_cast<std::size_t>(i)]);
            const Vec3 vb = b.velocity + Cross(b.angularVelocity, rb[static_cast<std::size_t>(i)]);
            const float vn = Dot(vb - va, normals[static_cast<std::size_t>(i)]);
            const float speedIntoContact = -vn;
            const float restitution = ComputeRestitution(speedIntoContact, a.restitution, b.restitution);
            float biasTerm = 0.0f;
            const float penetrationError = std::max(c.penetration - contactSolverConfig_.penetrationSlop, 0.0f);
            if (currentSubstepDt_ > kEpsilon && !contactSolverConfig_.useSplitImpulse) {
                const float maxSafeSeparatingSpeed = penetrationError / currentSubstepDt_;
                if (vn <= maxSafeSeparatingSpeed) {
                    biasTerm = (contactSolverConfig_.penetrationBiasFactor * penetrationError) / currentSubstepDt_;
                }
            }
            const float localRhs = -(1.0f + restitution) * vn + std::max(biasTerm, 0.0f);
            float shifted = localRhs;
            for (int j = 0; j < 4; ++j) {
                shifted += K[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * oldLambda[static_cast<std::size_t>(j)];
            }
            rhs[static_cast<std::size_t>(i)] = shifted;
        }

        std::array<float, 4> lambda = oldLambda;
        const int iterations = std::max<int>(contactSolverConfig_.face4Iterations, 1);
        for (int iter = 0; iter < iterations; ++iter) {
            float maxDelta = 0.0f;
            for (int i = 0; i < 4; ++i) {
                float sum = 0.0f;
                for (int j = 0; j < 4; ++j) {
                    if (i == j) continue;
                    sum += K[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * lambda[static_cast<std::size_t>(j)];
                }
                const float newValue = std::max(0.0f, (rhs[static_cast<std::size_t>(i)] - sum) / diagonal[static_cast<std::size_t>(i)]);
                maxDelta = std::max(maxDelta, std::abs(newValue - lambda[static_cast<std::size_t>(i)]));
                lambda[static_cast<std::size_t>(i)] = newValue;
            }
            if (maxDelta <= contactSolverConfig_.face4ProjectedGaussSeidelEpsilon) {
                break;
            }
        }

        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(lambda[static_cast<std::size_t>(i)])) {
                fallbackReason = BlockSolveFallbackReason::NonFiniteResult;
                return false;
            }
            Contact& c = manifold.contacts[static_cast<std::size_t>(i)];
            const float delta = lambda[static_cast<std::size_t>(i)] - oldLambda[static_cast<std::size_t>(i)];
            c.normalImpulseSum = lambda[static_cast<std::size_t>(i)];
            EnsurePerContactImpulseCache(manifold, c.key)[0] = c.normalImpulseSum;
            ApplyImpulse(a, b, invIA, invIB, ra[static_cast<std::size_t>(i)], rb[static_cast<std::size_t>(i)], delta * normals[static_cast<std::size_t>(i)]);
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


void World::SolveContactsInManifold(Manifold& manifold) {
        core_internal::ContactSolver solver;
        core_internal::ContactSolverContext context{
            bodies_,
            contacts_,
            manifolds_,
            previousManifolds_,
            [this](const PersistentPointKey& key, float& normal, std::array<float, 2>& tangent, std::uint16_t& age) {
                const auto it = persistentPointImpulses_.find(key);
                if (it == persistentPointImpulses_.end()) {
                    return false;
                }
                normal = it->second.normalImpulseSum;
                tangent = {it->second.tangentImpulseSum0, it->second.tangentImpulseSum1};
                age = it->second.persistenceAge;
                return true;
            },
            [](std::vector<Contact>& manifoldContacts) { SortManifoldContacts(manifoldContacts); },
            [](Manifold& manifold) { RefreshManifoldBlockCache(manifold); },
            [this](Manifold& manifold) { SelectBlockSolvePair(manifold); },
#ifndef NDEBUG
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
#ifndef NDEBUG
                if (reusedBasis) {
                    ++solverTelemetry_.tangentBasisReused;
                } else {
                    ++solverTelemetry_.tangentBasisResets;
                }
#else
                (void)reusedBasis;
#endif
            },
#ifndef NDEBUG
            [this]() { ++solverTelemetry_.manifoldFrictionBudgetSaturated; },
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
        core_internal::JointSolverContext context{bodies_, joints_, hingeJoints_};
        jointSolver.SolveJointPositions(context);
    }

void World::SolveIslands() {
        core_internal::ContactSolver solver;
        core_internal::ContactSolverContext contactContext{
            bodies_,
            contacts_,
            manifolds_,
            previousManifolds_,
            [this](const PersistentPointKey& key, float& normal, std::array<float, 2>& tangent, std::uint16_t& age) {
                const auto it = persistentPointImpulses_.find(key);
                if (it == persistentPointImpulses_.end()) {
                    return false;
                }
                normal = it->second.normalImpulseSum;
                tangent = {it->second.tangentImpulseSum0, it->second.tangentImpulseSum1};
                age = it->second.persistenceAge;
                return true;
            },
            [](std::vector<Contact>& manifoldContacts) { SortManifoldContacts(manifoldContacts); },
            [](Manifold& manifold) { RefreshManifoldBlockCache(manifold); },
            [this](Manifold& manifold) { SelectBlockSolvePair(manifold); },
#ifndef NDEBUG
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
#ifndef NDEBUG
                if (reusedBasis) {
                    ++solverTelemetry_.tangentBasisReused;
                } else {
                    ++solverTelemetry_.tangentBasisResets;
                }
#else
                (void)reusedBasis;
#endif
            },
#ifndef NDEBUG
            [this]() { ++solverTelemetry_.manifoldFrictionBudgetSaturated; },
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

        for (const Island& island : islands_) {
            for (std::size_t mi : island.manifolds) {
                solver.SolveContactsInManifold(contactContext, manifolds_[mi]);
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
        const core_internal::SleepSystem sleepSystem;
        const core_internal::SleepSystemContext context{
            bodies_,
            manifolds_,
            joints_,
            hingeJoints_,
            kSleepLinearThreshold,
            kSleepAngularThreshold,
            kSleepFramesThreshold,
        };
        sleepSystem.UpdateSleeping(context);
    }


} // namespace minphys3d
