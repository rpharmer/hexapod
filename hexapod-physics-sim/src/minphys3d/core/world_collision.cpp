#include "minphys3d/core/world.hpp"
#include "subsystems.hpp"

namespace minphys3d {
void World::BuildManifolds() {
        core_internal::ContactSolver solver;
        core_internal::ContactSolverContext context{
            bodies_,
            contacts_,
            manifolds_,
            previousManifolds_,
            [this](const PersistentPointKey& key, float& normal, float& tangent, std::uint16_t& age) {
                const auto it = persistentPointImpulses_.find(key);
                if (it == persistentPointImpulses_.end()) {
                    return false;
                }
                normal = it->second.normalImpulseSum;
                tangent = it->second.tangentImpulseSum;
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
            contactSolverConfig_,
        };
        solver.BuildManifolds(context);
    }

void World::GenerateContacts() {
        const core_internal::BroadphaseSystem broadphaseSystem;
        const std::vector<Pair> pairs = broadphaseSystem.ComputePotentialPairs({[this]() { return ComputePotentialPairs(); }});

        const core_internal::NarrowphaseSystem narrowphaseSystem;
        const core_internal::NarrowphaseContext context{
            bodies_,
            pairs,
            [this](std::uint32_t a, std::uint32_t b) { SphereSphere(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereCapsule(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleCapsule(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsulePlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SpherePlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { BoxPlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { BoxBox(a, b); },
        };
        narrowphaseSystem.GenerateContacts(context);
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


} // namespace minphys3d
