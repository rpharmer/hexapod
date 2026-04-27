#include "minphys3d/collision/convex_support.hpp"
#include "minphys3d/core/broadphase_system.hpp"
#include "minphys3d/core/contact_pipeline.hpp"
#include "minphys3d/core/subsystems.hpp"
#include "minphys3d/core/world.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace minphys3d {
namespace {

ConvexSupport BuildConvexSupport(const Body& body) {
    if (body.shape == ShapeType::Sphere) {
        return ConvexSupport{
            body.position,
            body.orientation,
            [center = body.position, radius = body.radius](const Vec3& direction) {
                Vec3 dir = direction;
                if (LengthSquared(dir) <= kEpsilon * kEpsilon) {
                    dir = {1.0f, 0.0f, 0.0f};
                }
                dir = Normalize(dir);
                return ConvexSupportPoint{center + dir * radius, ShapeTopology{}};
            }};
    }

    if (body.shape == ShapeType::Box) {
        return ConvexSupport{
            body.position,
            body.orientation,
            [center = body.position, orientation = body.orientation, ext = body.halfExtents](const Vec3& direction) {
                const Vec3 localDir = Rotate(Conjugate(orientation), direction);
                Vec3 local{
                    localDir.x >= 0.0f ? ext.x : -ext.x,
                    localDir.y >= 0.0f ? ext.y : -ext.y,
                    localDir.z >= 0.0f ? ext.z : -ext.z,
                };
                const std::uint8_t vertexId = static_cast<std::uint8_t>((local.x > 0.0f ? 1u : 0u) | (local.y > 0.0f ? 2u : 0u) | (local.z > 0.0f ? 4u : 0u));
                return ConvexSupportPoint{
                    center + Rotate(orientation, local),
                    ShapeTopology{vertexId, std::nullopt, std::nullopt},
                };
            }};
    }

    if (body.shape == ShapeType::Capsule) {
        return ConvexSupport{
            body.position,
            body.orientation,
            [center = body.position, orientation = body.orientation, halfHeight = body.halfHeight, radius = body.radius](const Vec3& direction) {
                const Vec3 axis = Normalize(Rotate(orientation, {0.0f, 1.0f, 0.0f}));
                Vec3 dir = direction;
                if (LengthSquared(dir) <= kEpsilon * kEpsilon) {
                    dir = axis;
                }
                dir = Normalize(dir);
                const float side = Dot(dir, axis);
                const bool upper = side >= 0.0f;
                const Vec3 segmentPoint = center + axis * (upper ? halfHeight : -halfHeight);
                return ConvexSupportPoint{
                    segmentPoint + dir * radius,
                    ShapeTopology{std::nullopt, std::nullopt, std::optional<std::uint16_t>(upper ? 1u : 0u)},
                };
            }};
    }

    if (body.shape == ShapeType::Cylinder) {
        return ConvexSupport{
            body.position,
            body.orientation,
            [center = body.position, orientation = body.orientation, halfHeight = body.halfHeight, radius = body.radius](const Vec3& direction) {
                Vec3 localDir = Rotate(Conjugate(orientation), direction);
                const float radialLengthSq = localDir.x * localDir.x + localDir.z * localDir.z;
                Vec3 localPoint{0.0f, localDir.y >= 0.0f ? halfHeight : -halfHeight, 0.0f};
                if (radialLengthSq > kEpsilon * kEpsilon) {
                    const float invRadialLength = 1.0f / std::sqrt(radialLengthSq);
                    localPoint.x = localDir.x * invRadialLength * radius;
                    localPoint.z = localDir.z * invRadialLength * radius;
                }
                return ConvexSupportPoint{
                    center + Rotate(orientation, localPoint),
                    ShapeTopology{},
                };
            }};
    }

    if (body.shape == ShapeType::HalfCylinder) {
        const Vec3 shapeOrigin = BodyWorldShapeOrigin(body);
        return ConvexSupport{
            shapeOrigin,
            body.orientation,
            [center = shapeOrigin, orientation = body.orientation, halfHeight = body.halfHeight, radius = body.radius](const Vec3& direction) {
                const Vec3 localDir = Rotate(Conjugate(orientation), direction);
                const float hx = radius;
                const float hy = halfHeight;

                Vec3 best{};
                float bestProj = -std::numeric_limits<float>::infinity();
                const auto consider = [&](const Vec3& p) {
                    const float proj = localDir.x * p.x + localDir.y * p.y + localDir.z * p.z;
                    if (proj > bestProj) {
                        bestProj = proj;
                        best = p;
                    }
                };

                // Flat face z = 0, |x| <= hx, |y| <= hy
                {
                    const float x = (localDir.x > kEpsilon) ? hx : ((localDir.x < -kEpsilon) ? -hx : 0.0f);
                    const float y = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0f);
                    consider({x, y, 0.0f});
                }

                // Cylindrical side / cap rim: (x,z) on circle, y = ±hy, only z >= 0
                const float radialLengthSq = localDir.x * localDir.x + localDir.z * localDir.z;
                if (radialLengthSq > kEpsilon * kEpsilon) {
                    const float invRadialLength = 1.0f / std::sqrt(radialLengthSq);
                    const float cx = localDir.x * invRadialLength * hx;
                    const float cz = localDir.z * invRadialLength * hx;
                    if (cz >= 0.0f) {
                        const float cy = localDir.y >= 0.0f ? hy : -hy;
                        consider({cx, cy, cz});
                    }
                } else if (localDir.z > kEpsilon) {
                    // Ridge x = 0, z = hx (pure +Z or nearly axial); pick y to maximize d·p along the segment |y| <= hy
                    const float ry = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0f);
                    consider({0.0f, ry, hx});
                }

                // Top / bottom semicircular caps (y = ±hy, x^2 + z^2 <= hx^2, z >= 0)
                const auto considerSemicap = [&](float capY) {
                    const float dx = localDir.x;
                    const float dz = localDir.z;
                    const float radialXZSq = dx * dx + dz * dz;
                    float sx = 0.0f;
                    float sz = 0.0f;
                    if (radialXZSq > kEpsilon * kEpsilon) {
                        const float inv = 1.0f / std::sqrt(radialXZSq);
                        sx = dx * inv * hx;
                        sz = dz * inv * hx;
                        if (sz < 0.0f) {
                            sx = (dx > kEpsilon) ? hx : ((dx < -kEpsilon) ? -hx : 0.0f);
                            sz = 0.0f;
                        }
                    }
                    consider({sx, capY, sz});
                };
                considerSemicap(hy);
                considerSemicap(-hy);

                return ConvexSupportPoint{
                    center + Rotate(orientation, best),
                    ShapeTopology{},
                };
            }};
    }

    return {};
}

struct ResolvedCollisionShape {
    Body body{};
    AABB bounds{};
    std::uint32_t encodedFeature = 0;
};

std::uint32_t EncodeShapeFeature(bool fromCompound, std::uint32_t childIndex, ShapeType shape) {
    const std::uint32_t compactShape = static_cast<std::uint32_t>(shape) & 0xffu;
    const std::uint32_t childBits = fromCompound ? std::min(childIndex + 1u, 0x00ffffffu) : 0u;
    return (childBits << 8u) | compactShape;
}

Body MaterializeCompoundChild(const Body& parent, const CompoundChild& child) {
    Body resolved = parent;
    resolved.shape = child.shape;
    resolved.position = parent.position + Rotate(parent.orientation, child.localPosition);
    resolved.orientation = Normalize(parent.orientation * child.localOrientation);
    resolved.radius = child.radius;
    resolved.halfHeight = child.halfHeight;
    resolved.halfExtents = child.halfExtents;
    resolved.compoundChildren.clear();
    return resolved;
}

std::vector<ResolvedCollisionShape> ResolveCollisionShapes(const Body& body) {
    std::vector<ResolvedCollisionShape> resolved{};
    if (!IsCompoundShape(body.shape)) {
        resolved.push_back({body, body.ComputeAABB(), EncodeShapeFeature(false, 0u, body.shape)});
        return resolved;
    }

    for (std::size_t childIndex = 0; childIndex < body.compoundChildren.size(); ++childIndex) {
        const CompoundChild& child = body.compoundChildren[childIndex];
        if (!IsCompoundChildShapeSupported(child.shape)) {
            continue;
        }
        Body childBody = MaterializeCompoundChild(body, child);
        resolved.push_back({
            childBody,
            childBody.ComputeAABB(),
            EncodeShapeFeature(true, static_cast<std::uint32_t>(childIndex), child.shape),
        });
    }

    if (resolved.empty()) {
        Body fallback = body;
        fallback.shape = ShapeType::Box;
        fallback.compoundChildren.clear();
        resolved.push_back({fallback, fallback.ComputeAABB(), EncodeShapeFeature(true, 0u, fallback.shape)});
    }
    return resolved;
}

bool ComputeConvexPenetration(const Body& a, const Body& b, EpaPenetrationResult& outPenetration) {
    const ConvexSupport supportA = BuildConvexSupport(a);
    const ConvexSupport supportB = BuildConvexSupport(b);
    if (!supportA.IsValid() || !supportB.IsValid()) {
        return false;
    }
    const GjkDistanceResult gjk = GjkDistance(supportA, supportB);
    if (!gjk.intersecting) {
        return false;
    }
    if (gjk.simplex.size >= 3) {
        outPenetration = ComputePenetrationEPA(supportA, supportB, gjk.simplex);
        if (outPenetration.valid && outPenetration.depth > 0.0f) {
            return true;
        }
    }

    Vec3 normal = b.position - a.position;
    if (LengthSquared(normal) <= kEpsilon * kEpsilon) {
        normal = {1.0f, 0.0f, 0.0f};
    } else {
        normal = Normalize(normal);
    }

    const ConvexSupportPoint witnessA = supportA.Support(normal);
    const ConvexSupportPoint witnessB = supportB.Support(-normal);
    const float depth = Dot(witnessA.point - witnessB.point, normal);
    if (depth <= 0.0f) {
        return false;
    }

    outPenetration = {};
    outPenetration.valid = true;
    outPenetration.depth = depth;
    outPenetration.normal = normal;
    outPenetration.witnessA = witnessA.point;
    outPenetration.witnessB = witnessB.point;
    return true;
}

const std::vector<float>& TerrainCollisionHeights(const World::TerrainHeightfieldAttachment& terrain) {
    return terrain.collisionHeightsM.empty() ? terrain.surfaceHeightsM : terrain.collisionHeightsM;
}

bool SampleTerrainCell(const World::TerrainHeightfieldAttachment& terrain,
                       int row,
                       int col,
                       float x,
                       float z,
                       float& outHeight,
                       Vec3& outNormal) {
    if (!terrain.enabled || terrain.rows < 2 || terrain.cols < 2) {
        return false;
    }
    if (row < 0 || col < 0 || row >= terrain.rows - 1 || col >= terrain.cols - 1) {
        return false;
    }
    const std::vector<float>& heights = TerrainCollisionHeights(terrain);
    const std::size_t idx00 = static_cast<std::size_t>(row * terrain.cols + col);
    const std::size_t idx10 = idx00 + 1u;
    const std::size_t idx01 = static_cast<std::size_t>((row + 1) * terrain.cols + col);
    const std::size_t idx11 = idx01 + 1u;
    if (idx11 >= heights.size()) {
        return false;
    }

    const float cellX0 = terrain.gridOriginWorld.x + static_cast<float>(col) * terrain.cellSizeM;
    const float cellZ0 = terrain.gridOriginWorld.z + static_cast<float>(row) * terrain.cellSizeM;
    const float invCell = 1.0f / std::max(terrain.cellSizeM, 1.0e-6f);
    const float tx = std::clamp((x - cellX0) * invCell, 0.0f, 1.0f);
    const float tz = std::clamp((z - cellZ0) * invCell, 0.0f, 1.0f);

    const float h00 = heights[idx00];
    const float h10 = heights[idx10];
    const float h01 = heights[idx01];
    const float h11 = heights[idx11];
    const float hx = h10 - h00;
    const float hz = h01 - h00;
    const float hxz = h00 - h10 - h01 + h11;

    outHeight = h00 + hx * tx + hz * tz + hxz * tx * tz;
    const float dhdx = (hx + hxz * tz) * invCell;
    const float dhdz = (hz + hxz * tx) * invCell;
    outNormal = { -dhdx, 1.0f, -dhdz };
    if (!TryNormalize(outNormal, outNormal)) {
        outNormal = {0.0f, 1.0f, 0.0f};
    }
    return true;
}

bool TerrainCellRangeForAabb(const World::TerrainHeightfieldAttachment& terrain,
                             const AABB& bounds,
                             int& row0,
                             int& row1,
                             int& col0,
                             int& col1) {
    if (!terrain.enabled || terrain.rows < 2 || terrain.cols < 2) {
        return false;
    }

    const float cellSize = std::max(terrain.cellSizeM, 1.0e-6f);
    const float minX = terrain.gridOriginWorld.x;
    const float minZ = terrain.gridOriginWorld.z;
    const float maxX = minX + static_cast<float>(terrain.cols - 1) * cellSize;
    const float maxZ = minZ + static_cast<float>(terrain.rows - 1) * cellSize;
    if (bounds.max.x < minX || bounds.min.x > maxX || bounds.max.z < minZ || bounds.min.z > maxZ) {
        return false;
    }

    col0 = std::clamp(static_cast<int>(std::floor((bounds.min.x - minX) / cellSize)), 0, terrain.cols - 2);
    col1 = std::clamp(static_cast<int>(std::floor((bounds.max.x - minX) / cellSize)), 0, terrain.cols - 2);
    row0 = std::clamp(static_cast<int>(std::floor((bounds.min.z - minZ) / cellSize)), 0, terrain.rows - 2);
    row1 = std::clamp(static_cast<int>(std::floor((bounds.max.z - minZ) / cellSize)), 0, terrain.rows - 2);
    return row0 <= row1 && col0 <= col1;
}

struct TerrainContactCandidate {
    std::uint32_t row = 0;
    std::uint32_t col = 0;
    std::uint32_t shapeFeature = 0;
    Vec3 normal{};
    Vec3 supportPoint{};
    float penetration = 0.0f;
    float height = 0.0f;
};

} // namespace

void World::ConvexPlane(std::uint32_t convexBodyId, std::uint32_t planeId) {
    const Body& convex = bodies_[convexBodyId];
    const Body& plane = bodies_[planeId];
    Vec3 n{};
    if (!TryGetPlaneNormal(plane, n)) {
        return;
    }
    const ConvexSupport support = BuildConvexSupport(convex);
    if (!support.IsValid()) {
        return;
    }
    const ConvexSupportPoint sp = support.Support(-n);
    const float signedDistance = Dot(n, sp.point) - plane.planeOffset;
    if (signedDistance >= 0.0f) {
        return;
    }
    const std::uint64_t featureId = CanonicalFeaturePairId(convexBodyId, planeId, 0u, 0u);
    AddContact(convexBodyId, planeId, -n, sp.point, -signedDistance, 11u, featureId);
}

void World::ConvexConvexEPA(std::uint32_t bodyAId, std::uint32_t bodyBId) {
    const Body& ba = bodies_[bodyAId];
    const Body& bb = bodies_[bodyBId];
    if (!IsConvexPrimitiveShape(ba.shape) || !IsConvexPrimitiveShape(bb.shape)) {
        return;
    }
    EpaPenetrationResult penetration{};
    if (!ComputeConvexPenetration(ba, bb, penetration)) {
        return;
    }
    Vec3 normal = penetration.normal;
    if (Dot(normal, bb.position - ba.position) < 0.0f) {
        normal = -normal;
    }
    const std::uint16_t detail = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(ba.shape) << 8u) | static_cast<std::uint16_t>(bb.shape));
    const std::uint64_t featureId = CanonicalFeaturePairId(bodyAId, bodyBId, 0u, 0u, detail);
    AddContact(
        bodyAId,
        bodyBId,
        normal,
        0.5f * (penetration.witnessA + penetration.witnessB),
        penetration.depth,
        12u,
        featureId);
}

std::uint64_t World::ComputeShapeGeometrySignature(const Body& body) {
    std::uint64_t seed = static_cast<std::uint64_t>(body.shape);
    auto mixFloat = [&seed](float value) {
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value), "float size mismatch");
        std::memcpy(&bits, &value, sizeof(bits));
        seed ^= static_cast<std::uint64_t>(bits) + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u);
    };
    auto mixU64 = [&seed](std::uint64_t value) {
        seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u);
    };
    mixFloat(body.radius);
    mixFloat(body.halfHeight);
    mixFloat(body.halfExtents.x);
    mixFloat(body.halfExtents.y);
    mixFloat(body.halfExtents.z);
    mixFloat(body.planeNormal.x);
    mixFloat(body.planeNormal.y);
    mixFloat(body.planeNormal.z);
    mixFloat(body.planeOffset);
    mixU64(body.compoundChildren.size());
    for (const CompoundChild& child : body.compoundChildren) {
        mixU64(static_cast<std::uint64_t>(child.shape));
        mixFloat(child.localPosition.x);
        mixFloat(child.localPosition.y);
        mixFloat(child.localPosition.z);
        mixFloat(child.localOrientation.w);
        mixFloat(child.localOrientation.x);
        mixFloat(child.localOrientation.y);
        mixFloat(child.localOrientation.z);
        mixFloat(child.radius);
        mixFloat(child.halfHeight);
        mixFloat(child.halfExtents.x);
        mixFloat(child.halfExtents.y);
        mixFloat(child.halfExtents.z);
    }
    return seed;
}

void World::RefreshShapeRevisionCounters() {
    if (shapeRevisionCounters_.size() != bodies_.size()) {
        shapeRevisionCounters_.assign(bodies_.size(), 0);
        shapeGeometrySignatures_.assign(bodies_.size(), 0);
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const std::uint64_t signature = ComputeShapeGeometrySignature(bodies_[i]);
        if (shapeGeometrySignatures_[i] == 0) {
            shapeGeometrySignatures_[i] = signature;
            continue;
        }
        if (shapeGeometrySignatures_[i] != signature) {
            shapeGeometrySignatures_[i] = signature;
            ++shapeRevisionCounters_[i];
        }
    }
}

bool World::ConvexOverlapWithCache(std::uint32_t a, std::uint32_t b) {
    const std::uint32_t lo = std::min(a, b);
    const std::uint32_t hi = std::max(a, b);
    const NarrowphaseCacheKey key{
        lo,
        hi,
        shapeRevisionCounters_[lo],
        shapeRevisionCounters_[hi],
    };
    auto [it, inserted] = narrowphaseCache_.try_emplace(key);
    if (inserted && narrowphaseCache_.size() > 4096) {
        narrowphaseCache_.clear();
        it = narrowphaseCache_.try_emplace(key).first;
    }
    const ConvexSupport supportA = BuildConvexSupport(bodies_[a]);
    const ConvexSupport supportB = BuildConvexSupport(bodies_[b]);
    const GjkDistanceResult gjk = GjkDistance(supportA, supportB, {}, &it->second);
    if (!gjk.intersecting) {
        return false;
    }

    if (gjk.simplex.size >= 3) {
        EpaPenetrationResult epa = ComputePenetrationEPA(supportA, supportB, gjk.simplex);
        if (epa.valid) {
            const auto [lo, hi] = CanonicalBodyOrder(a, b);
            if (a != lo) {
                std::swap(epa.witnessA, epa.witnessB);
                epa.normal = -epa.normal;
            }
            convexManifoldSeeds_[ConvexSeedKey{lo, hi}] = epa;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            if (epa.usedFallback) {
                ++solverTelemetry_.epaFallbackUsed;
            }
            if (epa.reachedIterationLimit) {
                ++solverTelemetry_.epaIterationBailout;
            }
            if (!epa.converged && !epa.reachedIterationLimit) {
                ++solverTelemetry_.epaDegenerateFaces;
            }
#endif
        } else {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
            ++solverTelemetry_.epaDuplicateSupports;
#endif
        }
    }
    return true;
}

void World::SetNarrowphaseDispatchPolicy(NarrowphaseDispatchPolicy policy) {
    narrowphaseDispatchPolicy_ = policy;
}

NarrowphaseDispatchPolicy World::GetNarrowphaseDispatchPolicy() const {
    return narrowphaseDispatchPolicy_;
}

ConvexDispatchRoute World::SelectConvexDispatchRoute(ShapeType a, ShapeType b) const {
    if (narrowphaseDispatchPolicy_ == NarrowphaseDispatchPolicy::PreferGenericConvexCore) {
        return ConvexDispatchRoute::GenericGateThenSpecialized;
    }
    (void)a;
    (void)b;
    return ConvexDispatchRoute::SpecializedOnly;
}

void World::BuildManifolds() {
        std::unordered_map<ManifoldKey, std::unordered_set<PersistentPointKey, PersistentPointKeyHash>, ManifoldKeyHash> warmStartUsedKeys;
        core_internal::ContactSolverContext solverContext{
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
        const core_internal::ContactPipeline pipeline;
        pipeline.BuildManifolds({solverContext});
        for (Manifold& manifold : manifolds_) {
            for (Contact& c : manifold.contacts) {
                if (c.a >= bodies_.size() || c.b >= bodies_.size()) {
                    c.anchorsValid = false;
                    continue;
                }
                const Body& a = bodies_[c.a];
                const Body& b = bodies_[c.b];
                c.localAnchorA = Rotate(Conjugate(a.orientation), c.point - a.position);
                c.localAnchorB = Rotate(Conjugate(b.orientation), c.point - b.position);
                c.referenceSeparation = std::max(c.penetration, 0.0f);
                c.anchorsValid = std::isfinite(c.referenceSeparation);
            }
        }
    }

void World::GenerateContacts() {
        RefreshShapeRevisionCounters();
        convexManifoldSeeds_.clear();
        const core_internal::BroadphaseSystem broadphaseSystem;
        const std::vector<Pair> pairs = broadphaseSystem.ComputePotentialPairs({[this]() { return ComputePotentialPairs(); }});
        std::vector<std::uint32_t> collisionComponent(bodies_.size());
        for (std::uint32_t i = 0; i < collisionComponent.size(); ++i) {
            collisionComponent[i] = i;
        }
        const auto findComponent = [&collisionComponent](std::uint32_t body_id) {
            std::uint32_t root = body_id;
            while (collisionComponent[root] != root) {
                root = collisionComponent[root];
            }
            std::uint32_t cursor = body_id;
            while (collisionComponent[cursor] != root) {
                const std::uint32_t next = collisionComponent[cursor];
                collisionComponent[cursor] = root;
                cursor = next;
            }
            return root;
        };
        const auto unionComponent = [&collisionComponent, &findComponent](std::uint32_t a, std::uint32_t b) {
            const std::uint32_t root_a = findComponent(a);
            const std::uint32_t root_b = findComponent(b);
            if (root_a != root_b) {
                collisionComponent[root_b] = root_a;
            }
        };
        for (const DistanceJoint& joint : joints_) {
            unionComponent(joint.a, joint.b);
        }
        for (const HingeJoint& joint : hingeJoints_) {
            unionComponent(joint.a, joint.b);
        }
        for (const BallSocketJoint& joint : ballSocketJoints_) {
            unionComponent(joint.a, joint.b);
        }
        for (const FixedJoint& joint : fixedJoints_) {
            unionComponent(joint.a, joint.b);
        }
        for (const PrismaticJoint& joint : prismaticJoints_) {
            unionComponent(joint.a, joint.b);
        }
        for (const ServoJoint& joint : servoJoints_) {
            unionComponent(joint.a, joint.b);
        }
        const auto bodiesShareArticulatedComponent = [this, &collisionComponent, &findComponent](
                                                         std::uint32_t a,
                                                         std::uint32_t b) {
            if (a >= collisionComponent.size() || b >= collisionComponent.size()) {
                return false;
            }
            if (bodies_[a].isStatic || bodies_[b].isStatic) {
                return false;
            }
            return findComponent(a) == findComponent(b);
        };

        auto generateCompoundPairContacts = [this](std::uint32_t bodyAId, std::uint32_t bodyBId) {
            const std::vector<ResolvedCollisionShape> shapesA = ResolveCollisionShapes(bodies_[bodyAId]);
            const std::vector<ResolvedCollisionShape> shapesB = ResolveCollisionShapes(bodies_[bodyBId]);

            for (const ResolvedCollisionShape& shapeA : shapesA) {
                for (const ResolvedCollisionShape& shapeB : shapesB) {
                    if (!Overlaps(shapeA.bounds, shapeB.bounds)) {
                        continue;
                    }

                    if (shapeA.body.shape == ShapeType::Plane && shapeB.body.shape == ShapeType::Plane) {
                        continue;
                    }

                    if (shapeA.body.shape == ShapeType::Plane || shapeB.body.shape == ShapeType::Plane) {
                        const bool aIsPlane = shapeA.body.shape == ShapeType::Plane;
                        const Body& plane = aIsPlane ? shapeA.body : shapeB.body;
                        const Body& convex = aIsPlane ? shapeB.body : shapeA.body;
                        const std::uint32_t convexBodyId = aIsPlane ? bodyBId : bodyAId;
                        const std::uint32_t planeBodyId = aIsPlane ? bodyAId : bodyBId;
                        const std::uint32_t convexFeature = aIsPlane ? shapeB.encodedFeature : shapeA.encodedFeature;
                        const std::uint32_t planeFeature = aIsPlane ? shapeA.encodedFeature : shapeB.encodedFeature;
                        Vec3 n{};
                        if (!TryGetPlaneNormal(plane, n)) {
                            continue;
                        }
                        const ConvexSupport support = BuildConvexSupport(convex);
                        if (!support.IsValid()) {
                            continue;
                        }
                        const ConvexSupportPoint sp = support.Support(-n);
                        const float signedDistance = Dot(n, sp.point) - plane.planeOffset;
                        if (signedDistance >= 0.0f) {
                            continue;
                        }
                        const std::uint64_t featureId = CanonicalFeaturePairId(convexBodyId, planeBodyId, convexFeature, planeFeature);
                        AddContact(convexBodyId, planeBodyId, -n, sp.point, -signedDistance, 13u, featureId);
                        continue;
                    }

                    if (!IsConvexPrimitiveShape(shapeA.body.shape) || !IsConvexPrimitiveShape(shapeB.body.shape)) {
                        continue;
                    }

                    EpaPenetrationResult penetration{};
                    if (!ComputeConvexPenetration(shapeA.body, shapeB.body, penetration)) {
                        continue;
                    }
                    Vec3 normal = penetration.normal;
                    if (Dot(normal, shapeB.body.position - shapeA.body.position) < 0.0f) {
                        normal = -normal;
                    }
                    const std::uint16_t detail = static_cast<std::uint16_t>(
                        (static_cast<std::uint16_t>(shapeA.body.shape) << 8u)
                        | static_cast<std::uint16_t>(shapeB.body.shape));
                    const std::uint64_t featureId = CanonicalFeaturePairId(
                        bodyAId,
                        bodyBId,
                        shapeA.encodedFeature,
                        shapeB.encodedFeature,
                        detail);
                    AddContact(
                        bodyAId,
                        bodyBId,
                        normal,
                        0.5f * (penetration.witnessA + penetration.witnessB),
                        penetration.depth,
                        12u,
                        featureId);
                }
            }
        };

        std::vector<Pair> primitivePairs{};
        primitivePairs.reserve(pairs.size());
        for (const Pair& pair : pairs) {
            if (bodiesShareArticulatedComponent(pair.a, pair.b)) {
                continue;
            }
            const Body& a = bodies_[pair.a];
            const Body& b = bodies_[pair.b];
            if (a.shape == ShapeType::Compound || b.shape == ShapeType::Compound) {
                generateCompoundPairContacts(pair.a, pair.b);
            } else {
                primitivePairs.push_back(pair);
            }
        }

        const core_internal::NarrowphaseSystem narrowphaseSystem;
        const core_internal::NarrowphaseContext context{
            bodies_,
            primitivePairs,
            [this](ShapeType a, ShapeType b) { return SelectConvexDispatchRoute(a, b); },
            [this](std::uint32_t a, std::uint32_t b) {
                return ConvexOverlapWithCache(a, b);
            },
            [this](std::uint32_t a, std::uint32_t b) { SphereSphere(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereCapsule(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleCapsule(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsulePlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SpherePlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { BoxPlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { BoxBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { ConvexPlane(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { ConvexConvexEPA(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CylinderBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CylinderCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { SphereHalfCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { HalfCylinderBox(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { HalfCylinderCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { HalfCylinderHalfCylinder(a, b); },
            [this](std::uint32_t a, std::uint32_t b) { CapsuleHalfCylinder(a, b); },
        };
        narrowphaseSystem.GenerateContacts(context);
        GenerateTerrainContacts();
        EmitConvexManifoldSeeds();
    }

void World::GenerateTerrainContacts() {
        if (!terrainAttachment_.enabled || terrainAttachmentBodyId_ == kInvalidBodyId) {
            return;
        }
        if (terrainAttachment_.rows < 2 || terrainAttachment_.cols < 2) {
            return;
        }

        const std::uint32_t terrainBodyId = terrainAttachmentBodyId_;
        for (std::uint32_t bodyId = 0; bodyId < bodies_.size(); ++bodyId) {
            if (bodyId == terrainBodyId) {
                continue;
            }
            const Body& body = bodies_[bodyId];
            if (body.isTerrainAttachment || body.invMass == 0.0f || body.isSleeping) {
                continue;
            }

            std::vector<TerrainContactCandidate> candidates;
            std::unordered_set<std::uint64_t> usedCellKeys;
            candidates.reserve(16);

            const std::vector<ResolvedCollisionShape> shapes = ResolveCollisionShapes(body);
            for (const ResolvedCollisionShape& shape : shapes) {
                if (shape.body.invMass == 0.0f) {
                    continue;
                }
                int row0 = 0;
                int row1 = 0;
                int col0 = 0;
                int col1 = 0;
                if (!TerrainCellRangeForAabb(terrainAttachment_, shape.bounds, row0, row1, col0, col1)) {
                    continue;
                }

                const ConvexSupport support = BuildConvexSupport(shape.body);
                if (!support.IsValid()) {
                    continue;
                }

                for (int row = row0; row <= row1; ++row) {
                    for (int col = col0; col <= col1; ++col) {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                        ++solverTelemetry_.terrainCellsTested;
#endif
                        const float cellX0 = terrainAttachment_.gridOriginWorld.x
                            + static_cast<float>(col) * terrainAttachment_.cellSizeM;
                        const float cellZ0 = terrainAttachment_.gridOriginWorld.z
                            + static_cast<float>(row) * terrainAttachment_.cellSizeM;
                        const float cellX1 = cellX0 + terrainAttachment_.cellSizeM;
                        const float cellZ1 = cellZ0 + terrainAttachment_.cellSizeM;
                        const float sampleX = std::clamp(shape.body.position.x, cellX0, cellX1);
                        const float sampleZ = std::clamp(shape.body.position.z, cellZ0, cellZ1);
                        float terrainHeight = 0.0f;
                        Vec3 terrainNormal{};
                        if (!SampleTerrainCell(terrainAttachment_, row, col, sampleX, sampleZ, terrainHeight, terrainNormal)) {
                            continue;
                        }

                        const ConvexSupportPoint sp = support.Support(-terrainNormal);
                        const float signedDistance = Dot(terrainNormal, sp.point) - terrainHeight;
                        if (signedDistance >= 0.0f) {
                            continue;
                        }

                        TerrainContactCandidate candidate{};
                        candidate.row = static_cast<std::uint32_t>(row);
                        candidate.col = static_cast<std::uint32_t>(col);
                        candidate.shapeFeature = shape.encodedFeature;
                        candidate.normal = terrainNormal;
                        candidate.supportPoint = sp.point;
                        candidate.penetration = -signedDistance;
                        candidate.height = terrainHeight;
                        candidates.push_back(candidate);
                    }
                }
            }
            if (candidates.empty()) {
                continue;
            }

            std::sort(candidates.begin(), candidates.end(), [](const TerrainContactCandidate& lhs, const TerrainContactCandidate& rhs) {
                if (std::abs(lhs.penetration - rhs.penetration) > 1.0e-6f) {
                    return lhs.penetration > rhs.penetration;
                }
                if (lhs.row != rhs.row) {
                    return lhs.row < rhs.row;
                }
                if (lhs.col != rhs.col) {
                    return lhs.col < rhs.col;
                }
                return lhs.shapeFeature < rhs.shapeFeature;
            });

            std::vector<TerrainContactCandidate> chosen;
            chosen.reserve(4);
            for (const TerrainContactCandidate& candidate : candidates) {
                const std::uint64_t cellKey =
                    (static_cast<std::uint64_t>(candidate.row) << 32u) | static_cast<std::uint64_t>(candidate.col);
                if (!usedCellKeys.insert(cellKey).second) {
                    continue;
                }
                chosen.push_back(candidate);
                if (chosen.size() >= 4) {
                    break;
                }
            }

            if (candidates.size() > chosen.size()) {
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                solverTelemetry_.terrainManifoldMerges += static_cast<std::uint64_t>(candidates.size() - chosen.size());
#endif
            }

            for (const TerrainContactCandidate& candidate : chosen) {
                const std::uint32_t cellFeature =
                    candidate.row * static_cast<std::uint32_t>(terrainAttachment_.cols) + candidate.col;
                const std::uint16_t detail = static_cast<std::uint16_t>(
                    static_cast<std::uint16_t>(terrainAttachment_.revision & 0xffffu)
                    ^ static_cast<std::uint16_t>(cellFeature & 0xffffu));
                const std::uint64_t featureId = CanonicalFeaturePairId(
                    bodyId,
                    terrainBodyId,
                    candidate.shapeFeature,
                    cellFeature,
                    detail);
                AddContact(
                    bodyId,
                    terrainBodyId,
                    -candidate.normal,
                    candidate.supportPoint,
                    candidate.penetration,
                    14u,
                    featureId);
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                ++solverTelemetry_.terrainContactsEmitted;
#endif
            }
        }
    }

void World::EmitConvexManifoldSeeds() {
        for (const auto& [key, seed] : convexManifoldSeeds_) {
            if (!seed.valid || seed.depth <= 0.0f) {
                continue;
            }
            bool hasPairContact = false;
            for (const Contact& c : contacts_) {
                const auto [lo, hi] = CanonicalBodyOrder(c.a, c.b);
                if (lo == key.loBody && hi == key.hiBody) {
                    hasPairContact = true;
                    break;
                }
            }
            if (hasPairContact) {
                continue;
            }
            const Vec3 point = 0.5f * (seed.witnessA + seed.witnessB);
            const std::uint64_t featureId = CanonicalFeaturePairId(key.loBody, key.hiBody, 0u, 0u);
            AddContact(key.loBody, key.hiBody, seed.normal, point, seed.depth, 10u, featureId);
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
        const std::uint64_t featureId = CanonicalFeaturePairId(ia, ib, 0u, 0u);
        AddContact(ia, ib, normal, a.position + normal * a.radius, radiusSum - dist, 1u, featureId);
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
                const std::uint32_t referenceFeature =
                    (static_cast<std::uint32_t>(referenceFaceIndex) << 8u) | static_cast<std::uint32_t>(contacts[i].clipMask);
                const std::uint32_t incidentFeature =
                    (static_cast<std::uint32_t>(incidentFaceIndex) << 8u) | static_cast<std::uint32_t>(contacts[i].incidentFeature);
                const std::uint16_t detail = static_cast<std::uint16_t>(contacts[i].clipMask ^ ((contacts[i].incidentFeature & 0xFu) << 4));
                const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, referenceFeature, incidentFeature, detail);
                AddContact(aId, bId, normalAB, contacts[i].point, bestOverlap, 9u, featureId);
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

        const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, edgeA, edgeB);
        AddContact(aId, bId, best.axis, 0.5f * (pointA + pointB), best.overlap, 10u, featureId);
    }


} // namespace minphys3d
