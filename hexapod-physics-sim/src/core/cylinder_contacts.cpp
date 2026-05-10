#include "minphys3d/collision/convex_support.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/narrowphase/epa.hpp"
#include "minphys3d/narrowphase/gjk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace minphys3d {
namespace {

constexpr Real kPi = 3.14159265;

Real ProjectFullCylinderHalfExtent(const Body& cyl, const Vec3& axisUnit) {
    const Vec3 cylAxis = Normalize(Rotate(cyl.orientation, {0.0, 1.0, 0.0}));
    const Real u = std::abs(Dot(cylAxis, axisUnit));
    const Real ring = cyl.radius * std::sqrt(std::max(0.0, 1.0 - u * u));
    return cyl.halfHeight * u + ring;
}

void AppendCylinderSurfaceSamples(const Body& cyl, std::vector<Vec3>& out) {
    const Vec3 A = Normalize(Rotate(cyl.orientation, {0.0, 1.0, 0.0}));
    Vec3 ref = (std::abs(A.x) < 0.85) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    Vec3 u = Normalize(Cross(ref, A));
    if (LengthSquared(u) <= kEpsilon * kEpsilon) {
        u = {1.0, 0.0, 0.0};
    }
    const Vec3 v = Normalize(Cross(A, u));
    constexpr int kSegs = 8;
    for (int capSign : {-1, 1}) {
        const Vec3 capCenter = cyl.position + A * (static_cast<float>(capSign) * cyl.halfHeight);
        for (int k = 0; k < kSegs; ++k) {
            const Real ang = (2.0 * kPi * static_cast<float>(k)) / static_cast<float>(kSegs);
            out.push_back(capCenter + (u * std::cos(ang) + v * std::sin(ang)) * cyl.radius);
        }
    }
    for (int k = 0; k < kSegs; ++k) {
        const Real ang = (2.0 * kPi * static_cast<float>(k)) / static_cast<float>(kSegs);
        const Vec3 rad = (u * std::cos(ang) + v * std::sin(ang)) * cyl.radius;
        out.push_back(cyl.position + rad);
        out.push_back(cyl.position + rad + A * (0.5 * cyl.halfHeight));
        out.push_back(cyl.position + rad - A * (0.5 * cyl.halfHeight));
    }
}

bool LocalPointInsideOBB(const Vec3& local, const Vec3& he, Real eps) {
    return std::abs(local.x) <= he.x + eps && std::abs(local.y) <= he.y + eps && std::abs(local.z) <= he.z + eps;
}

Vec3 SupportPointCylinder(const Body& c, const Vec3& direction) {
    Vec3 localDir = RotateInverse(Normalize(c.orientation), direction);
    const Real radialLengthSq = localDir.x * localDir.x + localDir.z * localDir.z;
    Vec3 localPoint{0.0, localDir.y >= 0.0 ? c.halfHeight : -c.halfHeight, 0.0};
    if (radialLengthSq > kEpsilon * kEpsilon) {
        const Real invRadialLength = 1.0 / std::sqrt(radialLengthSq);
        localPoint.x = localDir.x * invRadialLength * c.radius;
        localPoint.z = localDir.z * invRadialLength * c.radius;
    }
    return c.position + Rotate(c.orientation, localPoint);
}

Vec3 SupportPointCapsule(const Body& c, const Vec3& direction) {
    const Vec3 axis = Normalize(Rotate(c.orientation, {0.0, 1.0, 0.0}));
    Vec3 dir = direction;
    if (LengthSquared(dir) <= kEpsilon * kEpsilon) {
        dir = axis;
    }
    dir = Normalize(dir);
    const Real side = Dot(dir, axis);
    const bool upper = side >= 0.0;
    const Vec3 segmentPoint = c.position + axis * (upper ? c.halfHeight : -c.halfHeight);
    return segmentPoint + dir * c.radius;
}

Vec3 SupportPointHalfCylinder(const Body& body, const Vec3& direction) {
    const Quat invOrientation = Conjugate(Normalize(body.orientation));
    const Vec3 localDir = Rotate(invOrientation, direction);
    const Real hx = body.radius;
    const Real hy = body.halfHeight;

    Vec3 best{};
    Real bestProj = -std::numeric_limits<float>::infinity();
    const auto consider = [&](const Vec3& p) {
        const Real proj = localDir.x * p.x + localDir.y * p.y + localDir.z * p.z;
        if (proj > bestProj) {
            bestProj = proj;
            best = p;
        }
    };

    {
        const Real x = (localDir.x > kEpsilon) ? hx : ((localDir.x < -kEpsilon) ? -hx : 0.0);
        const Real y = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0);
        consider({x, y, 0.0});
    }

    const Real radialLengthSq = localDir.x * localDir.x + localDir.z * localDir.z;
    if (radialLengthSq > kEpsilon * kEpsilon) {
        const Real invRadialLength = 1.0 / std::sqrt(radialLengthSq);
        const Real cx = localDir.x * invRadialLength * hx;
        const Real cz = localDir.z * invRadialLength * hx;
        if (cz >= 0.0) {
            const Real cy = localDir.y >= 0.0 ? hy : -hy;
            consider({cx, cy, cz});
        }
    } else if (localDir.z > kEpsilon) {
        const Real ry = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0);
        consider({0.0, ry, hx});
    }

    const auto considerSemicap = [&](Real capY) {
        const Real dx = localDir.x;
        const Real dz = localDir.z;
        const Real radialXZSq = dx * dx + dz * dz;
        Real sx = 0.0;
        Real sz = 0.0;
        if (radialXZSq > kEpsilon * kEpsilon) {
            const Real inv = 1.0 / std::sqrt(radialXZSq);
            sx = dx * inv * hx;
            sz = dz * inv * hx;
            if (sz < 0.0) {
                sx = (dx > kEpsilon) ? hx : ((dx < -kEpsilon) ? -hx : 0.0);
                sz = 0.0;
            }
        }
        consider({sx, capY, sz});
    };
    considerSemicap(hy);
    considerSemicap(-hy);

    return BodyWorldShapeOrigin(body) + Rotate(body.orientation, best);
}

std::pair<Real, Real> HalfCylinderProjectionInterval(const Body& hc, const Vec3& axisUnit) {
    const Vec3 pHi = SupportPointHalfCylinder(hc, axisUnit);
    const Vec3 pLo = SupportPointHalfCylinder(hc, -axisUnit);
    return {Dot(pLo, axisUnit), Dot(pHi, axisUnit)};
}

std::pair<Real, Real> FullCylinderProjectionInterval(const Body& cyl, const Vec3& axisUnit) {
    const Real c = Dot(cyl.position, axisUnit);
    const Real w = ProjectFullCylinderHalfExtent(cyl, axisUnit);
    return {c - w, c + w};
}

Real IntervalOverlapLength(Real a0, Real a1, Real b0, Real b1) {
    return std::min(a1, b1) - std::max(a0, b0);
}

bool PointInsideFullCylinderWorld(const Body& cyl, const Vec3& p, Real eps) {
    const Quat inv = Conjugate(Normalize(cyl.orientation));
    const Vec3 l = Rotate(inv, p - cyl.position);
    const Real h = cyl.halfHeight;
    const Real r = cyl.radius;
    const Real xz2 = l.x * l.x + l.z * l.z;
    return std::abs(l.y) <= h + eps && xz2 <= (r + eps) * (r + eps);
}

bool PointInsideHalfCylinderWorld(const Body& hc, const Vec3& p, Real eps) {
    const Quat inv = Conjugate(Normalize(hc.orientation));
    const Vec3 l = Rotate(inv, p - BodyWorldShapeOrigin(hc));
    const Real hy = hc.halfHeight;
    const Real r = hc.radius;
    if (l.z < -eps) {
        return false;
    }
    if (std::abs(l.y) > hy + eps) {
        return false;
    }
    return l.x * l.x + l.z * l.z <= (r + eps) * (r + eps);
}

Vec3 ClosestPointOnFullCylinderSurfaceWorld(const Body& cyl, const Vec3& pWorld) {
    const Quat q = Normalize(cyl.orientation);
    const Quat inv = Conjugate(q);
    const Vec3 l = Rotate(inv, pWorld - cyl.position);
    const Real h = cyl.halfHeight;
    const Real r = cyl.radius;

    Vec3 bestW = cyl.position + Rotate(q, Vec3{r, h, 0.0});
    Real bestD2 = LengthSquared(bestW - pWorld);

    const auto considerLocal = [&](const Vec3& candLocal) {
        const Vec3 w = cyl.position + Rotate(q, candLocal);
        const Real d2 = LengthSquared(w - pWorld);
        if (d2 < bestD2) {
            bestD2 = d2;
            bestW = w;
        }
    };

    {
        Real rho = std::sqrt(l.x * l.x + l.z * l.z);
        Real sx = l.x;
        Real sz = l.z;
        if (rho > 1e-8 && rho > r) {
            sx *= r / rho;
            sz *= r / rho;
        }
        considerLocal({sx, h, sz});
    }
    {
        Real rho = std::sqrt(l.x * l.x + l.z * l.z);
        Real sx = l.x;
        Real sz = l.z;
        if (rho > 1e-8 && rho > r) {
            sx *= r / rho;
            sz *= r / rho;
        }
        considerLocal({sx, -h, sz});
    }
    {
        const Real yc = std::clamp(l.y, -h, h);
        const Real rho = std::sqrt(l.x * l.x + l.z * l.z);
        if (rho > 1e-8) {
            considerLocal({l.x / rho * r, yc, l.z / rho * r});
        } else {
            considerLocal({r, yc, 0.0});
        }
    }
    constexpr int kRim = 8;
    for (int i = 0; i < kRim; ++i) {
        const Real ang = (2.0 * kPi * static_cast<float>(i)) / static_cast<float>(kRim);
        const Real c = std::cos(ang) * r;
        const Real s = std::sin(ang) * r;
        considerLocal({c, h, s});
        considerLocal({c, -h, s});
    }
    return bestW;
}

void AppendHalfCylinderSurfaceSamples(const Body& hc, std::vector<Vec3>& out);

Vec3 ClosestHalfCylinderSurfaceToPointWorld(const Body& hc, const Vec3& pWorld, std::vector<Vec3>& scratch) {
    scratch.clear();
    AppendHalfCylinderSurfaceSamples(hc, scratch);
    Vec3 best = scratch[0];
    Real bestD2 = LengthSquared(best - pWorld);
    for (const Vec3& s : scratch) {
        const Real d2 = LengthSquared(s - pWorld);
        if (d2 < bestD2) {
            bestD2 = d2;
            best = s;
        }
    }
    return best;
}

void AppendHalfCylinderSurfaceSamples(const Body& hc, std::vector<Vec3>& out) {
    const Vec3 origin = BodyWorldShapeOrigin(hc);
    const Quat q = Normalize(hc.orientation);
    const Vec3 ex = Rotate(q, {1.0, 0.0, 0.0});
    const Vec3 ey = Rotate(q, {0.0, 1.0, 0.0});
    const Vec3 ez = Rotate(q, {0.0, 0.0, 1.0});
    const Real r = hc.radius;
    const Real hy = hc.halfHeight;
    const std::array<std::pair<Real, Real>, 4> flatCorners{{{r, hy}, {r, -hy}, {-r, hy}, {-r, -hy}}};
    for (const auto& xy : flatCorners) {
        out.push_back(origin + ex * xy.first + ey * xy.second);
    }
    out.push_back(origin + ey * hy);
    out.push_back(origin + ey * (-hy));
    out.push_back(origin);

    constexpr int kArc = 7;
    for (int iy = -1; iy <= 1; ++iy) {
        const Real y = static_cast<float>(iy) * hy;
        for (int k = 0; k <= kArc; ++k) {
            const Real theta = (kPi * static_cast<float>(k)) / static_cast<float>(kArc);
            const Real lx = r * std::cos(theta);
            const Real lz = r * std::sin(theta);
            out.push_back(origin + ex * lx + ey * y + ez * lz);
        }
    }
}

ConvexSupport MakeSphereConvexSupport(const Body& s) {
    return ConvexSupport{
        s.position,
        s.orientation,
        [center = s.position, radius = s.radius](const Vec3& direction) {
            Vec3 dir = direction;
            if (LengthSquared(dir) <= kEpsilon * kEpsilon) {
                dir = {1.0, 0.0, 0.0};
            }
            dir = Normalize(dir);
            return ConvexSupportPoint{center + dir * radius, ShapeTopology{}};
        }};
}

ConvexSupport MakeHalfCylinderConvexSupport(const Body& body) {
    const Vec3 shapeOrigin = BodyWorldShapeOrigin(body);
    return ConvexSupport{
        shapeOrigin,
        body.orientation,
        [center = shapeOrigin, orientation = body.orientation, halfHeight = body.halfHeight, radius = body.radius](const Vec3& direction) {
            const Vec3 localDir = Rotate(Conjugate(orientation), direction);
            const Real hx = radius;
            const Real hy = halfHeight;

            Vec3 best{};
            Real bestProj = -std::numeric_limits<float>::infinity();
            const auto consider = [&](const Vec3& p) {
                const Real proj = localDir.x * p.x + localDir.y * p.y + localDir.z * p.z;
                if (proj > bestProj) {
                    bestProj = proj;
                    best = p;
                }
            };

            {
                const Real x = (localDir.x > kEpsilon) ? hx : ((localDir.x < -kEpsilon) ? -hx : 0.0);
                const Real y = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0);
                consider({x, y, 0.0});
            }

            const Real radialLengthSq = localDir.x * localDir.x + localDir.z * localDir.z;
            if (radialLengthSq > kEpsilon * kEpsilon) {
                const Real invRadialLength = 1.0 / std::sqrt(radialLengthSq);
                const Real cx = localDir.x * invRadialLength * hx;
                const Real cz = localDir.z * invRadialLength * hx;
                if (cz >= 0.0) {
                    const Real cy = localDir.y >= 0.0 ? hy : -hy;
                    consider({cx, cy, cz});
                }
            } else if (localDir.z > kEpsilon) {
                const Real ry = (localDir.y > kEpsilon) ? hy : ((localDir.y < -kEpsilon) ? -hy : 0.0);
                consider({0.0, ry, hx});
            }

            const auto considerSemicap = [&](Real capY) {
                const Real dx = localDir.x;
                const Real dz = localDir.z;
                const Real radialXZSq = dx * dx + dz * dz;
                Real sx = 0.0;
                Real sz = 0.0;
                if (radialXZSq > kEpsilon * kEpsilon) {
                    const Real inv = 1.0 / std::sqrt(radialXZSq);
                    sx = dx * inv * hx;
                    sz = dz * inv * hx;
                    if (sz < 0.0) {
                        sx = (dx > kEpsilon) ? hx : ((dx < -kEpsilon) ? -hx : 0.0);
                        sz = 0.0;
                    }
                }
                consider({sx, capY, sz});
            };
            considerSemicap(hy);
            considerSemicap(-hy);

            return ConvexSupportPoint{center + Rotate(orientation, best), ShapeTopology{}};
        }};
}

} // namespace

// Manifold types: 14 sphere–cylinder, 15 cylinder–box, 16 cylinder–cylinder, 17 capsule–cylinder,
// 18 sphere–half-cylinder (GJK+EPA), 19 half-cylinder–box, 20 half-cylinder–cylinder,
// 21 half-cylinder–half-cylinder, 22 capsule–half-cylinder

void World::SphereCylinder(std::uint32_t sphereId, std::uint32_t cylId) {

    const Body& s = bodies_[sphereId];
    const Body& c = bodies_[cylId];
    const Vec3 axis = Normalize(Rotate(c.orientation, {0.0, 1.0, 0.0}));
    const Vec3 segA = c.position - axis * c.halfHeight;
    const Vec3 segB = c.position + axis * c.halfHeight;
    const Vec3 ab = segB - segA;
    const Real denom = Dot(ab, ab);
    Real t = 0.0;
    if (denom > kEpsilon) {
        t = std::clamp(Dot(s.position - segA, ab) / denom, 0.0, 1.0);
    }
    const Vec3 closest = segA + ab * t;
    const Vec3 delta = closest - s.position;
    const Real distSq = LengthSquared(delta);
    const Real radiusSum = s.radius + c.radius;
    if (distSq > radiusSum * radiusSum) {
        return;
    }
    const Real dist = std::sqrt(std::max(distSq, kEpsilon));
    const Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3{0.0, 1.0, 0.0};
    const std::uint8_t cylFeature = (t <= 1e-3) ? 0u : ((t >= 1.0 - 1e-3) ? 1u : 2u);
    const std::uint64_t featureId = CanonicalFeaturePairId(sphereId, cylId, 0u, cylFeature);
    AddContact(sphereId, cylId, normal, s.position + normal * s.radius, radiusSum - dist, 14u, featureId);
}

void World::CylinderBox(std::uint32_t cylId, std::uint32_t boxId) {

    const Body& cyl = bodies_[cylId];
    const Body& box = bodies_[boxId];
    const Vec3 d = box.position - cyl.position;
    const Vec3 cylAxis = Normalize(Rotate(cyl.orientation, {0.0, 1.0, 0.0}));
    const Vec3 bAxes[3] = {
        Rotate(box.orientation, {1.0, 0.0, 0.0}),
        Rotate(box.orientation, {0.0, 1.0, 0.0}),
        Rotate(box.orientation, {0.0, 0.0, 1.0}),
    };

    constexpr Real kParallelAxisEps = 1e-8;
    constexpr Real kCrossAxisBias = 0.0025;
    Real bestScore = std::numeric_limits<float>::infinity();
    Real bestOverlap = std::numeric_limits<float>::infinity();
    Vec3 bestAxis{1.0, 0.0, 0.0};

    auto tryAxis = [&](const Vec3& axisIn, bool isCross) -> bool {
        const Real lenSq = LengthSquared(axisIn);
        if (lenSq <= kParallelAxisEps) {
            return true;
        }
        const Vec3 axis = axisIn / std::sqrt(lenSq);
        const Real hc = ProjectFullCylinderHalfExtent(cyl, axis);
        const Real hb = ProjectBoxOntoAxis(box, axis);
        const Real dist = std::abs(Dot(d, axis));
        const Real overlap = hc + hb - dist;
        if (overlap < 0.0) {
            return false;
        }
        const Real biased = overlap + (isCross ? kCrossAxisBias : 0.0);
        if (biased < bestScore) {
            bestScore = biased;
            bestOverlap = overlap;
            bestAxis = axis;
            if (Dot(d, bestAxis) < 0.0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!tryAxis(bAxes[i], false)) {
            return;
        }
    }
    if (!tryAxis(cylAxis, false)) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        if (!tryAxis(Cross(bAxes[i], cylAxis), true)) {
            return;
        }
    }

    if (bestOverlap == std::numeric_limits<float>::infinity()) {
        return;
    }

    const Vec3 n = bestAxis;
    const Quat invBoxQ = Conjugate(Normalize(box.orientation));

    struct Sample {
        Vec3 point{};
        Real key = 0.0;
        std::uint32_t feature = 0u;
    };
    std::vector<Vec3> raw;
    AppendCylinderSurfaceSamples(cyl, raw);

    std::vector<Sample> hits;
    hits.reserve(raw.size());
    std::uint32_t geomId = 0;
    for (const Vec3& p : raw) {
        const Vec3 local = Rotate(invBoxQ, p - box.position);
        if (!LocalPointInsideOBB(local, box.halfExtents, 2e-3)) {
            ++geomId;
            continue;
        }
        const Vec3 q = ClosestPointOnBox(box, p);
        Sample s;
        s.point = q;
        s.key = Dot(p, n);
        s.feature = geomId++;
        bool duplicate = false;
        for (const Sample& e : hits) {
            if (LengthSquared(e.point - s.point) < 1e-5) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            hits.push_back(s);
        }
    }

    if (hits.empty()) {
        const Vec3 cylSupport = SupportPointCylinder(cyl, n);
        const Vec3 q = ClosestPointOnBox(box, cylSupport);
        const std::uint64_t featureId = CanonicalFeaturePairId(cylId, boxId, 0u, 0u);
        AddContact(cylId, boxId, n, q, bestOverlap, 15u, featureId);
        return;
    }

    std::sort(hits.begin(), hits.end(), [](const Sample& a, const Sample& b) { return a.key > b.key; });

    const std::size_t maxContacts = 4;
    std::array<std::size_t, 4> pick{};
    std::size_t pickCount = 0;
    if (hits.size() <= maxContacts) {
        for (std::size_t i = 0; i < hits.size(); ++i) {
            pick[pickCount++] = i;
        }
    } else {
        const auto coordOnPlane = [&hits, &n](std::size_t i, const Vec3& u, const Vec3& origin) {
            return Dot(hits[i].point - origin, u);
        };
        const Vec3 ref = (std::abs(n.x) < 0.85) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
        const Vec3 u = Normalize(Cross(n, ref));
        const Vec3 v = Normalize(Cross(n, u));
        const Vec3 origin = box.position;
        const auto pickIndex = [&](bool maxU, bool maxV) {
            std::size_t bestI = 0;
            Real bestScore = -std::numeric_limits<float>::infinity();
            for (std::size_t i = 0; i < hits.size(); ++i) {
                const Real uc = coordOnPlane(i, u, origin);
                const Real vc = coordOnPlane(i, v, origin);
                const Real score = (maxU ? uc : -uc) + (maxV ? vc : -vc);
                if (score > bestScore) {
                    bestScore = score;
                    bestI = i;
                }
            }
            return bestI;
        };
        pick[0] = pickIndex(false, false);
        pick[1] = pickIndex(true, false);
        pick[2] = pickIndex(true, true);
        pick[3] = pickIndex(false, true);
        pickCount = 4;
    }

    for (std::size_t k = 0; k < pickCount; ++k) {
        const Sample& s = hits[pick[k]];
        const std::uint64_t featureId = CanonicalFeaturePairId(cylId, boxId, s.feature, 0u);
        AddContact(cylId, boxId, n, s.point, bestOverlap, 15u, featureId);
    }
}

void World::CylinderCylinder(std::uint32_t aId, std::uint32_t bId) {

    const Body& a = bodies_[aId];
    const Body& b = bodies_[bId];
    const Vec3 axisA = Normalize(Rotate(a.orientation, {0.0, 1.0, 0.0}));
    const Vec3 axisB = Normalize(Rotate(b.orientation, {0.0, 1.0, 0.0}));
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
    const Real distSq = LengthSquared(delta);
    const Real radiusSum = a.radius + b.radius;
    if (distSq > radiusSum * radiusSum) {
        return;
    }
    const Real dist = std::sqrt(std::max(distSq, 0.0));
    const Vec3 centerDelta = b.position - a.position;
    Vec3 normal = StableDirection(delta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
    if (Dot(normal, centerDelta) < 0.0) {
        normal = -normal;
    }
    const Vec3 point = c1 + normal * a.radius;
    const Real penetration = radiusSum - dist;
    const std::uint8_t featureA = (s <= 1e-3) ? 0u : ((s >= 1.0 - 1e-3) ? 1u : 2u);
    const std::uint8_t featureB = (t <= 1e-3) ? 0u : ((t >= 1.0 - 1e-3) ? 1u : 2u);
    const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, featureA, featureB);
    AddContact(aId, bId, normal, point, penetration, 16u, featureId);

    const Real parallelFactor = std::abs(Dot(axisA, axisB));
    if (parallelFactor > 0.98 && featureA == 2u && featureB == 2u) {
        auto addProjectedEndpoint = [&](const Vec3& endpointA, std::uint8_t endpointFeature) {
            const auto [sProj, tProj] = ClosestSegmentParameters(endpointA, endpointA, p2, q2);
            (void)sProj;
            const Vec3 onB = p2 + d2 * tProj;
            const Vec3 epDelta = onB - endpointA;
            const Real epDistSq = LengthSquared(epDelta);
            if (epDistSq > radiusSum * radiusSum) {
                return;
            }
            Vec3 epNormal = StableDirection(epDelta, {centerDelta, axisA, axisB, Cross(axisA, axisB)});
            if (Dot(epNormal, centerDelta) < 0.0) {
                epNormal = -epNormal;
            }
            const Real epDist = std::sqrt(std::max(epDistSq, 0.0));
            const std::uint8_t featureBProj = (tProj <= 1e-3) ? 0u : ((tProj >= 1.0 - 1e-3) ? 1u : 2u);
            const std::uint64_t mfid = CanonicalFeaturePairId(aId, bId, endpointFeature, featureBProj, 1u);
            AddContact(aId, bId, epNormal, endpointA + epNormal * a.radius, radiusSum - epDist, 16u, mfid);
        };
        addProjectedEndpoint(p1, 0u);
        addProjectedEndpoint(q1, 1u);
    }
}

void World::CapsuleCylinder(std::uint32_t capsuleId, std::uint32_t cylId) {

    const Body& cap = bodies_[capsuleId];
    const Body& cyl = bodies_[cylId];
    const Vec3 axisC = Normalize(Rotate(cap.orientation, {0.0, 1.0, 0.0}));
    const Vec3 axisY = Normalize(Rotate(cyl.orientation, {0.0, 1.0, 0.0}));
    const Vec3 p1 = cap.position - axisC * cap.halfHeight;
    const Vec3 q1 = cap.position + axisC * cap.halfHeight;
    const Vec3 p2 = cyl.position - axisY * cyl.halfHeight;
    const Vec3 q2 = cyl.position + axisY * cyl.halfHeight;
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const auto [s, t] = ClosestSegmentParameters(p1, q1, p2, q2);
    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    const Vec3 delta = c2 - c1;
    const Real distSq = LengthSquared(delta);
    const Real radiusSum = cap.radius + cyl.radius;
    if (distSq > radiusSum * radiusSum) {
        return;
    }
    const Real dist = std::sqrt(std::max(distSq, 0.0));
    const Vec3 centerDelta = cyl.position - cap.position;
    Vec3 normal = StableDirection(delta, {centerDelta, axisC, axisY, Cross(axisC, axisY)});
    if (Dot(normal, centerDelta) < 0.0) {
        normal = -normal;
    }
    const Vec3 point = c1 + normal * cap.radius;
    const Real penetration = radiusSum - dist;
    const std::uint8_t featureCap = (s <= 1e-3) ? 0u : ((s >= 1.0 - 1e-3) ? 1u : 2u);
    const std::uint8_t featureCyl = (t <= 1e-3) ? 0u : ((t >= 1.0 - 1e-3) ? 1u : 2u);
    const std::uint64_t featureId = CanonicalFeaturePairId(capsuleId, cylId, featureCap, featureCyl);
    AddContact(capsuleId, cylId, normal, point, penetration, 17u, featureId);

    const Real parallelFactor = std::abs(Dot(axisC, axisY));
    if (parallelFactor > 0.98 && featureCap == 2u && featureCyl == 2u) {
        auto addEndpoint = [&](const Vec3& endpointCap, std::uint8_t capFeat) {
            const auto [sProj, tProj] = ClosestSegmentParameters(endpointCap, endpointCap, p2, q2);
            (void)sProj;
            const Vec3 onCyl = p2 + d2 * tProj;
            const Vec3 epD = onCyl - endpointCap;
            const Real epSq = LengthSquared(epD);
            if (epSq > radiusSum * radiusSum) {
                return;
            }
            Vec3 epN = StableDirection(epD, {centerDelta, axisC, axisY, Cross(axisC, axisY)});
            if (Dot(epN, centerDelta) < 0.0) {
                epN = -epN;
            }
            const Real epDist = std::sqrt(std::max(epSq, 0.0));
            const std::uint8_t cylFeat = (tProj <= 1e-3) ? 0u : ((tProj >= 1.0 - 1e-3) ? 1u : 2u);
            const std::uint64_t mfid = CanonicalFeaturePairId(capsuleId, cylId, capFeat, cylFeat, 1u);
            AddContact(capsuleId, cylId, epN, endpointCap + epN * cap.radius, radiusSum - epDist, 17u, mfid);
        };
        addEndpoint(p1, 0u);
        addEndpoint(q1, 1u);
    }
}

void World::SphereHalfCylinder(std::uint32_t sphereId, std::uint32_t halfCylinderId) {

    const Body& sphere = bodies_[sphereId];
    const Body& hc = bodies_[halfCylinderId];
    const ConvexSupport supportS = MakeSphereConvexSupport(sphere);
    const ConvexSupport supportH = MakeHalfCylinderConvexSupport(hc);
    if (!supportS.IsValid() || !supportH.IsValid()) {
        return;
    }
    const GjkDistanceResult gjk = GjkDistance(supportS, supportH);
    if (!gjk.intersecting) {
        return;
    }
    EpaPenetrationResult penetration{};
    if (gjk.simplex.size >= 3) {
        penetration = ComputePenetrationEPA(supportS, supportH, gjk.simplex);
    }
    if (!penetration.valid || penetration.depth <= 0.0) {
        Vec3 sep = BodyWorldShapeOrigin(hc) - sphere.position;
        if (LengthSquared(sep) <= kEpsilon * kEpsilon) {
            sep = {1.0, 0.0, 0.0};
        } else {
            sep = Normalize(sep);
        }
        const ConvexSupportPoint witnessA = supportS.Support(sep);
        const ConvexSupportPoint witnessB = supportH.Support(-sep);
        const Real depth = Dot(witnessA.point - witnessB.point, sep);
        if (depth <= 0.0) {
            return;
        }
        penetration.valid = true;
        penetration.depth = depth;
        penetration.normal = sep;
        penetration.witnessA = witnessA.point;
        penetration.witnessB = witnessB.point;
    }
    Vec3 normal = penetration.normal;
    if (Dot(normal, BodyWorldShapeOrigin(hc) - sphere.position) < 0.0) {
        normal = -normal;
    }
    const std::uint16_t detail = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(sphere.shape) << 8u) | static_cast<std::uint16_t>(hc.shape));
    const std::uint64_t featureId = CanonicalFeaturePairId(sphereId, halfCylinderId, 0u, 0u, detail);
    AddContact(
        sphereId,
        halfCylinderId,
        normal,
        0.5 * (penetration.witnessA + penetration.witnessB),
        penetration.depth,
        18u,
        featureId);
}

void World::HalfCylinderBox(std::uint32_t halfCylinderId, std::uint32_t boxId) {

    const Body& hc = bodies_[halfCylinderId];
    const Body& box = bodies_[boxId];
    const Vec3 d = box.position - BodyWorldShapeOrigin(hc);
    const Vec3 hcAxis = Normalize(Rotate(hc.orientation, {0.0, 1.0, 0.0}));
    const Vec3 flatOut = Normalize(Rotate(hc.orientation, {0.0, 0.0, -1.0}));
    const Vec3 bAxes[3] = {
        Rotate(box.orientation, {1.0, 0.0, 0.0}),
        Rotate(box.orientation, {0.0, 1.0, 0.0}),
        Rotate(box.orientation, {0.0, 0.0, 1.0}),
    };

    constexpr Real kParallelAxisEps = 1e-8;
    constexpr Real kCrossAxisBias = 0.0025;
    Real bestScore = std::numeric_limits<float>::infinity();
    Real bestOverlap = std::numeric_limits<float>::infinity();
    Vec3 bestAxis{1.0, 0.0, 0.0};

    auto tryAxis = [&](const Vec3& axisIn, bool isCross) -> bool {
        const Real lenSq = LengthSquared(axisIn);
        if (lenSq <= kParallelAxisEps) {
            return true;
        }
        const Vec3 axis = axisIn / std::sqrt(lenSq);
        const auto [hc0, hc1] = HalfCylinderProjectionInterval(hc, axis);
        const Real boxC = Dot(box.position, axis);
        const Real hb = ProjectBoxOntoAxis(box, axis);
        const Real box0 = boxC - hb;
        const Real box1 = boxC + hb;
        const Real overlap = IntervalOverlapLength(hc0, hc1, box0, box1);
        if (overlap < 0.0) {
            return false;
        }
        const Real biased = overlap + (isCross ? kCrossAxisBias : 0.0);
        if (biased < bestScore) {
            bestScore = biased;
            bestOverlap = overlap;
            bestAxis = axis;
            if (Dot(d, bestAxis) < 0.0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!tryAxis(bAxes[i], false)) {
            return;
        }
    }
    if (!tryAxis(hcAxis, false)) {
        return;
    }
    if (!tryAxis(flatOut, false)) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        if (!tryAxis(Cross(bAxes[i], hcAxis), true)) {
            return;
        }
        if (!tryAxis(Cross(bAxes[i], flatOut), true)) {
            return;
        }
    }
    if (!tryAxis(Cross(hcAxis, flatOut), true)) {
        return;
    }

    if (bestOverlap == std::numeric_limits<float>::infinity()) {
        return;
    }

    const Vec3 n = bestAxis;
    const Quat invBoxQ = Conjugate(Normalize(box.orientation));

    struct Sample {
        Vec3 point{};
        Real key = 0.0;
        std::uint32_t feature = 0u;
    };
    std::vector<Vec3> raw;
    AppendHalfCylinderSurfaceSamples(hc, raw);

    std::vector<Sample> hits;
    hits.reserve(raw.size());
    std::uint32_t geomId = 0;
    for (const Vec3& p : raw) {
        const Vec3 local = Rotate(invBoxQ, p - box.position);
        if (!LocalPointInsideOBB(local, box.halfExtents, 2e-3)) {
            ++geomId;
            continue;
        }
        const Vec3 q = ClosestPointOnBox(box, p);
        Sample s;
        s.point = q;
        s.key = Dot(p, n);
        s.feature = geomId++;
        bool duplicate = false;
        for (const Sample& e : hits) {
            if (LengthSquared(e.point - s.point) < 1e-5) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            hits.push_back(s);
        }
    }

    if (hits.empty()) {
        const Vec3 hcSupport = SupportPointHalfCylinder(hc, n);
        const Vec3 q = ClosestPointOnBox(box, hcSupport);
        const std::uint64_t featureId = CanonicalFeaturePairId(halfCylinderId, boxId, 0u, 0u);
        AddContact(halfCylinderId, boxId, n, q, bestOverlap, 19u, featureId);
        return;
    }

    std::sort(hits.begin(), hits.end(), [](const Sample& a, const Sample& b) { return a.key > b.key; });

    const std::size_t maxContacts = 4;
    std::array<std::size_t, 4> pick{};
    std::size_t pickCount = 0;
    if (hits.size() <= maxContacts) {
        for (std::size_t i = 0; i < hits.size(); ++i) {
            pick[pickCount++] = i;
        }
    } else {
        const auto coordOnPlane = [&hits, &n](std::size_t i, const Vec3& u, const Vec3& origin) {
            return Dot(hits[i].point - origin, u);
        };
        const Vec3 ref = (std::abs(n.x) < 0.85) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
        const Vec3 u = Normalize(Cross(n, ref));
        const Vec3 v = Normalize(Cross(n, u));
        const Vec3 origin = box.position;
        const auto pickIndex = [&](bool maxU, bool maxV) {
            std::size_t bestI = 0;
            Real bestS = -std::numeric_limits<float>::infinity();
            for (std::size_t i = 0; i < hits.size(); ++i) {
                const Real uc = coordOnPlane(i, u, origin);
                const Real vc = coordOnPlane(i, v, origin);
                const Real score = (maxU ? uc : -uc) + (maxV ? vc : -vc);
                if (score > bestS) {
                    bestS = score;
                    bestI = i;
                }
            }
            return bestI;
        };
        pick[0] = pickIndex(false, false);
        pick[1] = pickIndex(true, false);
        pick[2] = pickIndex(true, true);
        pick[3] = pickIndex(false, true);
        pickCount = 4;
    }

    for (std::size_t k = 0; k < pickCount; ++k) {
        const Sample& s = hits[pick[k]];
        const std::uint64_t featureId = CanonicalFeaturePairId(halfCylinderId, boxId, s.feature, 0u);
        AddContact(halfCylinderId, boxId, n, s.point, bestOverlap, 19u, featureId);
    }
}

void World::HalfCylinderCylinder(std::uint32_t halfCylinderId, std::uint32_t cylinderId) {

    const Body& hc = bodies_[halfCylinderId];
    const Body& cyl = bodies_[cylinderId];
    const Vec3 d = cyl.position - BodyWorldShapeOrigin(hc);
    const Vec3 axisH = Normalize(Rotate(hc.orientation, {0.0, 1.0, 0.0}));
    const Vec3 flatOut = Normalize(Rotate(hc.orientation, {0.0, 0.0, -1.0}));
    const Vec3 axisC = Normalize(Rotate(cyl.orientation, {0.0, 1.0, 0.0}));

    constexpr Real kParallelAxisEps = 1e-8;
    constexpr Real kCrossAxisBias = 0.0025;
    Real bestScore = std::numeric_limits<float>::infinity();
    Real bestOverlap = std::numeric_limits<float>::infinity();
    Vec3 bestAxis{1.0, 0.0, 0.0};

    auto tryAxis = [&](const Vec3& axisIn, bool isCross) -> bool {
        const Real lenSq = LengthSquared(axisIn);
        if (lenSq <= kParallelAxisEps) {
            return true;
        }
        const Vec3 axis = axisIn / std::sqrt(lenSq);
        const auto [h0, h1] = HalfCylinderProjectionInterval(hc, axis);
        const auto [c0, c1] = FullCylinderProjectionInterval(cyl, axis);
        const Real overlap = IntervalOverlapLength(h0, h1, c0, c1);
        if (overlap < 0.0) {
            return false;
        }
        const Real biased = overlap + (isCross ? kCrossAxisBias : 0.0);
        if (biased < bestScore) {
            bestScore = biased;
            bestOverlap = overlap;
            bestAxis = axis;
            if (Dot(d, bestAxis) < 0.0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    if (!tryAxis(axisC, false)) {
        return;
    }
    if (!tryAxis(axisH, false)) {
        return;
    }
    if (!tryAxis(flatOut, false)) {
        return;
    }
    if (!tryAxis(Cross(axisC, axisH), true)) {
        return;
    }
    if (!tryAxis(Cross(axisC, flatOut), true)) {
        return;
    }
    if (!tryAxis(Cross(axisH, flatOut), true)) {
        return;
    }

    if (bestOverlap == std::numeric_limits<float>::infinity()) {
        return;
    }

    const Vec3 n = bestAxis;
    const Vec3 refOrigin = 0.5 * (BodyWorldShapeOrigin(hc) + cyl.position);

    struct Hit {
        Vec3 point{};
        Real key = 0.0;
        std::uint32_t feature = 0u;
    };
    std::vector<Hit> hits;
    std::vector<Vec3> sHc;
    std::vector<Vec3> sCyl;
    std::vector<Vec3> scratch;
    AppendHalfCylinderSurfaceSamples(hc, sHc);
    AppendCylinderSurfaceSamples(cyl, sCyl);

    const Real insideEps = 0.02;
    const Real mergeTolSq = 1e-5;
    std::uint32_t geomId = 0;

    auto addUniqueHit = [&](const Vec3& contactOnCylinder) {
        for (const Hit& e : hits) {
            if (LengthSquared(e.point - contactOnCylinder) < mergeTolSq) {
                return;
            }
        }
        Hit h;
        h.point = contactOnCylinder;
        h.key = Dot(contactOnCylinder, n);
        h.feature = geomId++;
        hits.push_back(h);
    };

    for (const Vec3& p : sHc) {
        if (!PointInsideFullCylinderWorld(cyl, p, insideEps)) {
            continue;
        }
        const Vec3 q = ClosestPointOnFullCylinderSurfaceWorld(cyl, p);
        addUniqueHit(q);
    }
    for (const Vec3& q : sCyl) {
        if (!PointInsideHalfCylinderWorld(hc, q, insideEps)) {
            continue;
        }
        (void)ClosestHalfCylinderSurfaceToPointWorld(hc, q, scratch);
        addUniqueHit(q);
    }

    if (hits.empty()) {
        const Vec3 pHc = SupportPointHalfCylinder(hc, n);
        const Vec3 pCyl = SupportPointCylinder(cyl, -n);
        const std::uint64_t featureId = CanonicalFeaturePairId(halfCylinderId, cylinderId, 0u, 0u);
        AddContact(halfCylinderId, cylinderId, n, 0.5 * (pHc + pCyl), bestOverlap, 20u, featureId);
        return;
    }

    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.key > b.key; });

    constexpr std::size_t kMax = 4;
    std::array<std::size_t, kMax> pick{};
    std::size_t pickCount = 0;
    if (hits.size() <= kMax) {
        for (std::size_t i = 0; i < hits.size(); ++i) {
            pick[pickCount++] = i;
        }
    } else {
        const auto coordOnPlane = [&hits, &n](std::size_t i, const Vec3& u, const Vec3& origin) {
            return Dot(hits[i].point - origin, u);
        };
        const Vec3 ref = (std::abs(n.x) < 0.85) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
        const Vec3 u = Normalize(Cross(n, ref));
        const Vec3 v = Normalize(Cross(n, u));
        const auto pickIndex = [&](bool maxU, bool maxV) {
            std::size_t bestI = 0;
            Real bestS = -std::numeric_limits<float>::infinity();
            for (std::size_t i = 0; i < hits.size(); ++i) {
                const Real uc = coordOnPlane(i, u, refOrigin);
                const Real vc = coordOnPlane(i, v, refOrigin);
                const Real score = (maxU ? uc : -uc) + (maxV ? vc : -vc);
                if (score > bestS) {
                    bestS = score;
                    bestI = i;
                }
            }
            return bestI;
        };
        pick[0] = pickIndex(false, false);
        pick[1] = pickIndex(true, false);
        pick[2] = pickIndex(true, true);
        pick[3] = pickIndex(false, true);
        pickCount = kMax;
    }

    for (std::size_t k = 0; k < pickCount; ++k) {
        const Hit& s = hits[pick[k]];
        const std::uint64_t featureId = CanonicalFeaturePairId(halfCylinderId, cylinderId, s.feature, 0u);
        AddContact(halfCylinderId, cylinderId, n, s.point, bestOverlap, 20u, featureId);
    }
}

void World::HalfCylinderHalfCylinder(std::uint32_t aId, std::uint32_t bId) {

    const Body& a = bodies_[aId];
    const Body& b = bodies_[bId];
    const Vec3 d = BodyWorldShapeOrigin(b) - BodyWorldShapeOrigin(a);
    const Vec3 axisA = Normalize(Rotate(a.orientation, {0.0, 1.0, 0.0}));
    const Vec3 flatA = Normalize(Rotate(a.orientation, {0.0, 0.0, -1.0}));
    const Vec3 axisB = Normalize(Rotate(b.orientation, {0.0, 1.0, 0.0}));
    const Vec3 flatB = Normalize(Rotate(b.orientation, {0.0, 0.0, -1.0}));

    constexpr Real kParallelAxisEps = 1e-8;
    constexpr Real kCrossAxisBias = 0.0025;
    Real bestScore = std::numeric_limits<float>::infinity();
    Real bestOverlap = std::numeric_limits<float>::infinity();
    Vec3 bestAxis{1.0, 0.0, 0.0};

    auto tryAxis = [&](const Vec3& axisIn, bool isCross) -> bool {
        const Real lenSq = LengthSquared(axisIn);
        if (lenSq <= kParallelAxisEps) {
            return true;
        }
        const Vec3 axis = axisIn / std::sqrt(lenSq);
        const auto [a0, a1] = HalfCylinderProjectionInterval(a, axis);
        const auto [b0, b1] = HalfCylinderProjectionInterval(b, axis);
        const Real overlap = IntervalOverlapLength(a0, a1, b0, b1);
        if (overlap < 0.0) {
            return false;
        }
        const Real biased = overlap + (isCross ? kCrossAxisBias : 0.0);
        if (biased < bestScore) {
            bestScore = biased;
            bestOverlap = overlap;
            bestAxis = axis;
            if (Dot(d, bestAxis) < 0.0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    if (!tryAxis(axisA, false)) {
        return;
    }
    if (!tryAxis(axisB, false)) {
        return;
    }
    if (!tryAxis(flatA, false)) {
        return;
    }
    if (!tryAxis(flatB, false)) {
        return;
    }
    if (!tryAxis(Cross(axisA, axisB), true)) {
        return;
    }
    if (!tryAxis(Cross(axisA, flatB), true)) {
        return;
    }
    if (!tryAxis(Cross(axisB, flatA), true)) {
        return;
    }
    if (!tryAxis(Cross(flatA, flatB), true)) {
        return;
    }

    if (bestOverlap == std::numeric_limits<float>::infinity()) {
        return;
    }

    const Vec3 n = bestAxis;
    const Vec3 refOrigin = 0.5 * (BodyWorldShapeOrigin(a) + BodyWorldShapeOrigin(b));

    struct Hit {
        Vec3 point{};
        Real key = 0.0;
        std::uint32_t feature = 0u;
    };
    std::vector<Hit> hits;
    std::vector<Vec3> sA;
    std::vector<Vec3> sB;
    AppendHalfCylinderSurfaceSamples(a, sA);
    AppendHalfCylinderSurfaceSamples(b, sB);

    const Real rMin = std::min(std::min(a.radius, b.radius), 0.5);
    const Real pairDist =
        std::max(0.06 * rMin, std::min(0.18 * rMin, 0.25 * std::min(bestOverlap, 0.35)));
    const Real pairTolSq = pairDist * pairDist;
    const Real mergeTolSq = 1e-5;
    std::uint32_t geomId = 0;

    auto addUniqueMid = [&](const Vec3& mid) {
        for (const Hit& e : hits) {
            if (LengthSquared(e.point - mid) < mergeTolSq) {
                return;
            }
        }
        Hit h;
        h.point = mid;
        h.key = Dot(mid, n);
        h.feature = geomId++;
        hits.push_back(h);
    };

    for (const Vec3& p : sA) {
        for (const Vec3& q : sB) {
            if (LengthSquared(p - q) <= pairTolSq) {
                addUniqueMid(0.5 * (p + q));
            }
        }
    }

    if (hits.empty()) {
        const Vec3 pA = SupportPointHalfCylinder(a, n);
        const Vec3 pB = SupportPointHalfCylinder(b, -n);
        const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, 0u, 0u);
        AddContact(aId, bId, n, 0.5 * (pA + pB), bestOverlap, 21u, featureId);
        return;
    }

    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.key > b.key; });

    constexpr std::size_t kMax = 4;
    std::array<std::size_t, kMax> pick{};
    std::size_t pickCount = 0;
    if (hits.size() <= kMax) {
        for (std::size_t i = 0; i < hits.size(); ++i) {
            pick[pickCount++] = i;
        }
    } else {
        const auto coordOnPlane = [&hits, &n](std::size_t i, const Vec3& u, const Vec3& origin) {
            return Dot(hits[i].point - origin, u);
        };
        const Vec3 ref = (std::abs(n.x) < 0.85) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
        const Vec3 u = Normalize(Cross(n, ref));
        const Vec3 v = Normalize(Cross(n, u));
        const auto pickIndex = [&](bool maxU, bool maxV) {
            std::size_t bestI = 0;
            Real bestS = -std::numeric_limits<float>::infinity();
            for (std::size_t i = 0; i < hits.size(); ++i) {
                const Real uc = coordOnPlane(i, u, refOrigin);
                const Real vc = coordOnPlane(i, v, refOrigin);
                const Real score = (maxU ? uc : -uc) + (maxV ? vc : -vc);
                if (score > bestS) {
                    bestS = score;
                    bestI = i;
                }
            }
            return bestI;
        };
        pick[0] = pickIndex(false, false);
        pick[1] = pickIndex(true, false);
        pick[2] = pickIndex(true, true);
        pick[3] = pickIndex(false, true);
        pickCount = kMax;
    }

    for (std::size_t k = 0; k < pickCount; ++k) {
        const Hit& s = hits[pick[k]];
        const std::uint64_t featureId = CanonicalFeaturePairId(aId, bId, s.feature, 0u);
        AddContact(aId, bId, n, s.point, bestOverlap, 21u, featureId);
    }
}

void World::CapsuleHalfCylinder(std::uint32_t capsuleId, std::uint32_t halfCylinderId) {

    const Body& cap = bodies_[capsuleId];
    const Body& hc = bodies_[halfCylinderId];
    const Vec3 d = BodyWorldShapeOrigin(hc) - cap.position;
    const Vec3 axisC = Normalize(Rotate(cap.orientation, {0.0, 1.0, 0.0}));
    const Vec3 axisH = Normalize(Rotate(hc.orientation, {0.0, 1.0, 0.0}));
    const Vec3 flatH = Normalize(Rotate(hc.orientation, {0.0, 0.0, -1.0}));

    constexpr Real kParallelAxisEps = 1e-8;
    constexpr Real kCrossAxisBias = 0.0025;
    Real bestScore = std::numeric_limits<float>::infinity();
    Real bestOverlap = std::numeric_limits<float>::infinity();
    Vec3 bestAxis{1.0, 0.0, 0.0};

    auto tryAxis = [&](const Vec3& axisIn, bool isCross) -> bool {
        const Real lenSq = LengthSquared(axisIn);
        if (lenSq <= kParallelAxisEps) {
            return true;
        }
        const Vec3 axis = axisIn / std::sqrt(lenSq);
        const auto [c0, c1] = FullCylinderProjectionInterval(cap, axis);
        const auto [h0, h1] = HalfCylinderProjectionInterval(hc, axis);
        const Real overlap = IntervalOverlapLength(c0, c1, h0, h1);
        if (overlap < 0.0) {
            return false;
        }
        const Real biased = overlap + (isCross ? kCrossAxisBias : 0.0);
        if (biased < bestScore) {
            bestScore = biased;
            bestOverlap = overlap;
            bestAxis = axis;
            if (Dot(d, bestAxis) < 0.0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    if (!tryAxis(axisC, false)) {
        return;
    }
    if (!tryAxis(axisH, false)) {
        return;
    }
    if (!tryAxis(flatH, false)) {
        return;
    }
    if (!tryAxis(Cross(axisC, axisH), true)) {
        return;
    }
    if (!tryAxis(Cross(axisC, flatH), true)) {
        return;
    }
    if (!tryAxis(Cross(axisH, flatH), true)) {
        return;
    }

    if (bestOverlap == std::numeric_limits<float>::infinity()) {
        return;
    }

    const Vec3 n = bestAxis;
    const Vec3 pCap = SupportPointCapsule(cap, n);
    const Vec3 pHc = SupportPointHalfCylinder(hc, -n);
    const std::uint64_t featureId = CanonicalFeaturePairId(capsuleId, halfCylinderId, 0u, 0u);
    AddContact(capsuleId, halfCylinderId, n, 0.5 * (pCap + pHc), bestOverlap, 22u, featureId);

    const Real parallelFactor = std::abs(Dot(axisC, axisH));
    if (parallelFactor > 0.98) {
        const Vec3 hcOrigin = BodyWorldShapeOrigin(hc);
        const Vec3 p1 = cap.position - axisC * cap.halfHeight;
        const Vec3 q1 = cap.position + axisC * cap.halfHeight;
        const Vec3 p2 = hcOrigin - axisH * hc.halfHeight;
        const Vec3 q2 = hcOrigin + axisH * hc.halfHeight;
        const Vec3 hcD = q2 - p2;
        const Real radiusSum = cap.radius + hc.radius;
        const Vec3 centerDelta = hcOrigin - cap.position;
        auto addEndpoint = [&](const Vec3& endpointCap, std::uint8_t capFeat) {
            const auto [sProj, tProj] = ClosestSegmentParameters(endpointCap, endpointCap, p2, q2);
            (void)sProj;
            const Vec3 onHc = p2 + hcD * tProj;
            const Vec3 epD = onHc - endpointCap;
            const Real epSq = LengthSquared(epD);
            if (epSq > radiusSum * radiusSum) {
                return;
            }
            Vec3 epN = StableDirection(epD, {centerDelta, axisC, axisH, Cross(axisC, axisH)});
            if (Dot(epN, centerDelta) < 0.0) {
                epN = -epN;
            }
            const Real epDist = std::sqrt(std::max(epSq, 0.0));
            const std::uint8_t hcFeat = (tProj <= 1e-3) ? 0u : ((tProj >= 1.0 - 1e-3) ? 1u : 2u);
            const std::uint64_t mfid = CanonicalFeaturePairId(capsuleId, halfCylinderId, capFeat, hcFeat, 1u);
            AddContact(capsuleId, halfCylinderId, epN, endpointCap + epN * cap.radius, radiusSum - epDist, 22u, mfid);
        };
        addEndpoint(p1, 0u);
        addEndpoint(q1, 1u);
    }
}

} // namespace minphys3d
