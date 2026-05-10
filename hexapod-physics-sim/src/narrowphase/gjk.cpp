#include "minphys3d/narrowphase/gjk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace minphys3d {
namespace {

struct ClosestPointResult {
    int count = 0;
    std::array<int, 4> indices{};
    std::array<Real, 4> weights{};
    Vec3 closest{};
    Real distanceSq = std::numeric_limits<float>::infinity();
};

GjkVertex Support(const ConvexSupport& a, const ConvexSupport& b, const Vec3& direction) {
    const ConvexSupportPoint pointA = a.Support(direction);
    const ConvexSupportPoint pointB = b.Support(-direction);
    return GjkVertex{pointA.point, pointB.point, pointA.point - pointB.point};
}

ClosestPointResult ClosestOnSegment(const GjkSimplex& simplex) {
    const Vec3 a = simplex.vertices[0].point;
    const Vec3 b = simplex.vertices[1].point;
    const Vec3 ab = b - a;
    const Real denom = Dot(ab, ab);
    Real t = 0.0;
    if (denom > kEpsilon) {
        t = std::clamp(-Dot(a, ab) / denom, 0.0, 1.0);
    }
    const Vec3 p = a + t * ab;

    ClosestPointResult out;
    out.count = 2;
    out.indices = {0, 1, 0, 0};
    out.weights = {1.0 - t, t, 0.0, 0.0};
    out.closest = p;
    out.distanceSq = Dot(p, p);
    return out;
}

ClosestPointResult ClosestOnTriangle(const GjkSimplex& simplex) {
    const Vec3 a = simplex.vertices[0].point;
    const Vec3 b = simplex.vertices[1].point;
    const Vec3 c = simplex.vertices[2].point;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = -a;

    const Real d1 = Dot(ab, ap);
    const Real d2 = Dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        ClosestPointResult out;
        out.count = 1;
        out.indices = {0, 0, 0, 0};
        out.weights = {1.0, 0.0, 0.0, 0.0};
        out.closest = a;
        out.distanceSq = Dot(a, a);
        return out;
    }

    const Vec3 bp = -b;
    const Real d3 = Dot(ab, bp);
    const Real d4 = Dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        ClosestPointResult out;
        out.count = 1;
        out.indices = {1, 0, 0, 0};
        out.weights = {1.0, 0.0, 0.0, 0.0};
        out.closest = b;
        out.distanceSq = Dot(b, b);
        return out;
    }

    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const Real v = d1 / (d1 - d3);
        const Vec3 p = a + v * ab;
        ClosestPointResult out;
        out.count = 2;
        out.indices = {0, 1, 0, 0};
        out.weights = {1.0 - v, v, 0.0, 0.0};
        out.closest = p;
        out.distanceSq = Dot(p, p);
        return out;
    }

    const Vec3 cp = -c;
    const Real d5 = Dot(ab, cp);
    const Real d6 = Dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        ClosestPointResult out;
        out.count = 1;
        out.indices = {2, 0, 0, 0};
        out.weights = {1.0, 0.0, 0.0, 0.0};
        out.closest = c;
        out.distanceSq = Dot(c, c);
        return out;
    }

    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const Real w = d2 / (d2 - d6);
        const Vec3 p = a + w * ac;
        ClosestPointResult out;
        out.count = 2;
        out.indices = {0, 2, 0, 0};
        out.weights = {1.0 - w, 0.0, w, 0.0};
        out.closest = p;
        out.distanceSq = Dot(p, p);
        return out;
    }

    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        const Vec3 p = b + w * (c - b);
        ClosestPointResult out;
        out.count = 2;
        out.indices = {1, 2, 0, 0};
        out.weights = {1.0 - w, w, 0.0, 0.0};
        out.closest = p;
        out.distanceSq = Dot(p, p);
        return out;
    }

    const Real denom = 1.0 / (va + vb + vc);
    const Real v = vb * denom;
    const Real w = vc * denom;
    const Real u = 1.0 - v - w;
    const Vec3 p = u * a + v * b + w * c;

    ClosestPointResult out;
    out.count = 3;
    out.indices = {0, 1, 2, 0};
    out.weights = {u, v, w, 0.0};
    out.closest = p;
    out.distanceSq = Dot(p, p);
    return out;
}

ClosestPointResult ClosestOnTetrahedron(const GjkSimplex& simplex) {
    const int faces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    const int opposite[4] = {3, 2, 1, 0};

    bool inside = true;
    ClosestPointResult best;

    for (int fi = 0; fi < 4; ++fi) {
        const Vec3 a = simplex.vertices[faces[fi][0]].point;
        const Vec3 b = simplex.vertices[faces[fi][1]].point;
        const Vec3 c = simplex.vertices[faces[fi][2]].point;
        const Vec3 d = simplex.vertices[opposite[fi]].point;

        Vec3 normal = Cross(b - a, c - a);
        if (Dot(normal, d - a) > 0.0) {
            normal = -normal;
        }
        if (Dot(normal, -a) > 1e-7) {
            inside = false;
            GjkSimplex tri;
            tri.size = 3;
            tri.vertices[0] = simplex.vertices[faces[fi][0]];
            tri.vertices[1] = simplex.vertices[faces[fi][1]];
            tri.vertices[2] = simplex.vertices[faces[fi][2]];
            ClosestPointResult cand = ClosestOnTriangle(tri);
            if (cand.distanceSq < best.distanceSq) {
                best = {};
                best.count = cand.count;
                for (int i = 0; i < cand.count; ++i) {
                    best.indices[i] = faces[fi][cand.indices[i]];
                    best.weights[i] = cand.weights[i];
                }
                best.closest = cand.closest;
                best.distanceSq = cand.distanceSq;
            }
        }
    }

    if (inside) {
        ClosestPointResult out;
        out.count = 4;
        out.indices = {0, 1, 2, 3};
        out.weights = {0.25, 0.25, 0.25, 0.25};
        out.closest = {0.0, 0.0, 0.0};
        out.distanceSq = 0.0;
        return out;
    }

    return best;
}

ClosestPointResult ReduceSimplex(GjkSimplex& simplex) {
    ClosestPointResult result;
    if (simplex.size == 1) {
        result.count = 1;
        result.indices = {0, 0, 0, 0};
        result.weights = {1.0, 0.0, 0.0, 0.0};
        result.closest = simplex.vertices[0].point;
        result.distanceSq = Dot(result.closest, result.closest);
    } else if (simplex.size == 2) {
        result = ClosestOnSegment(simplex);
    } else if (simplex.size == 3) {
        result = ClosestOnTriangle(simplex);
    } else {
        result = ClosestOnTetrahedron(simplex);
    }

    GjkSimplex reduced;
    reduced.size = result.count;
    for (int i = 0; i < result.count; ++i) {
        reduced.vertices[i] = simplex.vertices[result.indices[i]];
    }
    simplex = reduced;
    return result;
}

void ComputeWitnessPoints(const GjkSimplex& simplex, const ClosestPointResult& reduced, Vec3& outA, Vec3& outB) {
    outA = {0.0, 0.0, 0.0};
    outB = {0.0, 0.0, 0.0};
    for (int i = 0; i < reduced.count; ++i) {
        const Real w = reduced.weights[i];
        const GjkVertex& v = simplex.vertices[i];
        outA += w * v.pointA;
        outB += w * v.pointB;
    }
}

} // namespace

GjkDistanceResult GjkDistance(const ConvexSupport& a, const ConvexSupport& b, const GjkSettings& settings, NarrowphaseCache* cache) {
    GjkDistanceResult result;
    if (!a.IsValid() || !b.IsValid()) {
        return result;
    }

    GjkSimplex simplex;
    Vec3 direction = b.Position() - a.Position();
    if (cache != nullptr && cache->hasSimplex && cache->simplex.size > 0) {
        simplex = cache->simplex;
        direction = cache->separatingAxis;
    }
    if (LengthSquared(direction) <= settings.distanceEpsilon * settings.distanceEpsilon) {
        direction = {1.0, 0.0, 0.0};
    }

    if (simplex.size == 0) {
        simplex.vertices[simplex.size++] = Support(a, b, direction);
    }

    Real lastDistanceSq = std::numeric_limits<float>::infinity();

    for (int iter = 0; iter < settings.maxIterations; ++iter) {
        result.iterations = iter + 1;
        const ClosestPointResult reduced = ReduceSimplex(simplex);
        Vec3 closestA{};
        Vec3 closestB{};
        ComputeWitnessPoints(simplex, reduced, closestA, closestB);
        const Vec3 diff = closestA - closestB;
        const Real distSq = Dot(diff, diff);

        if (distSq <= settings.distanceEpsilon * settings.distanceEpsilon || simplex.size == 4) {
            result.valid = true;
            result.intersecting = true;
            result.distance = 0.0;
            result.closestA = closestA;
            result.closestB = closestB;
            result.separatingAxis = (LengthSquared(direction) > settings.distanceEpsilon * settings.distanceEpsilon) ? Normalize(direction) : Vec3{1.0, 0.0, 0.0};
            result.simplex = simplex;
            if (cache != nullptr) {
                cache->simplex = simplex;
                cache->separatingAxis = result.separatingAxis;
                cache->hasSimplex = true;
                cache->lastResultIntersecting = true;
            }
            return result;
        }

        direction = -diff;
        const Real dirLenSq = LengthSquared(direction);
        if (dirLenSq <= settings.distanceEpsilon * settings.distanceEpsilon) {
            result.valid = true;
            result.intersecting = true;
            result.distance = 0.0;
            result.closestA = closestA;
            result.closestB = closestB;
            result.separatingAxis = {1.0, 0.0, 0.0};
            result.simplex = simplex;
            if (cache != nullptr) {
                cache->simplex = simplex;
                cache->separatingAxis = result.separatingAxis;
                cache->hasSimplex = true;
                cache->lastResultIntersecting = true;
            }
            return result;
        }

        const Vec3 dir = direction / std::sqrt(dirLenSq);
        const GjkVertex candidate = Support(a, b, dir);
        const Real progress = Dot(candidate.point, dir) - std::sqrt(distSq);
        if (progress <= settings.supportEpsilon || std::abs(lastDistanceSq - distSq) <= settings.distanceEpsilon * settings.distanceEpsilon) {
            result.valid = true;
            result.intersecting = false;
            result.distance = std::sqrt(std::max(distSq, 0.0));
            result.closestA = closestA;
            result.closestB = closestB;
            result.separatingAxis = -dir;
            result.simplex = simplex;
            if (cache != nullptr) {
                cache->simplex = simplex;
                cache->separatingAxis = result.separatingAxis;
                cache->hasSimplex = true;
                cache->lastResultIntersecting = false;
            }
            return result;
        }

        bool duplicate = false;
        for (int i = 0; i < simplex.size; ++i) {
            if (LengthSquared(simplex.vertices[i].point - candidate.point) <= settings.duplicateEpsilon * settings.duplicateEpsilon) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            result.valid = true;
            result.intersecting = false;
            result.distance = std::sqrt(std::max(distSq, 0.0));
            result.closestA = closestA;
            result.closestB = closestB;
            result.separatingAxis = -dir;
            result.simplex = simplex;
            if (cache != nullptr) {
                cache->simplex = simplex;
                cache->separatingAxis = result.separatingAxis;
                cache->hasSimplex = true;
                cache->lastResultIntersecting = false;
            }
            return result;
        }

        simplex.vertices[simplex.size++] = candidate;
        lastDistanceSq = distSq;
    }

    result.valid = true;
    result.reachedIterationLimit = true;
    result.intersecting = false;
    result.distance = 0.0;
    result.simplex = simplex;
    if (cache != nullptr) {
        cache->simplex = simplex;
        cache->separatingAxis = direction;
        cache->hasSimplex = simplex.size > 0;
        cache->lastResultIntersecting = false;
    }
    return result;
}

} // namespace minphys3d
