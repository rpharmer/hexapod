#include "minphys3d/narrowphase/epa.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace minphys3d {
namespace {

struct EpaVertex {
    Vec3 pointA{};
    Vec3 pointB{};
    Vec3 point{};
};

struct EpaFace {
    int i0 = 0;
    int i1 = 0;
    int i2 = 0;
    Vec3 normal{};
    float distance = 0.0f;
    bool valid = false;
};

struct Edge {
    int a = 0;
    int b = 0;
};

EpaVertex Support(const ConvexSupport& a, const ConvexSupport& b, Vec3 direction) {
    if (LengthSquared(direction) <= kEpsilon * kEpsilon) {
        direction = {1.0f, 0.0f, 0.0f};
    }
    direction = Normalize(direction);
    const ConvexSupportPoint sa = a.Support(direction);
    const ConvexSupportPoint sb = b.Support(-direction);
    return EpaVertex{sa.point, sb.point, sa.point - sb.point};
}

bool IsDuplicatePoint(const std::vector<EpaVertex>& vertices, const EpaVertex& candidate, float epsilon) {
    for (const EpaVertex& v : vertices) {
        if (LengthSquared(v.point - candidate.point) <= epsilon * epsilon) {
            return true;
        }
    }
    return false;
}

bool BuildFace(const std::vector<EpaVertex>& vertices, int i0, int i1, int i2, float faceEps, EpaFace& out) {
    const Vec3& a = vertices[static_cast<std::size_t>(i0)].point;
    const Vec3& b = vertices[static_cast<std::size_t>(i1)].point;
    const Vec3& c = vertices[static_cast<std::size_t>(i2)].point;

    Vec3 n = Cross(b - a, c - a);
    const float lenSq = LengthSquared(n);
    if (lenSq <= faceEps * faceEps) {
        return false;
    }

    n = n / std::sqrt(lenSq);
    float d = Dot(n, a);
    if (d < 0.0f) {
        n = -n;
        d = -d;
        std::swap(i1, i2);
    }

    out = EpaFace{i0, i1, i2, n, d, true};
    return true;
}

void AddEdge(std::vector<Edge>& edges, int a, int b) {
    // Edge cavity: if the mirror (b,a) already exists, cancel both (silhouette extraction).
    // Linear scan is fine for typical N=16-40 cavity sizes; the original code used
    // vector::erase which was O(n) per removal due to element shifting — replace with
    // O(1) swap-and-pop.
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (edges[i].a == b && edges[i].b == a) {
            edges[i] = edges.back();
            edges.pop_back();
            return;
        }
    }
    edges.push_back({a, b});
}

Vec3 AnyStableNormal(const std::vector<EpaVertex>& vertices) {
    for (std::size_t i = 0; i + 2 < vertices.size(); ++i) {
        const Vec3 n = Cross(vertices[i + 1].point - vertices[i].point, vertices[i + 2].point - vertices[i].point);
        if (LengthSquared(n) > kEpsilon * kEpsilon) {
            return Normalize(n);
        }
    }
    return {1.0f, 0.0f, 0.0f};
}

ConvexSupport OffsetSupport(const ConvexSupport& src, const Vec3& offset) {
    return ConvexSupport{
        src.Position() + offset,
        src.Orientation(),
        [&src, offset](const Vec3& direction) {
            ConvexSupportPoint p = src.Support(direction);
            p.point += offset;
            return p;
        }};
}

bool ConservativeAdvanceRecover(
    const ConvexSupport& a,
    const ConvexSupport& b,
    Vec3 normal,
    int bisectionIters,
    Vec3& outWitnessA,
    Vec3& outWitnessB,
    float& outDepth) {
    if (LengthSquared(normal) <= kEpsilon * kEpsilon) {
        normal = b.Position() - a.Position();
        if (LengthSquared(normal) <= kEpsilon * kEpsilon) {
            normal = {1.0f, 0.0f, 0.0f};
        }
    }
    normal = Normalize(normal);

    float hi = 0.0f;
    bool separated = false;
    for (int i = 0; i < 12; ++i) {
        hi = (i == 0) ? 1e-4f : hi * 2.0f;
        const ConvexSupport aShift = OffsetSupport(a, -normal * hi);
        const ConvexSupport bShift = OffsetSupport(b, normal * hi);
        const GjkDistanceResult probe = GjkDistance(aShift, bShift);
        if (probe.valid && !probe.intersecting) {
            separated = true;
            break;
        }
    }

    if (!separated) {
        const EpaVertex sa = Support(a, b, normal);
        const EpaVertex sb = Support(a, b, -normal);
        outWitnessA = 0.5f * (sa.pointA + sb.pointA);
        outWitnessB = 0.5f * (sa.pointB + sb.pointB);
        outDepth = std::max(0.0f, Dot(sa.pointA - sa.pointB, normal));
        return outDepth > 0.0f;
    }

    float lo = 0.0f;
    for (int i = 0; i < bisectionIters; ++i) {
        const float mid = 0.5f * (lo + hi);
        const ConvexSupport aShift = OffsetSupport(a, -normal * mid);
        const ConvexSupport bShift = OffsetSupport(b, normal * mid);
        const GjkDistanceResult probe = GjkDistance(aShift, bShift);
        if (probe.valid && !probe.intersecting) {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    const ConvexSupport aShift = OffsetSupport(a, -normal * hi);
    const ConvexSupport bShift = OffsetSupport(b, normal * hi);
    const GjkDistanceResult finalProbe = GjkDistance(aShift, bShift);
    if (finalProbe.valid) {
        outWitnessA = finalProbe.closestA + normal * hi;
        outWitnessB = finalProbe.closestB - normal * hi;
    } else {
        const EpaVertex sa = Support(a, b, normal);
        outWitnessA = sa.pointA;
        outWitnessB = sa.pointB;
    }

    const float geometricDepth = Dot(outWitnessA - outWitnessB, normal);
    outDepth = std::max(geometricDepth, hi * 2.0f);
    return outDepth > 0.0f;
}

void LocalClipWitness(const ConvexSupport& a, const ConvexSupport& b, const Vec3& normal, Vec3& ioA, Vec3& ioB) {
    Vec3 tangent0 = Cross(normal, std::abs(normal.y) < 0.9f ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f});
    if (!TryNormalize(tangent0, tangent0)) {
        return;
    }
    Vec3 tangent1 = Normalize(Cross(normal, tangent0));

    std::array<Vec3, 4> dirs{tangent0, -tangent0, tangent1, -tangent1};
    Vec3 sumA = ioA;
    Vec3 sumB = ioB;
    int count = 1;
    for (const Vec3& t : dirs) {
        const EpaVertex sa = Support(a, b, normal + 0.3f * t);
        const EpaVertex sb = Support(a, b, normal - 0.3f * t);
        sumA += 0.5f * (sa.pointA + sb.pointA);
        sumB += 0.5f * (sa.pointB + sb.pointB);
        ++count;
    }
    ioA = sumA / static_cast<float>(count);
    ioB = sumB / static_cast<float>(count);
}

} // namespace

EpaPenetrationResult ComputePenetrationEPA(const ConvexSupport& a, const ConvexSupport& b, const GjkSimplex& simplex, const EpaSettings& settings) {
    EpaPenetrationResult out;
    if (!a.IsValid() || !b.IsValid() || simplex.size < 3) {
        return out;
    }

    std::vector<EpaVertex> vertices;
    vertices.reserve(32);
    for (int i = 0; i < simplex.size; ++i) {
        const GjkVertex& gv = simplex.vertices[i];
        vertices.push_back({gv.pointA, gv.pointB, gv.point});
    }

    if (vertices.size() == 3) {
        const Vec3 triNormal = Cross(vertices[1].point - vertices[0].point, vertices[2].point - vertices[0].point);
        Vec3 dir = (LengthSquared(triNormal) > settings.faceEpsilon * settings.faceEpsilon)
            ? triNormal
            : Vec3{1.0f, 0.0f, 0.0f};
        EpaVertex extra = Support(a, b, dir);
        if (IsDuplicatePoint(vertices, extra, settings.duplicateEpsilon)) {
            extra = Support(a, b, -dir);
        }
        if (!IsDuplicatePoint(vertices, extra, settings.duplicateEpsilon)) {
            vertices.push_back(extra);
        }
    }

    if (vertices.size() < 4) {
        out.usedFallback = true;
        Vec3 normal = AnyStableNormal(vertices);
        if (Dot(b.Position() - a.Position(), normal) < 0.0f) {
            normal = -normal;
        }
        if (ConservativeAdvanceRecover(a, b, normal, settings.fallbackBisectionIterations, out.witnessA, out.witnessB, out.depth)) {
            LocalClipWitness(a, b, normal, out.witnessA, out.witnessB);
            out.valid = true;
            out.normal = normal;
        }
        return out;
    }

    std::vector<EpaFace> faces;
    faces.reserve(64);
    auto tryAddFace = [&](int i0, int i1, int i2) {
        EpaFace f;
        if (BuildFace(vertices, i0, i1, i2, settings.faceEpsilon, f)) {
            faces.push_back(f);
            return true;
        }
        return false;
    };

    tryAddFace(0, 1, 2);
    tryAddFace(0, 3, 1);
    tryAddFace(0, 2, 3);
    tryAddFace(1, 3, 2);

    if (faces.empty()) {
        out.usedFallback = true;
        Vec3 normal = Normalize(b.Position() - a.Position());
        if (LengthSquared(normal) <= kEpsilon * kEpsilon) {
            normal = {1.0f, 0.0f, 0.0f};
        }
        if (ConservativeAdvanceRecover(a, b, normal, settings.fallbackBisectionIterations, out.witnessA, out.witnessB, out.depth)) {
            LocalClipWitness(a, b, normal, out.witnessA, out.witnessB);
            out.valid = true;
            out.normal = normal;
        }
        return out;
    }

    for (int iter = 0; iter < settings.maxIterations; ++iter) {
        out.iterations = iter + 1;
        float bestDist = std::numeric_limits<float>::infinity();
        int bestFace = -1;
        for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
            if (!faces[static_cast<std::size_t>(i)].valid) {
                continue;
            }
            if (faces[static_cast<std::size_t>(i)].distance < bestDist) {
                bestDist = faces[static_cast<std::size_t>(i)].distance;
                bestFace = i;
            }
        }

        if (bestFace < 0) {
            break;
        }

        const EpaFace face = faces[static_cast<std::size_t>(bestFace)];
        const EpaVertex candidate = Support(a, b, face.normal);
        const float supportDist = Dot(candidate.point, face.normal);
        const float advance = supportDist - face.distance;

        if (advance <= settings.convergenceEpsilon || IsDuplicatePoint(vertices, candidate, settings.duplicateEpsilon)) {
            const Vec3& pa = vertices[static_cast<std::size_t>(face.i0)].point;
            const Vec3& pb = vertices[static_cast<std::size_t>(face.i1)].point;
            const Vec3& pc = vertices[static_cast<std::size_t>(face.i2)].point;
            const Vec3 p = face.normal * face.distance;

            const Vec3 v0 = pb - pa;
            const Vec3 v1 = pc - pa;
            const Vec3 v2 = p - pa;
            const float d00 = Dot(v0, v0);
            const float d01 = Dot(v0, v1);
            const float d11 = Dot(v1, v1);
            const float d20 = Dot(v2, v0);
            const float d21 = Dot(v2, v1);
            const float denom = d00 * d11 - d01 * d01;

            float u = 1.0f;
            float v = 0.0f;
            float w = 0.0f;
            if (std::abs(denom) > kEpsilon) {
                v = (d11 * d20 - d01 * d21) / denom;
                w = (d00 * d21 - d01 * d20) / denom;
                u = 1.0f - v - w;
                u = std::clamp(u, 0.0f, 1.0f);
                v = std::clamp(v, 0.0f, 1.0f);
                w = std::clamp(w, 0.0f, 1.0f);
                const float sum = u + v + w;
                if (sum > kEpsilon) {
                    const float inv = 1.0f / sum;
                    u *= inv;
                    v *= inv;
                    w *= inv;
                }
            }

            const EpaVertex& va = vertices[static_cast<std::size_t>(face.i0)];
            const EpaVertex& vb = vertices[static_cast<std::size_t>(face.i1)];
            const EpaVertex& vc = vertices[static_cast<std::size_t>(face.i2)];

            out.witnessA = u * va.pointA + v * vb.pointA + w * vc.pointA;
            out.witnessB = u * va.pointB + v * vb.pointB + w * vc.pointB;
            out.depth = std::max(0.0f, Dot(out.witnessA - out.witnessB, face.normal));
            out.normal = face.normal;
            if (Dot(b.Position() - a.Position(), out.normal) < 0.0f) {
                out.normal = -out.normal;
            }
            out.valid = true;
            out.converged = (advance <= settings.convergenceEpsilon);
            return out;
        }

        const int newIndex = static_cast<int>(vertices.size());
        vertices.push_back(candidate);

        std::vector<Edge> boundary;
        boundary.reserve(faces.size() * 2);
        for (EpaFace& f : faces) {
            if (!f.valid) {
                continue;
            }
            const Vec3& fa = vertices[static_cast<std::size_t>(f.i0)].point;
            if (Dot(f.normal, candidate.point - fa) > settings.faceEpsilon) {
                f.valid = false;
                AddEdge(boundary, f.i0, f.i1);
                AddEdge(boundary, f.i1, f.i2);
                AddEdge(boundary, f.i2, f.i0);
            }
        }

        for (const Edge& edge : boundary) {
            EpaFace nf;
            if (BuildFace(vertices, edge.a, edge.b, newIndex, settings.faceEpsilon, nf)) {
                faces.push_back(nf);
            }
        }
    }

    out.reachedIterationLimit = true;
    out.usedFallback = true;
    Vec3 fallbackNormal = Normalize(b.Position() - a.Position());
    if (LengthSquared(fallbackNormal) <= kEpsilon * kEpsilon) {
        fallbackNormal = AnyStableNormal(vertices);
    }
    if (ConservativeAdvanceRecover(a, b, fallbackNormal, settings.fallbackBisectionIterations, out.witnessA, out.witnessB, out.depth)) {
        LocalClipWitness(a, b, fallbackNormal, out.witnessA, out.witnessB);
        out.normal = fallbackNormal;
        out.valid = true;
    }
    return out;
}

} // namespace minphys3d
