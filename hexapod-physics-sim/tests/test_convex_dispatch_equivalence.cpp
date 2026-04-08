#include <cassert>
#include <cstdio>
#include <vector>

#include "minphys3d/collision/convex_support.hpp"
#include "minphys3d/core/body.hpp"

using namespace minphys3d;

namespace {
Quat AxisAngle(const Vec3& axis, float radians) {
    Vec3 n = axis;
    if (!TryNormalize(n, n)) {
        return {};
    }
    const float half = 0.5f * radians;
    const float s = std::sin(half);
    return Normalize(Quat{std::cos(half), n.x * s, n.y * s, n.z * s});
}

ConvexSupport BuildConvexSupportForTest(const Body& body) {
    if (body.shape == ShapeType::Sphere) {
        return ConvexSupport{body.position, body.orientation, [center = body.position, radius = body.radius](const Vec3& direction) {
            Vec3 dir = direction;
            if (LengthSquared(dir) <= kEpsilon * kEpsilon) dir = {1.0f, 0.0f, 0.0f};
            dir = Normalize(dir);
            return ConvexSupportPoint{center + dir * radius, {}};
        }};
    }
    if (body.shape == ShapeType::Box) {
        return ConvexSupport{body.position, body.orientation, [center = body.position, orientation = body.orientation, ext = body.halfExtents](const Vec3& direction) {
            const Vec3 localDir = Rotate(Conjugate(orientation), direction);
            const Vec3 local{
                localDir.x >= 0.0f ? ext.x : -ext.x,
                localDir.y >= 0.0f ? ext.y : -ext.y,
                localDir.z >= 0.0f ? ext.z : -ext.z,
            };
            return ConvexSupportPoint{center + Rotate(orientation, local), {}};
        }};
    }
    if (body.shape == ShapeType::Capsule) {
        return ConvexSupport{body.position, body.orientation, [center = body.position, orientation = body.orientation, halfHeight = body.halfHeight, radius = body.radius](const Vec3& direction) {
            const Vec3 axis = Normalize(Rotate(orientation, {0.0f, 1.0f, 0.0f}));
            Vec3 dir = direction;
            if (LengthSquared(dir) <= kEpsilon * kEpsilon) dir = axis;
            dir = Normalize(dir);
            const Vec3 endpoint = center + axis * (Dot(dir, axis) >= 0.0f ? halfHeight : -halfHeight);
            return ConvexSupportPoint{endpoint + dir * radius, {}};
        }};
    }
    return {};
}

bool SpecializedSphereSphere(const Body& a, const Body& b) {
    const float r = a.radius + b.radius;
    return LengthSquared(b.position - a.position) <= r * r;
}

bool SpecializedSphereBox(const Body& sphere, const Body& box) {
    const Quat invQ = Conjugate(Normalize(box.orientation));
    const Vec3 p = Rotate(invQ, sphere.position - box.position);
    const Vec3 closest{
        std::clamp(p.x, -box.halfExtents.x, box.halfExtents.x),
        std::clamp(p.y, -box.halfExtents.y, box.halfExtents.y),
        std::clamp(p.z, -box.halfExtents.z, box.halfExtents.z),
    };
    return LengthSquared(p - closest) <= sphere.radius * sphere.radius;
}

float ProjectBoxOntoAxis(const Body& box, const Vec3& axis) {
    const Vec3 x = Rotate(box.orientation, {1.0f, 0.0f, 0.0f});
    const Vec3 y = Rotate(box.orientation, {0.0f, 1.0f, 0.0f});
    const Vec3 z = Rotate(box.orientation, {0.0f, 0.0f, 1.0f});
    return std::abs(Dot(x, axis)) * box.halfExtents.x + std::abs(Dot(y, axis)) * box.halfExtents.y + std::abs(Dot(z, axis)) * box.halfExtents.z;
}

bool SpecializedBoxBox(const Body& a, const Body& b) {
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
    const Vec3 centerDelta = b.position - a.position;

    std::vector<Vec3> axes;
    axes.reserve(15);
    axes.insert(axes.end(), aAxes, aAxes + 3);
    axes.insert(axes.end(), bAxes, bAxes + 3);
    for (const Vec3& u : aAxes) {
        for (const Vec3& v : bAxes) {
            axes.push_back(Cross(u, v));
        }
    }

    for (Vec3 axis : axes) {
        const float lenSq = LengthSquared(axis);
        if (lenSq <= 1e-8f) continue;
        axis = axis / std::sqrt(lenSq);
        const float overlap = ProjectBoxOntoAxis(a, axis) + ProjectBoxOntoAxis(b, axis) - std::abs(Dot(centerDelta, axis));
        if (overlap < 0.0f) return false;
    }
    return true;
}

bool SpecializedOverlap(const Body& a, const Body& b) {
    if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) return SpecializedSphereSphere(a, b);
    return ConvexOverlapGJK(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
}

void ExpectEquivalent(const char* label, const Body& a, const Body& b) {
    const bool specialized = SpecializedOverlap(a, b);
    const bool generic = ConvexOverlapGJK(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
    if (specialized != generic) {
        std::fprintf(stderr, "Mismatch in %s (specialized=%d generic=%d)\\n", label, specialized ? 1 : 0, generic ? 1 : 0);
    }
    assert(specialized == generic);
}

} // namespace

int main() {
    Body s0;
    s0.shape = ShapeType::Sphere;
    s0.radius = 0.5f;
    s0.position = {0.0f, 0.0f, 0.0f};

    Body s1 = s0;
    s1.position = {0.75f, 0.0f, 0.0f};
    ExpectEquivalent("sphere-sphere-overlap", s0, s1);
    s1.position = {1.5f, 0.0f, 0.0f};
    ExpectEquivalent("sphere-sphere-separate", s0, s1);

    Body capsule;
    capsule.shape = ShapeType::Capsule;
    capsule.radius = 0.2f;
    capsule.halfHeight = 0.6f;
    capsule.position = {0.0f, 0.0f, 0.0f};
    capsule.orientation = AxisAngle({0.0f, 0.0f, 1.0f}, 0.25f);
    ExpectEquivalent("capsule-sphere-overlap", capsule, s0);
    s1.position = {2.0f, 0.0f, 0.0f};
    ExpectEquivalent("capsule-sphere-separate", capsule, s1);

    return 0;
}
