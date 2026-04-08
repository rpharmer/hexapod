#include <cassert>
#include <cmath>

#include "minphys3d/core/body.hpp"
#include "minphys3d/narrowphase/gjk.hpp"

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
    return {};
}

void TestNearParallelFacesTinyGap() {
    Body a;
    a.shape = ShapeType::Box;
    a.halfExtents = {0.5f, 0.45f, 0.7f};
    a.position = {0.0f, 0.0f, 0.0f};

    Body b = a;
    b.orientation = AxisAngle({0.0f, 1.0f, 0.0f}, 0.0015f);
    b.position = {1.003f, 0.0f, 0.0f}; // tiny but positive gap

    NarrowphaseCache cache;
    const GjkDistanceResult result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);
    assert(result.valid);
    assert(!result.intersecting);
    assert(result.distance > 0.0f);
    assert(result.distance < 0.01f);
}

void TestDeepOverlapEntry() {
    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.5f;
    a.position = {0.0f, 0.0f, 0.0f};

    Body b = a;
    b.position = {0.08f, 0.0f, 0.0f};

    const GjkDistanceResult result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
    assert(result.valid);
    assert(result.intersecting);
    assert(result.distance == 0.0f);
}

void TestRepeatedFrameCoherence() {
    Body a;
    a.shape = ShapeType::Box;
    a.halfExtents = {0.5f, 0.5f, 0.5f};

    Body b = a;
    b.position = {1.02f, 0.03f, -0.01f};
    b.orientation = AxisAngle({0.0f, 0.0f, 1.0f}, 0.03f);

    NarrowphaseCache cache;
    const auto first = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);
    const auto second = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);

    assert(first.valid && second.valid);
    assert(!first.intersecting && !second.intersecting);
    assert(std::abs(first.distance - second.distance) <= 1e-5f);
    assert(second.iterations <= first.iterations);
}

void TestTinyGapSpheres() {
    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.5f;

    Body b = a;
    b.position = {1.0f + 1e-5f, 0.0f, 0.0f};

    const auto result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
    assert(result.valid);
    assert(!result.intersecting);
    assert(result.distance > 0.0f);
    assert(result.distance < 5e-5f);
}

} // namespace

int main() {
    TestNearParallelFacesTinyGap();
    TestTinyGapSpheres();
    TestDeepOverlapEntry();
    TestRepeatedFrameCoherence();
    return 0;
}
