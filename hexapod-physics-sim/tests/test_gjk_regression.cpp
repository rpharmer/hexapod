#include <cassert>
#include <cmath>

#include "minphys3d/core/body.hpp"
#include "minphys3d/narrowphase/gjk.hpp"

using namespace minphys3d;

namespace {

Quat AxisAngle(const Vec3& axis, Real radians) {
    Vec3 n = axis;
    if (!TryNormalize(n, n)) {
        return {};
    }
    const Real half = 0.5 * radians;
    const Real s = std::sin(half);
    return Normalize(Quat{std::cos(half), n.x * s, n.y * s, n.z * s});
}

ConvexSupport BuildConvexSupportForTest(const Body& body) {
    if (body.shape == ShapeType::Sphere) {
        return ConvexSupport{body.position, body.orientation, [center = body.position, radius = body.radius](const Vec3& direction) {
            Vec3 dir = direction;
            if (LengthSquared(dir) <= kEpsilon * kEpsilon) dir = {1.0, 0.0, 0.0};
            dir = Normalize(dir);
            return ConvexSupportPoint{center + dir * radius, {}};
        }};
    }
    if (body.shape == ShapeType::Box) {
        return ConvexSupport{body.position, body.orientation, [center = body.position, orientation = body.orientation, ext = body.halfExtents](const Vec3& direction) {
            const Vec3 localDir = Rotate(Conjugate(orientation), direction);
            const Vec3 local{
                localDir.x >= 0.0 ? ext.x : -ext.x,
                localDir.y >= 0.0 ? ext.y : -ext.y,
                localDir.z >= 0.0 ? ext.z : -ext.z,
            };
            return ConvexSupportPoint{center + Rotate(orientation, local), {}};
        }};
    }
    return {};
}

void TestNearParallelFacesTinyGap() {
    Body a;
    a.shape = ShapeType::Box;
    a.halfExtents = {0.5, 0.45, 0.7};
    a.position = {0.0, 0.0, 0.0};

    Body b = a;
    b.orientation = AxisAngle({0.0, 1.0, 0.0}, 0.0015);
    b.position = {1.003, 0.0, 0.0}; // tiny but positive gap

    NarrowphaseCache cache;
    const GjkDistanceResult result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);
    assert(result.valid);
    assert(!result.intersecting);
    assert(result.distance > 0.0);
    assert(result.distance < 0.01);
}

void TestDeepOverlapEntry() {
    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.5;
    a.position = {0.0, 0.0, 0.0};

    Body b = a;
    b.position = {0.08, 0.0, 0.0};

    const GjkDistanceResult result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
    assert(result.valid);
    assert(result.intersecting);
    assert(result.distance == 0.0);
}

void TestRepeatedFrameCoherence() {
    Body a;
    a.shape = ShapeType::Box;
    a.halfExtents = {0.5, 0.5, 0.5};

    Body b = a;
    b.position = {1.02, 0.03, -0.01};
    b.orientation = AxisAngle({0.0, 0.0, 1.0}, 0.03);

    NarrowphaseCache cache;
    const auto first = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);
    const auto second = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b), {}, &cache);

    assert(first.valid && second.valid);
    assert(!first.intersecting && !second.intersecting);
    assert(std::abs(first.distance - second.distance) <= 1e-5);
    assert(second.iterations <= first.iterations);
}

void TestTinyGapSpheres() {
    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.5;

    Body b = a;
    b.position = {1.0 + 1e-5, 0.0, 0.0};

    const auto result = GjkDistance(BuildConvexSupportForTest(a), BuildConvexSupportForTest(b));
    assert(result.valid);
    assert(!result.intersecting);
    assert(result.distance > 0.0);
    assert(result.distance < 5e-5);
}

} // namespace

int main() {
    TestNearParallelFacesTinyGap();
    TestTinyGapSpheres();
    TestDeepOverlapEntry();
    TestRepeatedFrameCoherence();
    return 0;
}
