#include <cassert>
#include <cmath>

#include "minphys3d/minphys3d.hpp"

int main() {
    using namespace minphys3d;

    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 5.0, 6.0};
    assert(Dot(a, b) == 32.0);

    const Quat q = Normalize(Quat{0.9238795, 0.0, 0.3826834, 0.0});
    const Vec3 r = Rotate(q, {1.0, 0.0, 0.0});
    assert(r.z < -0.7 && r.x > 0.6);

    Vec3 normalized{};
    const bool ok = TryNormalize(Vec3{3.0, 0.0, 0.0}, normalized);
    assert(ok);
    assert(std::abs(normalized.x - 1.0) < 1e-6);
    assert(std::abs(normalized.y) < 1e-6);
    assert(std::abs(normalized.z) < 1e-6);

    normalized = {42.0, 42.0, 42.0};
    const bool degenerate = TryNormalize(Vec3{0.0, 0.0, 0.0}, normalized);
    assert(!degenerate);
    assert(std::abs(normalized.x) < 1e-6);
    assert(std::abs(normalized.y) < 1e-6);
    assert(std::abs(normalized.z) < 1e-6);

    return 0;
}
