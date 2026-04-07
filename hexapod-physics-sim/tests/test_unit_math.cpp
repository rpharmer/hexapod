#include <cassert>
#include <cmath>

#include "minphys3d/minphys3d.hpp"

int main() {
    using namespace minphys3d;

    const Vec3 a{1.0f, 2.0f, 3.0f};
    const Vec3 b{4.0f, 5.0f, 6.0f};
    assert(Dot(a, b) == 32.0f);

    const Quat q = Normalize(Quat{0.9238795f, 0.0f, 0.3826834f, 0.0f});
    const Vec3 r = Rotate(q, {1.0f, 0.0f, 0.0f});
    assert(r.z < -0.7f && r.x > 0.6f);

    Vec3 normalized{};
    const bool ok = TryNormalize(Vec3{3.0f, 0.0f, 0.0f}, normalized);
    assert(ok);
    assert(std::abs(normalized.x - 1.0f) < 1e-6f);
    assert(std::abs(normalized.y) < 1e-6f);
    assert(std::abs(normalized.z) < 1e-6f);

    normalized = {42.0f, 42.0f, 42.0f};
    const bool degenerate = TryNormalize(Vec3{0.0f, 0.0f, 0.0f}, normalized);
    assert(!degenerate);
    assert(std::abs(normalized.x) < 1e-6f);
    assert(std::abs(normalized.y) < 1e-6f);
    assert(std::abs(normalized.z) < 1e-6f);

    return 0;
}
