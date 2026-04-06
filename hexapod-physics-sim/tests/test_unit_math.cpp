#include <cassert>

#include "minphys3d/minphys3d.hpp"

int main() {
    using namespace minphys3d;

    const Vec3 a{1.0f, 2.0f, 3.0f};
    const Vec3 b{4.0f, 5.0f, 6.0f};
    assert(Dot(a, b) == 32.0f);

    const Quat q = Normalize(Quat{0.9238795f, 0.0f, 0.3826834f, 0.0f});
    const Vec3 r = Rotate(q, {1.0f, 0.0f, 0.0f});
    assert(r.z < -0.7f && r.x > 0.6f);

    return 0;
}
