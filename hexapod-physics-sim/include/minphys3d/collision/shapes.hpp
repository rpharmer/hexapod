#pragma once

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

enum class ShapeType {
    Sphere,
    Box,
    Plane,
    Capsule,
};

struct AABB {
    Vec3 min{};
    Vec3 max{};
};

inline bool Overlaps(const AABB& a, const AABB& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
        && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

} // namespace minphys3d
