#pragma once

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

enum class ShapeType {
    Sphere,
    Box,
    Plane,
    Capsule,
    Cylinder,
    HalfCylinder,
    Compound,
};

inline bool IsConvexPrimitiveShape(ShapeType shape) {
    return shape == ShapeType::Sphere
        || shape == ShapeType::Box
        || shape == ShapeType::Capsule
        || shape == ShapeType::Cylinder
        || shape == ShapeType::HalfCylinder;
}

inline bool IsPlaneShape(ShapeType shape) {
    return shape == ShapeType::Plane;
}

inline bool IsCompoundShape(ShapeType shape) {
    return shape == ShapeType::Compound;
}

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
