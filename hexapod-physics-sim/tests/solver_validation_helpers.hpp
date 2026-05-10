#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "minphys3d/core/world.hpp"

namespace minphys3d::tests {

inline Real WrapAngle(Real angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool IsFiniteVec3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool IsFiniteQuat(const Quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

inline Body MakeStaticBase(const Vec3& position = {0.0, 0.0, 0.0}) {
    Body base;
    base.shape = ShapeType::Box;
    base.isStatic = true;
    base.position = position;
    base.halfExtents = {0.12, 0.12, 0.12};
    base.restitution = 0.0;
    return base;
}

inline Body MakeArmLink(const Vec3& center, Real length, Real mass) {
    Body link;
    link.shape = ShapeType::Box;
    link.position = center;
    link.halfExtents = {0.5 * length, 0.03, 0.03};
    link.mass = mass;
    link.restitution = 0.0;
    link.staticFriction = 0.8;
    link.dynamicFriction = 0.6;
    link.linearDamping = 0.05;
    link.angularDamping = 0.08;
    return link;
}

inline Real BoxInertiaAboutY(Real mass, Real full_x, Real full_z) {
    return (mass / 12.0) * (full_x * full_x + full_z * full_z);
}

} // namespace minphys3d::tests
