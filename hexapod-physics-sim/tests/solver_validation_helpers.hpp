#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "minphys3d/core/world.hpp"

namespace minphys3d::tests {

inline float WrapAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool IsFiniteVec3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool IsFiniteQuat(const Quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

inline Body MakeStaticBase(const Vec3& position = {0.0f, 0.0f, 0.0f}) {
    Body base;
    base.shape = ShapeType::Box;
    base.isStatic = true;
    base.position = position;
    base.halfExtents = {0.12f, 0.12f, 0.12f};
    base.restitution = 0.0f;
    return base;
}

inline Body MakeArmLink(const Vec3& center, float length, float mass) {
    Body link;
    link.shape = ShapeType::Box;
    link.position = center;
    link.halfExtents = {0.5f * length, 0.03f, 0.03f};
    link.mass = mass;
    link.restitution = 0.0f;
    link.staticFriction = 0.8f;
    link.dynamicFriction = 0.6f;
    link.linearDamping = 0.05f;
    link.angularDamping = 0.08f;
    return link;
}

inline float BoxInertiaAboutY(float mass, float full_x, float full_z) {
    return (mass / 12.0f) * (full_x * full_x + full_z * full_z);
}

} // namespace minphys3d::tests
