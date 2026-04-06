#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/math/mat3.hpp"

namespace minphys3d {

struct Body {
    ShapeType shape = ShapeType::Sphere;

    Vec3 position{};
    Vec3 velocity{};
    Vec3 force{};

    Quat orientation{};
    Vec3 angularVelocity{};
    Vec3 torque{};

    float radius = 0.5f;
    float halfHeight = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};

    Vec3 planeNormal{0.0f, 1.0f, 0.0f};
    float planeOffset = 0.0f;

    float mass = 1.0f;
    float invMass = 1.0f;
    Mat3 invInertiaLocal = Mat3::Identity();
    float restitution = 0.2f;
    float staticFriction = 0.6f;
    float dynamicFriction = 0.4f;
    bool isStatic = false;
    bool isSleeping = false;
    int sleepCounter = 0;

    void RecomputeMassProperties();
    Mat3 InvInertiaWorld() const;
    AABB ComputeAABB() const;
};

inline void Body::RecomputeMassProperties() {
    if (shape == ShapeType::Plane) isStatic = true;

    if (isStatic || mass <= kEpsilon) {
        mass = std::numeric_limits<float>::infinity();
        invMass = 0.0f;
        invInertiaLocal = {};
        isStatic = true;
        isSleeping = false;
        return;
    }

    invMass = 1.0f / mass;
    invInertiaLocal = {};

    if (shape == ShapeType::Sphere) {
        const float inertia = 0.4f * mass * radius * radius;
        const float invI = (inertia > kEpsilon) ? (1.0f / inertia) : 0.0f;
        invInertiaLocal.m[0][0] = invI;
        invInertiaLocal.m[1][1] = invI;
        invInertiaLocal.m[2][2] = invI;
    } else if (shape == ShapeType::Capsule) {
        const float capsuleHeight = halfHeight * 2.0f + 2.0f * radius;
        const float ix = 0.25f * mass * radius * radius + (1.0f / 12.0f) * mass * capsuleHeight * capsuleHeight;
        const float iy = 0.5f * mass * radius * radius;
        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0f / iy) : 0.0f;
        invInertiaLocal.m[2][2] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
    } else if (shape == ShapeType::Box) {
        const float wx = halfExtents.x * 2.0f;
        const float wy = halfExtents.y * 2.0f;
        const float wz = halfExtents.z * 2.0f;

        const float ix = (mass / 12.0f) * (wy * wy + wz * wz);
        const float iy = (mass / 12.0f) * (wx * wx + wz * wz);
        const float iz = (mass / 12.0f) * (wx * wx + wy * wy);

        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0f / iy) : 0.0f;
        invInertiaLocal.m[2][2] = (iz > kEpsilon) ? (1.0f / iz) : 0.0f;
    }
}

inline Mat3 Body::InvInertiaWorld() const {
    if (invMass == 0.0f) return {};
    const Mat3 R = RotationMatrix(orientation);
    return R * invInertiaLocal * Transpose(R);
}

inline AABB Body::ComputeAABB() const {
    if (shape == ShapeType::Sphere) {
        return {{position.x - radius, position.y - radius, position.z - radius}, {position.x + radius, position.y + radius, position.z + radius}};
    }
    if (shape == ShapeType::Capsule) {
        const Vec3 axis = Rotate(orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 top = position + axis * halfHeight;
        const Vec3 bottom = position - axis * halfHeight;
        return {{std::min(top.x, bottom.x) - radius, std::min(top.y, bottom.y) - radius, std::min(top.z, bottom.z) - radius},
                {std::max(top.x, bottom.x) + radius, std::max(top.y, bottom.y) + radius, std::max(top.z, bottom.z) + radius}};
    }
    if (shape == ShapeType::Plane) return {{-1e9f, -1e9f, -1e9f}, {1e9f, 1e9f, 1e9f}};

    const Vec3 ax = Rotate(orientation, {1.0f, 0.0f, 0.0f});
    const Vec3 ay = Rotate(orientation, {0.0f, 1.0f, 0.0f});
    const Vec3 az = Rotate(orientation, {0.0f, 0.0f, 1.0f});
    const Vec3 ext = {
        std::abs(ax.x) * halfExtents.x + std::abs(ay.x) * halfExtents.y + std::abs(az.x) * halfExtents.z,
        std::abs(ax.y) * halfExtents.x + std::abs(ay.y) * halfExtents.y + std::abs(az.y) * halfExtents.z,
        std::abs(ax.z) * halfExtents.x + std::abs(ay.z) * halfExtents.y + std::abs(az.z) * halfExtents.z,
    };
    return {{position.x - ext.x, position.y - ext.y, position.z - ext.z}, {position.x + ext.x, position.y + ext.y, position.z + ext.z}};
}

} // namespace minphys3d
