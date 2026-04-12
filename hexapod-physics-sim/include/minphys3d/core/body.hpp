#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/math/mat3.hpp"

namespace minphys3d {

struct CompoundChild {
    ShapeType shape = ShapeType::Box;
    Vec3 localPosition{};
    Quat localOrientation{};
    float radius = 0.5f;
    float halfHeight = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
};

inline bool IsCompoundChildShapeSupported(ShapeType shape) {
    return IsConvexPrimitiveShape(shape);
}

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
    std::vector<CompoundChild> compoundChildren{};

    Vec3 planeNormal{0.0f, 1.0f, 0.0f};
    float planeOffset = 0.0f;

    float mass = 1.0f;
    float invMass = 1.0f;
    Mat3 invInertiaLocal = Mat3::Identity();
    float restitution = 0.2f;
    float staticFriction = 0.6f;
    float dynamicFriction = 0.4f;
    float linearDamping = 0.0f;
    float angularDamping = 0.0f;
    bool isStatic = false;
    bool isSleeping = false;
    int sleepCounter = 0;

    /// For asymmetric primitives (e.g. half-cylinder), offset from the **collision primitive origin** (JSON /
    /// authoring frame) to the **center of mass**, expressed in body-local axes. `position` is integrated as the
    /// world-space center of mass after `World::CreateBody` finalizes the primitive.
    Vec3 centerOfMassLocal{};

    void RecomputeMassProperties();
    Mat3 InvInertiaWorld() const;
    AABB ComputeAABB() const;
};

/// Solid half-cylinder with local y along the cylinder axis and material in z>=0 (flat cut at z=0).
/// Returns the vector from the primitive origin (mid-height, center of the flat diameter) to the uniform-density COM.
inline Vec3 HalfCylinderComOffsetFromShapeOriginLocal(float radius) {
    constexpr float kPi = 3.14159265f;
    return {0.0f, 0.0f, 4.0f * radius / (3.0f * kPi)};
}

/// World-space primitive frame origin used for collision geometry (equals `body.position` when `centerOfMassLocal` is zero).
inline Vec3 BodyWorldShapeOrigin(const Body& body) {
    const Quat q = Normalize(body.orientation);
    return body.position - Rotate(q, body.centerOfMassLocal);
}

inline Vec3 PrimitiveShapeBoundsHalfExtents(const Body& body) {
    switch (body.shape) {
        case ShapeType::Sphere:
            return {body.radius, body.radius, body.radius};
        case ShapeType::Capsule:
            return {body.radius, body.halfHeight + body.radius, body.radius};
        case ShapeType::Cylinder:
        case ShapeType::HalfCylinder:
            return {body.radius, body.halfHeight, body.radius};
        case ShapeType::Box:
            return body.halfExtents;
        case ShapeType::Plane:
            return {1e9f, 1e9f, 1e9f};
        case ShapeType::Compound:
            return body.halfExtents;
    }
    return body.halfExtents;
}

inline AABB MergeAABB(const AABB& a, const AABB& b) {
    return {
        {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)},
        {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)},
    };
}

inline AABB ComputePrimitiveAABB(
    ShapeType shape,
    const Vec3& position,
    const Quat& orientation,
    float radius,
    float halfHeight,
    const Vec3& halfExtents,
    const Vec3& planeNormal,
    float planeOffset) {
    if (shape == ShapeType::Sphere) {
        return {
            {position.x - radius, position.y - radius, position.z - radius},
            {position.x + radius, position.y + radius, position.z + radius},
        };
    }
    if (shape == ShapeType::Capsule) {
        const Vec3 axis = Rotate(orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 top = position + axis * halfHeight;
        const Vec3 bottom = position - axis * halfHeight;
        return {
            {std::min(top.x, bottom.x) - radius, std::min(top.y, bottom.y) - radius, std::min(top.z, bottom.z) - radius},
            {std::max(top.x, bottom.x) + radius, std::max(top.y, bottom.y) + radius, std::max(top.z, bottom.z) + radius},
        };
    }
    if (shape == ShapeType::Plane) {
        Vec3 n{};
        if (!TryNormalize(planeNormal, n)) {
            return {{-1e9f, -1e9f, -1e9f}, {1e9f, 1e9f, 1e9f}};
        }
        (void)planeOffset;
        return {{-1e9f, -1e9f, -1e9f}, {1e9f, 1e9f, 1e9f}};
    }

    Vec3 boundsHalfExtents = halfExtents;
    if (shape == ShapeType::Sphere) {
        boundsHalfExtents = {radius, radius, radius};
    } else if (shape == ShapeType::Capsule) {
        boundsHalfExtents = {radius, halfHeight + radius, radius};
    } else if (shape == ShapeType::Cylinder || shape == ShapeType::HalfCylinder) {
        boundsHalfExtents = {radius, halfHeight, radius};
    }
    const Vec3 ax = Rotate(orientation, {1.0f, 0.0f, 0.0f});
    const Vec3 ay = Rotate(orientation, {0.0f, 1.0f, 0.0f});
    const Vec3 az = Rotate(orientation, {0.0f, 0.0f, 1.0f});
    const Vec3 ext = {
        std::abs(ax.x) * boundsHalfExtents.x + std::abs(ay.x) * boundsHalfExtents.y + std::abs(az.x) * boundsHalfExtents.z,
        std::abs(ax.y) * boundsHalfExtents.x + std::abs(ay.y) * boundsHalfExtents.y + std::abs(az.y) * boundsHalfExtents.z,
        std::abs(ax.z) * boundsHalfExtents.x + std::abs(ay.z) * boundsHalfExtents.y + std::abs(az.z) * boundsHalfExtents.z,
    };
    return {
        {position.x - ext.x, position.y - ext.y, position.z - ext.z},
        {position.x + ext.x, position.y + ext.y, position.z + ext.z},
    };
}

inline Vec3 CompoundLocalBoundsHalfExtents(const Body& body) {
    bool hasSupportedChild = false;
    AABB merged{};
    for (const CompoundChild& child : body.compoundChildren) {
        if (!IsCompoundChildShapeSupported(child.shape)) {
            continue;
        }
        const AABB childBounds = ComputePrimitiveAABB(
            child.shape,
            child.localPosition,
            Normalize(child.localOrientation),
            child.radius,
            child.halfHeight,
            child.halfExtents,
            body.planeNormal,
            body.planeOffset);
        if (!hasSupportedChild) {
            merged = childBounds;
            hasSupportedChild = true;
        } else {
            merged = MergeAABB(merged, childBounds);
        }
    }

    if (!hasSupportedChild) {
        return body.halfExtents;
    }
    return 0.5f * (merged.max - merged.min);
}

inline Vec3 ShapeBoundsHalfExtents(const Body& body) {
    if (body.shape == ShapeType::Compound) {
        return CompoundLocalBoundsHalfExtents(body);
    }
    return PrimitiveShapeBoundsHalfExtents(body);
}

inline Mat3 DiagonalMat3(float ix, float iy, float iz) {
    Mat3 d{};
    d.m[0][0] = ix;
    d.m[1][1] = iy;
    d.m[2][2] = iz;
    return d;
}

// Convex child volume in the child's local frame (used for mass splitting on compounds).
inline float CompoundChildConvexVolume(const CompoundChild& c) {
    switch (c.shape) {
        case ShapeType::Box:
            return 8.0f * c.halfExtents.x * c.halfExtents.y * c.halfExtents.z;
        case ShapeType::Sphere:
            return (4.0f / 3.0f) * 3.14159265f * c.radius * c.radius * c.radius;
        case ShapeType::Capsule: {
            const float h = 2.0f * c.halfHeight;
            const float cylinder = 3.14159265f * c.radius * c.radius * h;
            const float caps = (4.0f / 3.0f) * 3.14159265f * c.radius * c.radius * c.radius;
            return cylinder + caps;
        }
        case ShapeType::Cylinder: {
            const float h = 2.0f * c.halfHeight;
            return 3.14159265f * c.radius * c.radius * h;
        }
        default:
            return 0.0f;
    }
}

// Principal moments (diagonal) about the child's center, expressed in the child's principal axes.
inline void CompoundChildPrincipalMoments(const CompoundChild& c, float childMass, float& ix, float& iy, float& iz) {
    switch (c.shape) {
        case ShapeType::Sphere: {
            const float i = 0.4f * childMass * c.radius * c.radius;
            ix = iy = iz = i;
            return;
        }
        case ShapeType::Box: {
            const float wx = c.halfExtents.x * 2.0f;
            const float wy = c.halfExtents.y * 2.0f;
            const float wz = c.halfExtents.z * 2.0f;
            ix = (childMass / 12.0f) * (wy * wy + wz * wz);
            iy = (childMass / 12.0f) * (wx * wx + wz * wz);
            iz = (childMass / 12.0f) * (wx * wx + wy * wy);
            return;
        }
        case ShapeType::Capsule: {
            const float capsuleHeight = c.halfHeight * 2.0f + 2.0f * c.radius;
            ix = 0.25f * childMass * c.radius * c.radius + (1.0f / 12.0f) * childMass * capsuleHeight * capsuleHeight;
            iy = 0.5f * childMass * c.radius * c.radius;
            iz = ix;
            return;
        }
        case ShapeType::Cylinder: {
            const float height = c.halfHeight * 2.0f;
            ix = (childMass / 12.0f) * (3.0f * c.radius * c.radius + height * height);
            iy = 0.5f * childMass * c.radius * c.radius;
            iz = ix;
            return;
        }
        default:
            ix = iy = iz = 0.0f;
            return;
    }
}

// Volume-weighted composite inertia about the compound's combined center of mass (body-local).
// Falls back to the caller when unsupported children or a singular tensor are encountered.
inline bool TryCompoundInvInertiaFromChildren(const Body& body, Mat3& invInertiaLocalOut) {
    if (body.shape != ShapeType::Compound) {
        return false;
    }

    float totalVolume = 0.0f;
    for (const CompoundChild& ch : body.compoundChildren) {
        if (!IsCompoundChildShapeSupported(ch.shape)) {
            return false;
        }
        if (ch.shape == ShapeType::HalfCylinder) {
            return false;
        }
        const float v = CompoundChildConvexVolume(ch);
        if (v <= kEpsilon) {
            return false;
        }
        totalVolume += v;
    }

    if (totalVolume <= kEpsilon || body.compoundChildren.empty()) {
        return false;
    }

    const float mass = body.mass;
    Vec3 weightedPos{};
    for (const CompoundChild& ch : body.compoundChildren) {
        const float v = CompoundChildConvexVolume(ch);
        const float mi = mass * (v / totalVolume);
        weightedPos += mi * ch.localPosition;
    }
    const Vec3 rCom = weightedPos / mass;

    Mat3 inertiaAboutOrigin{};
    for (const CompoundChild& ch : body.compoundChildren) {
        const float v = CompoundChildConvexVolume(ch);
        const float mi = mass * (v / totalVolume);
        float pix = 0.0f;
        float piy = 0.0f;
        float piz = 0.0f;
        CompoundChildPrincipalMoments(ch, mi, pix, piy, piz);
        const Mat3 R = RotationMatrix(Normalize(ch.localOrientation));
        const Mat3 inertiaChildCm = R * DiagonalMat3(pix, piy, piz) * Transpose(R);
        const Vec3& ri = ch.localPosition;
        const float rLenSq = Dot(ri, ri);
        inertiaAboutOrigin = inertiaAboutOrigin + inertiaChildCm + mi * (ScaleIdentity(rLenSq) - OuterProduct(ri, ri));
    }

    const float comLenSq = Dot(rCom, rCom);
    const Mat3 inertiaAboutCom = inertiaAboutOrigin - mass * (ScaleIdentity(comLenSq) - OuterProduct(rCom, rCom));

    Mat3 sym{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            sym.m[i][j] = 0.5f * (inertiaAboutCom.m[i][j] + inertiaAboutCom.m[j][i]);
        }
    }

    return InvertMat3(sym, invInertiaLocalOut);
}

inline void Body::RecomputeMassProperties() {
    centerOfMassLocal = {};
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
    } else if (shape == ShapeType::Cylinder) {
        const float height = halfHeight * 2.0f;
        const float ix = (mass / 12.0f) * (3.0f * radius * radius + height * height);
        const float iy = 0.5f * mass * radius * radius;
        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0f / iy) : 0.0f;
        invInertiaLocal.m[2][2] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
    } else if (shape == ShapeType::HalfCylinder) {
        centerOfMassLocal = HalfCylinderComOffsetFromShapeOriginLocal(radius);
        const float H = halfHeight;
        const float R = radius;
        const float zc = centerOfMassLocal.z;
        const float IxxO = mass * (H * H / 3.0f + R * R / 4.0f);
        const float IyyO = 0.5f * mass * R * R;
        const float IzzO = mass * (H * H / 3.0f + R * R / 4.0f);
        const float mzc2 = mass * zc * zc;
        const float Ixx = std::max(IxxO - mzc2, kEpsilon * mass);
        const float Iyy = std::max(IyyO - mzc2, kEpsilon * mass);
        const float Izz = std::max(IzzO - mzc2, kEpsilon * mass);
        invInertiaLocal.m[0][0] = 1.0f / Ixx;
        invInertiaLocal.m[1][1] = 1.0f / Iyy;
        invInertiaLocal.m[2][2] = 1.0f / Izz;
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
    } else if (shape == ShapeType::Compound) {
        if (!TryCompoundInvInertiaFromChildren(*this, invInertiaLocal)) {
            const Vec3 localHalfExtents = CompoundLocalBoundsHalfExtents(*this);
            const float wx = std::max(localHalfExtents.x * 2.0f, kEpsilon);
            const float wy = std::max(localHalfExtents.y * 2.0f, kEpsilon);
            const float wz = std::max(localHalfExtents.z * 2.0f, kEpsilon);

            const float ix = (mass / 12.0f) * (wy * wy + wz * wz);
            const float iy = (mass / 12.0f) * (wx * wx + wz * wz);
            const float iz = (mass / 12.0f) * (wx * wx + wy * wy);

            invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0f / ix) : 0.0f;
            invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0f / iy) : 0.0f;
            invInertiaLocal.m[2][2] = (iz > kEpsilon) ? (1.0f / iz) : 0.0f;
        }
    }
}

inline Mat3 Body::InvInertiaWorld() const {
    if (invMass == 0.0f) return {};
    const Mat3 R = RotationMatrix(orientation);
    return R * invInertiaLocal * Transpose(R);
}

inline AABB Body::ComputeAABB() const {
    if (shape != ShapeType::Compound) {
        const Vec3 primitivePosition = (shape == ShapeType::HalfCylinder) ? BodyWorldShapeOrigin(*this) : position;
        return ComputePrimitiveAABB(
            shape, primitivePosition, orientation, radius, halfHeight, halfExtents, planeNormal, planeOffset);
    }

    bool hasSupportedChild = false;
    AABB merged{};
    for (const CompoundChild& child : compoundChildren) {
        if (!IsCompoundChildShapeSupported(child.shape)) {
            continue;
        }

        const Vec3 childWorldPosition = position + Rotate(orientation, child.localPosition);
        const Quat childWorldOrientation = Normalize(orientation * child.localOrientation);
        const AABB childBounds = ComputePrimitiveAABB(
            child.shape,
            childWorldPosition,
            childWorldOrientation,
            child.radius,
            child.halfHeight,
            child.halfExtents,
            planeNormal,
            planeOffset);
        if (!hasSupportedChild) {
            merged = childBounds;
            hasSupportedChild = true;
        } else {
            merged = MergeAABB(merged, childBounds);
        }
    }

    if (!hasSupportedChild) {
        return ComputePrimitiveAABB(ShapeType::Box, position, orientation, radius, halfHeight, halfExtents, planeNormal, planeOffset);
    }
    return merged;
}

} // namespace minphys3d
