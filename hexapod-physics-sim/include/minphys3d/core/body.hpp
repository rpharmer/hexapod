#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/math/mat3.hpp"

namespace minphys3d {

struct CompoundChild {
    ShapeType shape = ShapeType::Box;
    Vec3 localPosition{};
    Quat localOrientation{};
    Real radius = 0.5;
    Real halfHeight = 0.5;
    Vec3 halfExtents{0.5, 0.5, 0.5};
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

    Real radius = 0.5;
    Real halfHeight = 0.5;
    Vec3 halfExtents{0.5, 0.5, 0.5};
    std::vector<CompoundChild> compoundChildren{};

    Vec3 planeNormal{0.0, 1.0, 0.0};
    Real planeOffset = 0.0;

    Real mass = 1.0;
    Real invMass = 1.0;
    Mat3 invInertiaLocal = Mat3::Identity();
    Real restitution = 0.2;
    Real staticFriction = 0.6;
    Real dynamicFriction = 0.4;
    Real linearDamping = 0.0;
    Real angularDamping = 0.0;
    bool isStatic = false;
    bool isSleeping = false;
    bool isTerrainAttachment = false;
    int sleepCounter = 0;

    std::uint32_t collisionGroup = 0x0001;
    std::uint32_t collisionMask  = 0xFFFFFFFF;

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
inline Vec3 HalfCylinderComOffsetFromShapeOriginLocal(Real radius) {
    constexpr Real kPi = 3.14159265;
    return {0.0, 0.0, 4.0 * radius / (3.0 * kPi)};
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
            return {1e9, 1e9, 1e9};
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
    Real radius,
    Real halfHeight,
    const Vec3& halfExtents,
    const Vec3& planeNormal,
    Real planeOffset) {
    if (shape == ShapeType::Sphere) {
        return {
            {position.x - radius, position.y - radius, position.z - radius},
            {position.x + radius, position.y + radius, position.z + radius},
        };
    }
    if (shape == ShapeType::Capsule) {
        const Vec3 axis = Rotate(orientation, {0.0, 1.0, 0.0});
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
            return {{-1e9, -1e9, -1e9}, {1e9, 1e9, 1e9}};
        }
        (void)planeOffset;
        return {{-1e9, -1e9, -1e9}, {1e9, 1e9, 1e9}};
    }

    Vec3 boundsHalfExtents = halfExtents;
    if (shape == ShapeType::Sphere) {
        boundsHalfExtents = {radius, radius, radius};
    } else if (shape == ShapeType::Capsule) {
        boundsHalfExtents = {radius, halfHeight + radius, radius};
    } else if (shape == ShapeType::Cylinder || shape == ShapeType::HalfCylinder) {
        boundsHalfExtents = {radius, halfHeight, radius};
    }
    const Vec3 ax = Rotate(orientation, {1.0, 0.0, 0.0});
    const Vec3 ay = Rotate(orientation, {0.0, 1.0, 0.0});
    const Vec3 az = Rotate(orientation, {0.0, 0.0, 1.0});
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
    return 0.5 * (merged.max - merged.min);
}

inline Vec3 ShapeBoundsHalfExtents(const Body& body) {
    if (body.shape == ShapeType::Compound) {
        return CompoundLocalBoundsHalfExtents(body);
    }
    return PrimitiveShapeBoundsHalfExtents(body);
}

inline Mat3 DiagonalMat3(Real ix, Real iy, Real iz) {
    Mat3 d{};
    d.m[0][0] = ix;
    d.m[1][1] = iy;
    d.m[2][2] = iz;
    return d;
}

// Convex child volume in the child's local frame (used for mass splitting on compounds).
inline Real CompoundChildConvexVolume(const CompoundChild& c) {
    switch (c.shape) {
        case ShapeType::Box:
            return 8.0 * c.halfExtents.x * c.halfExtents.y * c.halfExtents.z;
        case ShapeType::Sphere:
            return (4.0 / 3.0) * 3.14159265 * c.radius * c.radius * c.radius;
        case ShapeType::Capsule: {
            const Real h = 2.0 * c.halfHeight;
            const Real cylinder = 3.14159265 * c.radius * c.radius * h;
            const Real caps = (4.0 / 3.0) * 3.14159265 * c.radius * c.radius * c.radius;
            return cylinder + caps;
        }
        case ShapeType::Cylinder: {
            const Real h = 2.0 * c.halfHeight;
            return 3.14159265 * c.radius * c.radius * h;
        }
        default:
            return 0.0;
    }
}

// Principal moments (diagonal) about the child's center, expressed in the child's principal axes.
inline void CompoundChildPrincipalMoments(const CompoundChild& c, Real childMass, Real& ix, Real& iy, Real& iz) {
    switch (c.shape) {
        case ShapeType::Sphere: {
            const Real i = 0.4 * childMass * c.radius * c.radius;
            ix = iy = iz = i;
            return;
        }
        case ShapeType::Box: {
            const Real wx = c.halfExtents.x * 2.0;
            const Real wy = c.halfExtents.y * 2.0;
            const Real wz = c.halfExtents.z * 2.0;
            ix = (childMass / 12.0) * (wy * wy + wz * wz);
            iy = (childMass / 12.0) * (wx * wx + wz * wz);
            iz = (childMass / 12.0) * (wx * wx + wy * wy);
            return;
        }
        case ShapeType::Capsule: {
            const Real capsuleHeight = c.halfHeight * 2.0 + 2.0 * c.radius;
            ix = 0.25 * childMass * c.radius * c.radius + (1.0 / 12.0) * childMass * capsuleHeight * capsuleHeight;
            iy = 0.5 * childMass * c.radius * c.radius;
            iz = ix;
            return;
        }
        case ShapeType::Cylinder: {
            const Real height = c.halfHeight * 2.0;
            ix = (childMass / 12.0) * (3.0 * c.radius * c.radius + height * height);
            iy = 0.5 * childMass * c.radius * c.radius;
            iz = ix;
            return;
        }
        default:
            ix = iy = iz = 0.0;
            return;
    }
}

// Volume-weighted composite inertia about the compound's combined center of mass (body-local).
// Falls back to the caller when unsupported children or a singular tensor are encountered.
inline bool TryCompoundInvInertiaFromChildren(const Body& body, Mat3& invInertiaLocalOut) {
    if (body.shape != ShapeType::Compound) {
        return false;
    }

    Real totalVolume = 0.0;
    for (const CompoundChild& ch : body.compoundChildren) {
        if (!IsCompoundChildShapeSupported(ch.shape)) {
            return false;
        }
        if (ch.shape == ShapeType::HalfCylinder) {
            return false;
        }
        const Real v = CompoundChildConvexVolume(ch);
        if (v <= kEpsilon) {
            return false;
        }
        totalVolume += v;
    }

    if (totalVolume <= kEpsilon || body.compoundChildren.empty()) {
        return false;
    }

    const Real mass = body.mass;
    Vec3 weightedPos{};
    for (const CompoundChild& ch : body.compoundChildren) {
        const Real v = CompoundChildConvexVolume(ch);
        const Real mi = mass * (v / totalVolume);
        weightedPos += mi * ch.localPosition;
    }
    const Vec3 rCom = weightedPos / mass;

    Mat3 inertiaAboutOrigin{};
    for (const CompoundChild& ch : body.compoundChildren) {
        const Real v = CompoundChildConvexVolume(ch);
        const Real mi = mass * (v / totalVolume);
        Real pix = 0.0;
        Real piy = 0.0;
        Real piz = 0.0;
        CompoundChildPrincipalMoments(ch, mi, pix, piy, piz);
        const Mat3 R = RotationMatrix(Normalize(ch.localOrientation));
        const Mat3 inertiaChildCm = R * DiagonalMat3(pix, piy, piz) * Transpose(R);
        const Vec3& ri = ch.localPosition;
        const Real rLenSq = Dot(ri, ri);
        inertiaAboutOrigin = inertiaAboutOrigin + inertiaChildCm + mi * (ScaleIdentity(rLenSq) - OuterProduct(ri, ri));
    }

    const Real comLenSq = Dot(rCom, rCom);
    const Mat3 inertiaAboutCom = inertiaAboutOrigin - mass * (ScaleIdentity(comLenSq) - OuterProduct(rCom, rCom));

    Mat3 sym{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            sym.m[i][j] = 0.5 * (inertiaAboutCom.m[i][j] + inertiaAboutCom.m[j][i]);
        }
    }

    return InvertMat3(sym, invInertiaLocalOut);
}

inline void Body::RecomputeMassProperties() {
    centerOfMassLocal = {};
    if (shape == ShapeType::Plane) isStatic = true;

    if (isStatic || mass <= kEpsilon) {
        mass = std::numeric_limits<float>::infinity();
        invMass = 0.0;
        invInertiaLocal = {};
        isStatic = true;
        isSleeping = false;
        return;
    }

    invMass = 1.0 / mass;
    invInertiaLocal = {};

    if (shape == ShapeType::Sphere) {
        const Real inertia = 0.4 * mass * radius * radius;
        const Real invI = (inertia > kEpsilon) ? (1.0 / inertia) : 0.0;
        invInertiaLocal.m[0][0] = invI;
        invInertiaLocal.m[1][1] = invI;
        invInertiaLocal.m[2][2] = invI;
    } else if (shape == ShapeType::Capsule) {
        const Real capsuleHeight = halfHeight * 2.0 + 2.0 * radius;
        const Real ix = 0.25 * mass * radius * radius + (1.0 / 12.0) * mass * capsuleHeight * capsuleHeight;
        const Real iy = 0.5 * mass * radius * radius;
        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0 / iy) : 0.0;
        invInertiaLocal.m[2][2] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
    } else if (shape == ShapeType::Cylinder) {
        const Real height = halfHeight * 2.0;
        const Real ix = (mass / 12.0) * (3.0 * radius * radius + height * height);
        const Real iy = 0.5 * mass * radius * radius;
        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0 / iy) : 0.0;
        invInertiaLocal.m[2][2] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
    } else if (shape == ShapeType::HalfCylinder) {
        centerOfMassLocal = HalfCylinderComOffsetFromShapeOriginLocal(radius);
        const Real H = halfHeight;
        const Real R = radius;
        const Real zc = centerOfMassLocal.z;
        const Real IxxO = mass * (H * H / 3.0 + R * R / 4.0);
        const Real IyyO = 0.5 * mass * R * R;
        const Real IzzO = mass * (H * H / 3.0 + R * R / 4.0);
        const Real mzc2 = mass * zc * zc;
        const Real Ixx = std::max(IxxO - mzc2, kEpsilon * mass);
        const Real Iyy = std::max(IyyO - mzc2, kEpsilon * mass);
        const Real Izz = std::max(IzzO - mzc2, kEpsilon * mass);
        invInertiaLocal.m[0][0] = 1.0 / Ixx;
        invInertiaLocal.m[1][1] = 1.0 / Iyy;
        invInertiaLocal.m[2][2] = 1.0 / Izz;
    } else if (shape == ShapeType::Box) {
        const Real wx = halfExtents.x * 2.0;
        const Real wy = halfExtents.y * 2.0;
        const Real wz = halfExtents.z * 2.0;

        const Real ix = (mass / 12.0) * (wy * wy + wz * wz);
        const Real iy = (mass / 12.0) * (wx * wx + wz * wz);
        const Real iz = (mass / 12.0) * (wx * wx + wy * wy);

        invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
        invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0 / iy) : 0.0;
        invInertiaLocal.m[2][2] = (iz > kEpsilon) ? (1.0 / iz) : 0.0;
    } else if (shape == ShapeType::Compound) {
        if (!TryCompoundInvInertiaFromChildren(*this, invInertiaLocal)) {
            const Vec3 localHalfExtents = CompoundLocalBoundsHalfExtents(*this);
            const Real wx = std::max(localHalfExtents.x * 2.0, kEpsilon);
            const Real wy = std::max(localHalfExtents.y * 2.0, kEpsilon);
            const Real wz = std::max(localHalfExtents.z * 2.0, kEpsilon);

            const Real ix = (mass / 12.0) * (wy * wy + wz * wz);
            const Real iy = (mass / 12.0) * (wx * wx + wz * wz);
            const Real iz = (mass / 12.0) * (wx * wx + wy * wy);

            invInertiaLocal.m[0][0] = (ix > kEpsilon) ? (1.0 / ix) : 0.0;
            invInertiaLocal.m[1][1] = (iy > kEpsilon) ? (1.0 / iy) : 0.0;
            invInertiaLocal.m[2][2] = (iz > kEpsilon) ? (1.0 / iz) : 0.0;
        }
    }
}

inline Mat3 Body::InvInertiaWorld() const {
    if (invMass == 0.0) return {};
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
