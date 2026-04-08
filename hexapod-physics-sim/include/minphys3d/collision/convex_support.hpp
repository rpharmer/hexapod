#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>

#include "minphys3d/math/quat.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

struct ConvexTopologyMetadata {
    std::optional<std::uint8_t> vertexId{};
    std::optional<std::uint8_t> edgeId{};
    std::optional<std::uint8_t> faceId{};
};

struct ConvexSupportPoint {
    Vec3 point{};
    ConvexTopologyMetadata topology{};
};

class ConvexSupport {
public:
    using SupportMapping = std::function<ConvexSupportPoint(const Vec3&)>;

    ConvexSupport() = default;
    ConvexSupport(Vec3 shapePosition, Quat shapeOrientation, SupportMapping supportMapping)
        : position_(shapePosition), orientation_(shapeOrientation), supportMapping_(std::move(supportMapping)) {}

    ConvexSupportPoint Support(const Vec3& direction) const {
        return supportMapping_ ? supportMapping_(direction) : ConvexSupportPoint{};
    }

    const Vec3& Position() const { return position_; }
    const Quat& Orientation() const { return orientation_; }
    bool IsValid() const { return static_cast<bool>(supportMapping_); }

private:
    Vec3 position_{};
    Quat orientation_{};
    SupportMapping supportMapping_{};
};

inline bool ConvexOverlapGJK(const ConvexSupport& a, const ConvexSupport& b, int maxIterations = 24) {
    if (!a.IsValid() || !b.IsValid()) {
        return false;
    }

    auto supportMinkowski = [&](const Vec3& direction) {
        const ConvexSupportPoint pa = a.Support(direction);
        const ConvexSupportPoint pb = b.Support(-direction);
        return pa.point - pb.point;
    };

    auto sameDirection = [](const Vec3& direction, const Vec3& ao) {
        return Dot(direction, ao) > 0.0f;
    };

    std::array<Vec3, 4> simplex{};
    int simplexSize = 0;
    Vec3 direction = b.Position() - a.Position();
    if (LengthSquared(direction) <= kEpsilon * kEpsilon) {
        direction = {1.0f, 0.0f, 0.0f};
    }

    simplex[simplexSize++] = supportMinkowski(direction);
    direction = -simplex[0];

    for (int i = 0; i < maxIterations; ++i) {
        const Vec3 newPoint = supportMinkowski(direction);
        if (Dot(newPoint, direction) <= 0.0f) {
            return false;
        }

        simplex[simplexSize++] = newPoint;

        if (simplexSize == 2) {
            const Vec3 a0 = simplex[1];
            const Vec3 b0 = simplex[0];
            const Vec3 ab = b0 - a0;
            const Vec3 ao = -a0;
            if (sameDirection(ab, ao)) {
                direction = Cross(Cross(ab, ao), ab);
                if (LengthSquared(direction) <= kEpsilon * kEpsilon) {
                    return true;
                }
            } else {
                simplex[0] = a0;
                simplexSize = 1;
                direction = ao;
            }
        } else if (simplexSize == 3) {
            const Vec3 a0 = simplex[2];
            const Vec3 b0 = simplex[1];
            const Vec3 c0 = simplex[0];
            const Vec3 ab = b0 - a0;
            const Vec3 ac = c0 - a0;
            const Vec3 ao = -a0;
            const Vec3 abc = Cross(ab, ac);
            if (LengthSquared(abc) <= kEpsilon * kEpsilon) {
                direction = Cross(ab, ao);
                if (LengthSquared(direction) <= kEpsilon * kEpsilon) {
                    return true;
                }
            }

            const Vec3 abPerp = Cross(abc, ab);
            if (sameDirection(abPerp, ao)) {
                simplex[0] = b0;
                simplex[1] = a0;
                simplexSize = 2;
                direction = Cross(Cross(ab, ao), ab);
                continue;
            }

            const Vec3 acPerp = Cross(ac, abc);
            if (sameDirection(acPerp, ao)) {
                simplex[0] = c0;
                simplex[1] = a0;
                simplexSize = 2;
                direction = Cross(Cross(ac, ao), ac);
                continue;
            }

            direction = sameDirection(abc, ao) ? abc : -abc;
            if (!sameDirection(abc, ao)) {
                simplex[0] = b0;
                simplex[1] = c0;
                simplex[2] = a0;
            }
        } else if (simplexSize == 4) {
            const Vec3 a0 = simplex[3];
            const Vec3 b0 = simplex[2];
            const Vec3 c0 = simplex[1];
            const Vec3 d0 = simplex[0];
            const Vec3 ao = -a0;

            const Vec3 abc = Cross(b0 - a0, c0 - a0);
            const Vec3 acd = Cross(c0 - a0, d0 - a0);
            const Vec3 adb = Cross(d0 - a0, b0 - a0);

            if (sameDirection(abc, ao)) {
                simplex[0] = c0;
                simplex[1] = b0;
                simplex[2] = a0;
                simplexSize = 3;
                direction = abc;
                continue;
            }
            if (sameDirection(acd, ao)) {
                simplex[0] = d0;
                simplex[1] = c0;
                simplex[2] = a0;
                simplexSize = 3;
                direction = acd;
                continue;
            }
            if (sameDirection(adb, ao)) {
                simplex[0] = b0;
                simplex[1] = d0;
                simplex[2] = a0;
                simplexSize = 3;
                direction = adb;
                continue;
            }
            return true;
        }

        if (LengthSquared(direction) <= kEpsilon * kEpsilon) {
            return true;
        }
    }

    return false;
}

} // namespace minphys3d
