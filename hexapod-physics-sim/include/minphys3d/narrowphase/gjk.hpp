#pragma once

#include <array>
#include <cstdint>

#include "minphys3d/collision/convex_support.hpp"

namespace minphys3d {

struct GjkVertex {
    Vec3 pointA{};
    Vec3 pointB{};
    Vec3 point{};
};

struct GjkSimplex {
    std::array<GjkVertex, 4> vertices{};
    int size = 0;
};

struct GjkSettings {
    int maxIterations = 32;
    float supportEpsilon = 1e-6f;
    float distanceEpsilon = 1e-6f;
    float duplicateEpsilon = 1e-7f;
};

struct GjkDistanceResult {
    bool valid = false;
    bool intersecting = false;
    bool reachedIterationLimit = false;
    int iterations = 0;
    float distance = 0.0f;
    Vec3 closestA{};
    Vec3 closestB{};
    Vec3 separatingAxis{1.0f, 0.0f, 0.0f};
    GjkSimplex simplex{};
};

struct NarrowphaseCache {
    GjkSimplex simplex{};
    Vec3 separatingAxis{1.0f, 0.0f, 0.0f};
    bool hasSimplex = false;
    bool lastResultIntersecting = false;
};

GjkDistanceResult GjkDistance(
    const ConvexSupport& a,
    const ConvexSupport& b,
    const GjkSettings& settings = {},
    NarrowphaseCache* cache = nullptr);

inline bool GjkOverlap(
    const ConvexSupport& a,
    const ConvexSupport& b,
    const GjkSettings& settings = {},
    NarrowphaseCache* cache = nullptr) {
    return GjkDistance(a, b, settings, cache).intersecting;
}

} // namespace minphys3d
