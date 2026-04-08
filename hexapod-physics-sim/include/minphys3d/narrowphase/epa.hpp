#pragma once

#include "minphys3d/collision/convex_support.hpp"
#include "minphys3d/narrowphase/gjk.hpp"

namespace minphys3d {

struct EpaSettings {
    int maxIterations = 40;
    float duplicateEpsilon = 1e-6f;
    float faceEpsilon = 1e-7f;
    float convergenceEpsilon = 1e-4f;
    int fallbackBisectionIterations = 16;
};

struct EpaPenetrationResult {
    bool valid = false;
    bool converged = false;
    bool usedFallback = false;
    bool reachedIterationLimit = false;
    int iterations = 0;
    float depth = 0.0f;
    Vec3 normal{1.0f, 0.0f, 0.0f};
    Vec3 witnessA{};
    Vec3 witnessB{};
};

EpaPenetrationResult ComputePenetrationEPA(
    const ConvexSupport& a,
    const ConvexSupport& b,
    const GjkSimplex& simplex,
    const EpaSettings& settings = {});

} // namespace minphys3d
