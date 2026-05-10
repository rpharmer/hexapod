#pragma once

#include "minphys3d/collision/convex_support.hpp"
#include "minphys3d/narrowphase/gjk.hpp"

namespace minphys3d {

struct EpaSettings {
    int maxIterations = 40;
    Real duplicateEpsilon = 1e-6;
    Real faceEpsilon = 1e-7;
    Real convergenceEpsilon = 1e-4;
    int fallbackBisectionIterations = 16;
};

struct EpaPenetrationResult {
    bool valid = false;
    bool converged = false;
    bool usedFallback = false;
    bool reachedIterationLimit = false;
    int iterations = 0;
    Real depth = 0.0;
    Vec3 normal{1.0, 0.0, 0.0};
    Vec3 witnessA{};
    Vec3 witnessB{};
};

EpaPenetrationResult ComputePenetrationEPA(
    const ConvexSupport& a,
    const ConvexSupport& b,
    const GjkSimplex& simplex,
    const EpaSettings& settings = {});

} // namespace minphys3d
