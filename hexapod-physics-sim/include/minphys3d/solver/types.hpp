#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

constexpr float kSleepLinearThreshold = 0.05f;
constexpr float kSleepAngularThreshold = 0.05f;
constexpr int kSleepFramesThreshold = 120;
constexpr float kMaxSubstepDistanceFactor = 0.5f;

struct ContactSolverConfig {
    float bounceVelocityThreshold = 0.0f;
    float restitutionSuppressionSpeed = 0.0f;
    float staticToDynamicTransitionSpeed = 0.0f;
    float penetrationSlop = 0.01f;
    float penetrationBiasFactor = 0.0f;
    float positionalCorrectionPercent = 0.8f;
    bool useSplitImpulse = false;
    float splitImpulseCorrectionFactor = 0.8f;
    float blockDeterminantEpsilon = 1e-8f;
    float blockDiagonalMinimum = 1e-6f;
    float blockConditionEstimateMax = 0.0f;
};

struct Contact {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    Vec3 point{};
    float penetration = 0.0f;
    float normalImpulseSum = 0.0f;
    float tangentImpulseSum = 0.0f;
    std::uint64_t key = 0;
};

struct Manifold {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{};
    std::vector<Contact> contacts;
    std::array<float, 2> blockNormalImpulseSum{0.0f, 0.0f};
    std::array<std::uint64_t, 2> blockContactKeys{0u, 0u};
    std::array<bool, 2> blockSlotValid{false, false};

    std::uint64_t pairKey() const {
        const std::uint32_t lo = std::min(a, b);
        const std::uint32_t hi = std::max(a, b);
        return (static_cast<std::uint64_t>(lo) << 32) | hi;
    }
};

struct Island {
    std::vector<std::uint32_t> bodies;
    std::vector<std::size_t> manifolds;
    std::vector<std::size_t> joints;
    std::vector<std::size_t> hinges;
};

} // namespace minphys3d
