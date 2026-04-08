#include "block2_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minphys3d::solver_internal {

Block2SolveResult SolveBlock2NormalLcp(const Block2SolveInput& input) {
    Block2SolveResult result{};
    result.fallbackReason = BlockSolveFallbackCode::None;
    result.conditionEstimate = std::numeric_limits<float>::quiet_NaN();

    const Vec3& n0 = input.contacts[0].normal;
    const Vec3& n1 = input.contacts[1].normal;
    const Vec3 ra0xn0 = Cross(input.contacts[0].ra, n0);
    const Vec3 rb0xn0 = Cross(input.contacts[0].rb, n0);
    const Vec3 ra1xn1 = Cross(input.contacts[1].ra, n1);
    const Vec3 rb1xn1 = Cross(input.contacts[1].rb, n1);

    const float normalDot = Dot(n0, n1);
    const float k11 = input.invMassSum + Dot(ra0xn0, input.invIA * ra0xn0) + Dot(rb0xn0, input.invIB * rb0xn0);
    const float k22 = input.invMassSum + Dot(ra1xn1, input.invIA * ra1xn1) + Dot(rb1xn1, input.invIB * rb1xn1);
    const float k12 = input.invMassSum * normalDot + Dot(ra0xn0, input.invIA * ra1xn1) + Dot(rb0xn0, input.invIB * rb1xn1);
    const float det = k11 * k22 - k12 * k12;
    result.conditionEstimate = det;

    if (k11 <= input.blockDiagonalMinimum || k22 <= input.blockDiagonalMinimum || std::abs(det) <= input.blockDeterminantEpsilon) {
        result.fallbackReason = BlockSolveFallbackCode::DegenerateMassMatrix;
        return result;
    }

    if (input.blockConditionEstimateMax > 0.0f) {
        const float matrixNorm = std::max(std::abs(k11) + std::abs(k12), std::abs(k12) + std::abs(k22));
        const float invNorm = std::max(std::abs(k22) + std::abs(k12), std::abs(k12) + std::abs(k11)) / std::abs(det);
        const float conditionEstimate = matrixNorm * invNorm;
        result.conditionEstimate = conditionEstimate;
        if (!std::isfinite(conditionEstimate) || conditionEstimate > input.blockConditionEstimateMax) {
            result.fallbackReason = BlockSolveFallbackCode::ConditionEstimateExceeded;
            return result;
        }
    }

    const float old0 = std::max(input.contacts[0].oldImpulse, 0.0f);
    const float old1 = std::max(input.contacts[1].oldImpulse, 0.0f);
    const float q0 = input.contacts[0].rhs + k11 * old0 + k12 * old1;
    const float q1 = input.contacts[1].rhs + k12 * old0 + k22 * old1;

    auto residualW = [&](float l0, float l1) {
        return std::array<float, 2>{k11 * l0 + k12 * l1 - q0, k12 * l0 + k22 * l1 - q1};
    };

    constexpr float lcpEpsilon = 1e-5f;
    bool solved = false;
    float new0 = old0;
    float new1 = old1;

    const float invDet = 1.0f / det;
    const float both0 = (k22 * q0 - k12 * q1) * invDet;
    const float both1 = (-k12 * q0 + k11 * q1) * invDet;
    if (both0 >= 0.0f && both1 >= 0.0f) {
        const auto w = residualW(both0, both1);
        if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
            new0 = both0;
            new1 = both1;
            solved = true;
        }
    }
    if (!solved && k11 > 1e-8f) {
        const auto w = residualW(std::max(0.0f, q0 / k11), 0.0f);
        if (w[1] >= -lcpEpsilon) {
            new0 = std::max(0.0f, q0 / k11);
            new1 = 0.0f;
            solved = true;
        }
    }
    if (!solved && k22 > 1e-8f) {
        const auto w = residualW(0.0f, std::max(0.0f, q1 / k22));
        if (w[0] >= -lcpEpsilon) {
            new0 = 0.0f;
            new1 = std::max(0.0f, q1 / k22);
            solved = true;
        }
    }
    if (!solved) {
        const auto w = residualW(0.0f, 0.0f);
        if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
            new0 = 0.0f;
            new1 = 0.0f;
            solved = true;
        }
    }

    if (!solved) {
        result.fallbackReason = BlockSolveFallbackCode::LcpFailure;
        return result;
    }
    if (!std::isfinite(new0) || !std::isfinite(new1)) {
        result.fallbackReason = BlockSolveFallbackCode::NonFiniteResult;
        return result;
    }

    new0 = std::max(0.0f, new0);
    new1 = std::max(0.0f, new1);
    result.success = true;
    result.solvedImpulses = {new0, new1};
    result.impulseDeltas = {new0 - old0, new1 - old1};
    return result;
}

} // namespace minphys3d::solver_internal
