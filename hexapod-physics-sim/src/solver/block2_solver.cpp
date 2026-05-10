#include "minphys3d/solver/block2_solver.hpp"

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

    const Real normalDot = Dot(n0, n1);
    const Real k11 = input.invMassSum + Dot(ra0xn0, input.invIA * ra0xn0) + Dot(rb0xn0, input.invIB * rb0xn0);
    const Real k22 = input.invMassSum + Dot(ra1xn1, input.invIA * ra1xn1) + Dot(rb1xn1, input.invIB * rb1xn1);
    const Real k12 = input.invMassSum * normalDot + Dot(ra0xn0, input.invIA * ra1xn1) + Dot(rb0xn0, input.invIB * rb1xn1);
    const Real det = k11 * k22 - k12 * k12;
    result.conditionEstimate = det;

    if (k11 <= input.blockDiagonalMinimum || k22 <= input.blockDiagonalMinimum || std::abs(det) <= input.blockDeterminantEpsilon) {
        result.fallbackReason = BlockSolveFallbackCode::DegenerateMassMatrix;
        return result;
    }

    if (input.blockConditionEstimateMax > 0.0) {
        const Real matrixNorm = std::max(std::abs(k11) + std::abs(k12), std::abs(k12) + std::abs(k22));
        const Real invNorm = std::max(std::abs(k22) + std::abs(k12), std::abs(k12) + std::abs(k11)) / std::abs(det);
        const Real conditionEstimate = matrixNorm * invNorm;
        result.conditionEstimate = conditionEstimate;
        if (!std::isfinite(conditionEstimate) || conditionEstimate > input.blockConditionEstimateMax) {
            result.fallbackReason = BlockSolveFallbackCode::ConditionEstimateExceeded;
            return result;
        }
    }

    const Real old0 = std::max(input.contacts[0].oldImpulse, 0.0);
    const Real old1 = std::max(input.contacts[1].oldImpulse, 0.0);
    const Real q0 = input.contacts[0].rhs + k11 * old0 + k12 * old1;
    const Real q1 = input.contacts[1].rhs + k12 * old0 + k22 * old1;

    auto residualW = [&](Real l0, Real l1) {
        return std::array<Real, 2>{k11 * l0 + k12 * l1 - q0, k12 * l0 + k22 * l1 - q1};
    };

    constexpr Real lcpEpsilon = 1e-5;
    bool solved = false;
    Real new0 = old0;
    Real new1 = old1;

    const Real invDet = 1.0 / det;
    const Real both0 = (k22 * q0 - k12 * q1) * invDet;
    const Real both1 = (-k12 * q0 + k11 * q1) * invDet;
    if (both0 >= 0.0 && both1 >= 0.0) {
        const auto w = residualW(both0, both1);
        if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
            new0 = both0;
            new1 = both1;
            solved = true;
        }
    }
    if (!solved && k11 > 1e-8) {
        const auto w = residualW(std::max(0.0, q0 / k11), 0.0);
        if (w[1] >= -lcpEpsilon) {
            new0 = std::max(0.0, q0 / k11);
            new1 = 0.0;
            solved = true;
        }
    }
    if (!solved && k22 > 1e-8) {
        const auto w = residualW(0.0, std::max(0.0, q1 / k22));
        if (w[0] >= -lcpEpsilon) {
            new0 = 0.0;
            new1 = std::max(0.0, q1 / k22);
            solved = true;
        }
    }
    if (!solved) {
        const auto w = residualW(0.0, 0.0);
        if (w[0] >= -lcpEpsilon && w[1] >= -lcpEpsilon) {
            new0 = 0.0;
            new1 = 0.0;
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

    new0 = std::max(0.0, new0);
    new1 = std::max(0.0, new1);
    result.success = true;
    result.solvedImpulses = {new0, new1};
    result.impulseDeltas = {new0 - old0, new1 - old1};
    return result;
}

} // namespace minphys3d::solver_internal
