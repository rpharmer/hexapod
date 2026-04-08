#include "block4_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minphys3d::solver_internal {

Block4SolveResult SolveBlock4ProjectedGaussSeidel(const Block4SolveInput& input) {
    Block4SolveResult result{};
    result.fallbackReason = BlockSolveFallbackCode::None;
    result.conditionEstimate = std::numeric_limits<float>::quiet_NaN();

    Vec3 centroid{0.0f, 0.0f, 0.0f};
    for (const auto& c : input.contacts) centroid += c.point;
    centroid = centroid / 4.0f;
    float spreadSq = 0.0f;
    float areaProxy = 0.0f;
    for (int i = 0; i < 4; ++i) {
        spreadSq = std::max(spreadSq, LengthSquared(input.contacts[i].point - centroid));
        for (int j = i + 1; j < 4; ++j) {
            areaProxy = std::max(areaProxy, LengthSquared(Cross(input.contacts[i].point - centroid, input.contacts[j].point - centroid)));
        }
    }
    if (spreadSq < input.face4MinSpreadSq || areaProxy < input.face4MinArea) {
        result.fallbackReason = BlockSolveFallbackCode::QualityGate;
        return result;
    }

    std::array<std::array<float, 4>, 4> K{};
    std::array<float, 4> diagonal{};
    std::array<float, 4> oldLambda{};
    std::array<float, 4> rhs{};
    std::array<Vec3, 4> raCrossN{};
    std::array<Vec3, 4> rbCrossN{};

    for (int i = 0; i < 4; ++i) {
        raCrossN[i] = Cross(input.contacts[i].ra, input.contacts[i].normal);
        rbCrossN[i] = Cross(input.contacts[i].rb, input.contacts[i].normal);
        oldLambda[i] = std::max(input.contacts[i].oldImpulse, 0.0f);
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            K[i][j] = input.invMassSum * Dot(input.contacts[i].normal, input.contacts[j].normal)
                + Dot(raCrossN[i], input.invIA * raCrossN[j])
                + Dot(rbCrossN[i], input.invIB * rbCrossN[j]);
            if (!std::isfinite(K[i][j])) {
                result.fallbackReason = BlockSolveFallbackCode::NonFiniteResult;
                return result;
            }
        }
        diagonal[i] = K[i][i];
        if (diagonal[i] <= input.blockDiagonalMinimum) {
            result.fallbackReason = BlockSolveFallbackCode::DegenerateMassMatrix;
            return result;
        }
        for (int j = i + 1; j < 4; ++j) {
            const float scale = std::max(1.0f, std::max(std::abs(K[i][j]), std::abs(K[j][i])));
            if (std::abs(K[i][j] - K[j][i]) > input.symmetryTolerance * scale) {
                result.fallbackReason = BlockSolveFallbackCode::DegenerateMassMatrix;
                return result;
            }
        }
    }

    float matrixNormInf = 0.0f;
    float augmented[4][8]{};
    for (int i = 0; i < 4; ++i) {
        float rowSum = 0.0f;
        for (int j = 0; j < 4; ++j) {
            rowSum += std::abs(K[i][j]);
            augmented[i][j] = K[i][j];
        }
        matrixNormInf = std::max(matrixNormInf, rowSum);
        augmented[i][4 + i] = 1.0f;
    }

    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        float pivotAbs = std::abs(augmented[pivot][col]);
        for (int row = col + 1; row < 4; ++row) {
            if (std::abs(augmented[row][col]) > pivotAbs) {
                pivot = row;
                pivotAbs = std::abs(augmented[row][col]);
            }
        }
        if (pivotAbs <= input.blockDiagonalMinimum || !std::isfinite(pivotAbs)) {
            result.fallbackReason = BlockSolveFallbackCode::DegenerateMassMatrix;
            return result;
        }
        if (pivot != col) {
            for (int k = 0; k < 8; ++k) std::swap(augmented[col][k], augmented[pivot][k]);
        }
        const float invPivot = 1.0f / augmented[col][col];
        for (int k = 0; k < 8; ++k) augmented[col][k] *= invPivot;
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            const float factor = augmented[row][col];
            for (int k = 0; k < 8; ++k) augmented[row][k] -= factor * augmented[col][k];
        }
    }

    float inverseNormInf = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float rowSum = 0.0f;
        for (int j = 0; j < 4; ++j) {
            const float v = augmented[i][4 + j];
            if (!std::isfinite(v)) {
                result.fallbackReason = BlockSolveFallbackCode::NonFiniteResult;
                return result;
            }
            rowSum += std::abs(v);
        }
        inverseNormInf = std::max(inverseNormInf, rowSum);
    }

    result.conditionEstimate = matrixNormInf > 0.0f ? matrixNormInf * inverseNormInf : std::numeric_limits<float>::infinity();
    const float conditionCap = input.face4ConditionEstimateMax > 0.0f ? input.face4ConditionEstimateMax : input.blockConditionEstimateMax;
    if (conditionCap > 0.0f && (!std::isfinite(result.conditionEstimate) || result.conditionEstimate > conditionCap)) {
        result.fallbackReason = BlockSolveFallbackCode::ConditionEstimateExceeded;
        return result;
    }

    for (int i = 0; i < 4; ++i) {
        float shifted = input.contacts[i].rhs;
        for (int j = 0; j < 4; ++j) shifted += K[i][j] * oldLambda[j];
        if (!std::isfinite(shifted)) {
            result.fallbackReason = BlockSolveFallbackCode::NonFiniteResult;
            return result;
        }
        rhs[i] = shifted;
    }

    std::array<float, 4> lambda = oldLambda;
    for (int iter = 0; iter < std::max(input.face4Iterations, 1); ++iter) {
        float maxDelta = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float sum = 0.0f;
            for (int j = 0; j < 4; ++j) if (i != j) sum += K[i][j] * lambda[j];
            const float nv = std::max(0.0f, (rhs[i] - sum) / diagonal[i]);
            maxDelta = std::max(maxDelta, std::abs(nv - lambda[i]));
            lambda[i] = nv;
        }
        if (maxDelta <= input.face4ProjectedGaussSeidelEpsilon) break;
    }

    float residual = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float wi = -rhs[i];
        for (int j = 0; j < 4; ++j) wi += K[i][j] * lambda[j];
        if (!std::isfinite(wi) || !std::isfinite(lambda[i])) {
            result.fallbackReason = BlockSolveFallbackCode::NonFiniteResult;
            return result;
        }
        residual = std::max(residual, std::max(std::max(0.0f, -wi), std::abs(lambda[i] * wi)));
    }
    const float residualThreshold = std::max(5e-4f, 20.0f * input.face4ProjectedGaussSeidelEpsilon);
    if (residual > residualThreshold) {
        result.fallbackReason = (conditionCap > 0.0f && std::isfinite(result.conditionEstimate) && result.conditionEstimate > 0.5f * conditionCap)
            ? BlockSolveFallbackCode::ConditionEstimateExceeded
            : BlockSolveFallbackCode::LcpFailure;
        return result;
    }

    result.success = true;
    result.solvedImpulses = lambda;
    for (int i = 0; i < 4; ++i) result.impulseDeltas[i] = lambda[i] - oldLambda[i];
    return result;
}

} // namespace minphys3d::solver_internal
