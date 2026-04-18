#pragma once

#include "physics_sim_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

/// Shared matrix ToF LiDAR geometry and range limits (sim, server decode, navigation projection).
namespace matrix_lidar_geom {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;

inline constexpr double kMatrix64x8FovHRad = 120.0 * kDegToRad;
inline constexpr double kMatrix64x8FovVRad = 20.0 * kDegToRad;
inline constexpr double kVl53FovRad = 60.0 * kDegToRad;

/// Downward pitch of the optical axis from body +X (server body: +X forward, +Y left, +Z up).
inline constexpr double kOpticalAxisPitchDownRad = 8.0 * kDegToRad;

/// Nominal sensor origin in **server body** frame (metres): +X forward, +Y left, +Z up.
inline constexpr double kSensorOffsetForwardM = 0.055;
inline constexpr double kSensorOffsetLeftM = 0.0;
inline constexpr double kSensorOffsetUpM = 0.035;

/// Same physical offset in **minphys3d** chassis body axes (+Y up, forward local -Z): maps server
/// (+X forward, +Y left, +Z up) to minphys body (X_m, Y_m, Z_m) = (+Y_l, +Z_u, -X_f).
inline constexpr double kSensorOffsetMinphysBodyXM = kSensorOffsetLeftM;
inline constexpr double kSensorOffsetMinphysBodyYM = kSensorOffsetUpM;
inline constexpr double kSensorOffsetMinphysBodyZM = -kSensorOffsetForwardM;

inline constexpr std::uint16_t kMinRangeMatrix64x8Mm = 100;
inline constexpr std::uint16_t kMaxRangeMatrix64x8Mm = 5000;
inline constexpr std::uint16_t kMinRangeVl53Mm = 20;
inline constexpr std::uint16_t kMaxRangeVl53Mm = 3500;

inline void fovForModel(const physics_sim::MatrixLidarModel model, double& fov_h_rad, double& fov_v_rad) {
    if (model == physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        fov_h_rad = kVl53FovRad;
        fov_v_rad = kVl53FovRad;
        return;
    }
    fov_h_rad = kMatrix64x8FovHRad;
    fov_v_rad = kMatrix64x8FovVRad;
}

/// Normalised grid cell → azimuth / elevation (radians) about the optical frame used in sim and server.
inline void cellAzElRad(const int row,
                         const int rows,
                         const int col,
                         const int cols,
                         const double fov_h_rad,
                         const double fov_v_rad,
                         double& out_az_rad,
                         double& out_el_rad) {
    const double v = (static_cast<double>(row) + 0.5) / static_cast<double>(rows);
    const double u = (static_cast<double>(col) + 0.5) / static_cast<double>(cols);
    out_el_rad = (-0.5 + v) * fov_v_rad;
    out_az_rad = (-0.5 + u) * fov_h_rad;
}

inline bool expectedGridShape(const physics_sim::MatrixLidarModel model,
                               const std::size_t cols,
                               const std::size_t rows) {
    if (model == physics_sim::MatrixLidarModel::Matrix64x8) {
        return cols == 64 && rows == 8;
    }
    if (model == physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        return cols == 8 && rows == 8;
    }
    return false;
}

inline std::uint16_t maxRangeMmForModel(const physics_sim::MatrixLidarModel model) {
    if (model == physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        return kMaxRangeVl53Mm;
    }
    return kMaxRangeMatrix64x8Mm;
}

/// Clamp a wired range into the model datasheet span; invalid marker is preserved.
inline std::uint16_t clampRangeMm(const physics_sim::MatrixLidarModel model, const std::uint16_t mm) {
    if (mm == physics_sim::kMatrixLidarInvalidMm) {
        return mm;
    }
    std::uint16_t lo = kMinRangeMatrix64x8Mm;
    std::uint16_t hi = kMaxRangeMatrix64x8Mm;
    if (model == physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        lo = kMinRangeVl53Mm;
        hi = kMaxRangeVl53Mm;
    }
    return static_cast<std::uint16_t>(std::clamp(static_cast<int>(mm), static_cast<int>(lo), static_cast<int>(hi)));
}

} // namespace matrix_lidar_geom
