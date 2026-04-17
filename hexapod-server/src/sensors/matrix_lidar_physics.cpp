#include "matrix_lidar_physics.hpp"

#include "matrix_lidar_geometry.hpp"

void matrixLidarFromPhysicsStateResponse(const physics_sim::StateResponse& rsp,
                                         TimePointUs timestamp_us,
                                         RobotState& out) {
    if (rsp.matrix_lidar_valid == 0) {
        out.has_matrix_lidar = false;
        out.matrix_lidar = MatrixLidarFrame{};
        return;
    }

    const auto model = rsp.matrix_lidar_model;
    if (model != physics_sim::MatrixLidarModel::Matrix64x8 &&
        model != physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        out.has_matrix_lidar = false;
        out.matrix_lidar = MatrixLidarFrame{};
        return;
    }

    const std::size_t cols = rsp.matrix_lidar_cols;
    const std::size_t rows = rsp.matrix_lidar_rows;
    const std::size_t count = cols * rows;
    if (cols == 0 || rows == 0 || count > MatrixLidarFrame::kMaxCells) {
        out.has_matrix_lidar = false;
        out.matrix_lidar = MatrixLidarFrame{};
        return;
    }

    if (!matrix_lidar_geom::expectedGridShape(model, cols, rows)) {
        out.has_matrix_lidar = false;
        out.matrix_lidar = MatrixLidarFrame{};
        return;
    }

    out.matrix_lidar.timestamp_us = timestamp_us;
    out.matrix_lidar.model = static_cast<std::uint8_t>(model);
    out.matrix_lidar.cols = rsp.matrix_lidar_cols;
    out.matrix_lidar.rows = rsp.matrix_lidar_rows;
    out.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
    for (std::size_t i = 0; i < count; ++i) {
        out.matrix_lidar.ranges_mm[i] = matrix_lidar_geom::clampRangeMm(model, rsp.matrix_lidar_ranges_mm[i]);
    }
    out.matrix_lidar.valid = true;
    out.has_matrix_lidar = true;
}
