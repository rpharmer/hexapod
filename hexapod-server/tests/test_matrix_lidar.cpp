#include "dummy_matrix_lidar.hpp"
#include "matrix_lidar_physics.hpp"
#include "physics_sim_protocol.hpp"
#include "types.hpp"

#include <iostream>

namespace {

bool expect(bool ok, const char* msg) {
    if (!ok) {
        std::cerr << "FAIL: " << msg << '\n';
    }
    return ok;
}

} // namespace

int main() {
    static_assert(physics_sim::kMatrixLidarMaxCells == MatrixLidarFrame::kMaxCells);

    physics_sim::StateResponse rsp{};
    rsp.matrix_lidar_valid = 1;
    rsp.matrix_lidar_model = physics_sim::MatrixLidarModel::Matrix64x8;
    rsp.matrix_lidar_cols = 64;
    rsp.matrix_lidar_rows = 8;
    for (std::size_t i = 0; i < 512; ++i) {
        rsp.matrix_lidar_ranges_mm[i] = static_cast<std::uint16_t>(1000 + i);
    }

    RobotState st{};
    matrixLidarFromPhysicsStateResponse(rsp, TimePointUs{99}, st);
    if (!expect(st.has_matrix_lidar, "decoder should set has_matrix_lidar")) {
        return 1;
    }
    if (!expect(st.matrix_lidar.valid, "decoder should set matrix_lidar.valid")) {
        return 1;
    }
    if (!expect(st.matrix_lidar.cols == 64 && st.matrix_lidar.rows == 8, "dims should match wire")) {
        return 1;
    }
    if (!expect(st.matrix_lidar.ranges_mm[0] == 1000 && st.matrix_lidar.ranges_mm[511] == 1511,
                "in-range cells should pass through decode")) {
        return 1;
    }
    if (!expect(st.matrix_lidar.timestamp_us.value == 99, "timestamp should propagate")) {
        return 1;
    }

    rsp.matrix_lidar_ranges_mm[0] = 7000;
    rsp.matrix_lidar_ranges_mm[1] = 50;
    matrixLidarFromPhysicsStateResponse(rsp, TimePointUs{100}, st);
    if (!expect(st.matrix_lidar.ranges_mm[0] == 5000 && st.matrix_lidar.ranges_mm[1] == 100,
                "ranges should clamp to model min/max mm")) {
        return 1;
    }

    rsp.matrix_lidar_model = static_cast<physics_sim::MatrixLidarModel>(99);
    rsp.matrix_lidar_ranges_mm[0] = 1000;
    rsp.matrix_lidar_ranges_mm[1] = 1001;
    matrixLidarFromPhysicsStateResponse(rsp, TimePointUs{101}, st);
    if (!expect(!st.has_matrix_lidar, "unknown model should clear has_matrix_lidar")) {
        return 1;
    }

    rsp.matrix_lidar_model = physics_sim::MatrixLidarModel::Matrix64x8;
    rsp.matrix_lidar_cols = 8;
    rsp.matrix_lidar_rows = 8;
    matrixLidarFromPhysicsStateResponse(rsp, TimePointUs{102}, st);
    if (!expect(!st.has_matrix_lidar, "wrong grid for model should clear has_matrix_lidar")) {
        return 1;
    }

    rsp.matrix_lidar_cols = 0;
    matrixLidarFromPhysicsStateResponse(rsp, TimePointUs{1}, st);
    if (!expect(!st.has_matrix_lidar, "invalid dims should clear has_matrix_lidar")) {
        return 1;
    }

    RobotState dummy_state{};
    DummyMatrixLidar dummy{};
    dummy.inject(dummy_state, TimePointUs{3});
    if (!expect(dummy_state.has_matrix_lidar && dummy_state.matrix_lidar.valid, "dummy should enable")) {
        return 1;
    }
    if (!expect(dummy_state.matrix_lidar.cols == 8 && dummy_state.matrix_lidar.rows == 8, "dummy is 8x8")) {
        return 1;
    }
    if (!expect(dummy_state.matrix_lidar.ranges_mm[0] == 250, "dummy corner range")) {
        return 1;
    }
    if (!expect(dummy_state.matrix_lidar.ranges_mm[63] <= 3500, "dummy last cell within VL53 max")) {
        return 1;
    }
    if (!expect(dummy_state.matrix_lidar.ranges_mm[64] == physics_sim::kMatrixLidarInvalidMm,
                "padding cells should be invalid")) {
        return 1;
    }

    return 0;
}
