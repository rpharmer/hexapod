#pragma once

#include "physics_sim_protocol.hpp"
#include "types.hpp"

#include <algorithm>

/** Test / bench matrix ToF LiDAR: writes a fixed 8×8 VL53-style grid into `RobotState`. */
struct DummyMatrixLidar {
    bool enabled{true};

    void inject(RobotState& st, TimePointUs timestamp_us) const {
        if (!enabled) {
            st.has_matrix_lidar = false;
            st.matrix_lidar = MatrixLidarFrame{};
            return;
        }
        st.matrix_lidar = MatrixLidarFrame{};
        st.matrix_lidar.timestamp_us = timestamp_us;
        st.matrix_lidar.model =
            static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::Vl53l7Cx8x8);
        st.matrix_lidar.cols = 8;
        st.matrix_lidar.rows = 8;
        for (std::size_t r = 0; r < 8; ++r) {
            for (std::size_t c = 0; c < 8; ++c) {
                const std::size_t i = r * 8 + c;
                // Simple ramp in mm within VL53 nominal span, quantized to 2 mm for tests.
                const int mm = 250 + static_cast<int>(i) * 14;
                st.matrix_lidar.ranges_mm[i] = static_cast<std::uint16_t>(std::min(mm, 3500));
            }
        }
        for (std::size_t i = 64; i < MatrixLidarFrame::kMaxCells; ++i) {
            st.matrix_lidar.ranges_mm[i] = 0xFFFF;
        }
        st.matrix_lidar.valid = true;
        st.has_matrix_lidar = true;
    }
};
