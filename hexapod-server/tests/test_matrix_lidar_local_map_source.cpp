#include "matrix_lidar_local_map_source.hpp"
#include "physics_sim_protocol.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool hasOccupiedNear(const LocalMapObservation& obs, double x_m, double y_m, double tol_m) {
    for (const LocalMapObservationSample& s : obs.samples) {
        if (s.state != LocalMapCellState::Occupied) {
            continue;
        }
        if (std::hypot(s.x_m - x_m, s.y_m - y_m) <= tol_m) {
            return true;
        }
    }
    return false;
}

} // namespace

int main() {
    MatrixLidarLocalMapObservationSource source{};

    RobotState est{};
    est.has_matrix_lidar = true;
    est.matrix_lidar.valid = true;
    est.matrix_lidar.model = static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::Matrix64x8);
    est.matrix_lidar.cols = 64;
    est.matrix_lidar.rows = 8;
    est.matrix_lidar.timestamp_us = TimePointUs{5000};
    est.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
    // Center beam, 2.0 m to a forward obstacle.
    est.matrix_lidar.ranges_mm[4 * 64 + 32] = 2000;

    const NavPose2d pose{0.0, 0.0, 0.0};
    const LocalMapObservation obs = source.collect(pose, est, TimePointUs{9000});

    if (!expect(obs.timestamp_us.value == 5000, "observation should use lidar frame timestamp")) {
        return EXIT_FAILURE;
    }
    if (!expect(!obs.samples.empty(), "center return should produce at least one occupied sample")) {
        return EXIT_FAILURE;
    }
    // Forward hit should lie near +X from the sensor mount (~0.055 m) at roughly 2 m.
    if (!expect(hasOccupiedNear(obs, 2.0, 0.0, 0.25), "hit should be near +X world from robot origin")) {
        return EXIT_FAILURE;
    }

    RobotState no_lidar{};
    const LocalMapObservation empty = source.collect(pose, no_lidar, TimePointUs{10'000});
    if (!expect(empty.samples.empty(), "without lidar data the observation should be empty")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
