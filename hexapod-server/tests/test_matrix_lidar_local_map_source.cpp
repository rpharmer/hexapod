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

bool hasFreeNear(const LocalMapObservation& obs, double x_m, double y_m, double tol_m) {
    for (const LocalMapObservationSample& s : obs.samples) {
        if (s.state != LocalMapCellState::Free) {
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
    if (!expect(hasFreeNear(obs, 1.0, 0.0, 0.25),
                "center return should also clear free space along the ray")) {
        return EXIT_FAILURE;
    }
    // Forward hit should lie near +X from the sensor mount (~0.055 m) at roughly 2 m.
    if (!expect(hasOccupiedNear(obs, 2.0, 0.0, 0.25), "hit should be near +X world from robot origin")) {
        return EXIT_FAILURE;
    }
    bool hit_has_z = false;
    for (const LocalMapObservationSample& s : obs.samples) {
        if (s.state == LocalMapCellState::Occupied && std::isfinite(s.z_m)) {
            hit_has_z = true;
            break;
        }
    }
    if (!expect(hit_has_z, "forward obstacle hit should carry world z_m for elevation mapping")) {
        return EXIT_FAILURE;
    }

    est.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
    est.matrix_lidar.ranges_mm[4 * 64 + 32] = 250;
    const LocalMapObservation ground_like = source.collect(pose, est, TimePointUs{10'000});
    if (!expect(hasFreeNear(ground_like, 0.2, 0.0, 0.15),
                "ground-like returns should still contribute free-space clearing")) {
        return EXIT_FAILURE;
    }
    if (!expect(!hasOccupiedNear(ground_like, 0.2, 0.0, 0.15),
                "ground-like returns should not stamp occupied cells")) {
        return EXIT_FAILURE;
    }

    est.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
    const LocalMapObservation no_return = source.collect(pose, est, TimePointUs{11'000});
    if (!expect(hasFreeNear(no_return, 1.5, 0.0, 0.4),
                "no-return cells should still clear free space out to sensor max range")) {
        return EXIT_FAILURE;
    }
    if (!expect(!hasOccupiedNear(no_return, 2.0, 0.0, 0.4),
                "no-return cells should not fabricate occupied endpoints")) {
        return EXIT_FAILURE;
    }

    RobotState no_lidar{};
    const LocalMapObservation empty = source.collect(pose, no_lidar, TimePointUs{10'000});
    if (!expect(empty.samples.empty(), "without lidar data the observation should be empty")) {
        return EXIT_FAILURE;
    }
    if (!expect(empty.timestamp_us.isZero(), "without lidar data the observation should not refresh map freshness")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
