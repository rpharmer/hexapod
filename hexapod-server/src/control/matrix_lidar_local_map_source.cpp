#include "matrix_lidar_local_map_source.hpp"

#include "math_types.hpp"
#include "matrix_lidar_geometry.hpp"
#include "physics_sim_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace {

constexpr double kRaySampleSpacingM = 0.05;
constexpr double kGroundHitHeightToleranceM = 0.05;
constexpr double kGroundExpectedRangeAbsToleranceM = 0.06;
constexpr double kGroundExpectedRangeRelTolerance = 0.12;

Vec3 normalizeVec3(const Vec3& v) {
    const double n = vecNorm(v);
    if (n <= 1.0e-12) {
        return Vec3{1.0, 0.0, 0.0};
    }
    return v * (1.0 / n);
}

void fillOpticalBasis(Vec3& out_optical_axis, Vec3& out_optical_right, Vec3& out_optical_up) {
    const Vec3 forward{1.0, 0.0, 0.0};
    const Vec3 up{0.0, 0.0, 1.0};
    const Vec3 optical_axis =
        normalizeVec3(forward * std::cos(matrix_lidar_geom::kOpticalAxisPitchDownRad) -
                      up * std::sin(matrix_lidar_geom::kOpticalAxisPitchDownRad));
    const Vec3 optical_right = normalizeVec3(cross(up, optical_axis));
    const Vec3 optical_up = normalizeVec3(cross(optical_axis, optical_right));
    out_optical_axis = optical_axis;
    out_optical_right = optical_right;
    out_optical_up = optical_up;
}

Vec3 cellDirectionBody(const Vec3& optical_axis,
                       const Vec3& optical_right,
                       const Vec3& optical_up,
                       const int row,
                       const int rows,
                       const int col,
                       const int cols,
                       const double fov_h_rad,
                       const double fov_v_rad) {
    double az = 0.0;
    double el = 0.0;
    matrix_lidar_geom::cellAzElRad(row, rows, col, cols, fov_h_rad, fov_v_rad, az, el);
    const double c_el = std::cos(el);
    return normalizeVec3(optical_axis * (c_el * std::cos(az)) + optical_right * (c_el * std::sin(az)) +
                         optical_up * std::sin(el));
}

bool finitePose(const NavPose2d& pose) {
    return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) && std::isfinite(pose.yaw_rad);
}

void appendRaySamples(LocalMapObservation& out,
                      const double sensor_x,
                      const double sensor_y,
                      const double sensor_z_world,
                      const Vec3& dir_w,
                      const double clear_range_m,
                      const bool has_hit,
                      const double hit_range_m,
                      const double hit_z_world_m,
                      const bool stamp_ground_z_on_free_samples) {
    if (!std::isfinite(clear_range_m) || clear_range_m <= 0.0) {
        return;
    }

    const double free_end_m =
        has_hit ? std::max(0.0, hit_range_m - (0.5 * kRaySampleSpacingM)) : clear_range_m;
    for (double distance_m = kRaySampleSpacingM; distance_m <= free_end_m; distance_m += kRaySampleSpacingM) {
        const double sample_x = sensor_x + distance_m * dir_w.x;
        const double sample_y = sensor_y + distance_m * dir_w.y;
        if (!std::isfinite(sample_x) || !std::isfinite(sample_y)) {
            continue;
        }
        LocalMapObservationSample sample{sample_x, sample_y, LocalMapCellState::Free};
        if (stamp_ground_z_on_free_samples && std::isfinite(sensor_z_world) && std::isfinite(dir_w.z)) {
            sample.ground_z_m = sensor_z_world + distance_m * dir_w.z;
        }
        out.samples.push_back(sample);
    }

    if (!has_hit) {
        return;
    }

    const double hit_x = sensor_x + hit_range_m * dir_w.x;
    const double hit_y = sensor_y + hit_range_m * dir_w.y;
    if (!std::isfinite(hit_x) || !std::isfinite(hit_y)) {
        return;
    }
    LocalMapObservationSample hit{hit_x, hit_y, LocalMapCellState::Occupied};
    if (std::isfinite(hit_z_world_m)) {
        hit.z_m = hit_z_world_m;
    }
    out.samples.push_back(hit);
}

bool isGroundReturn(const double sensor_z,
                    const Vec3& dir_w,
                    const double range_m) {
    if (!std::isfinite(sensor_z) || !std::isfinite(dir_w.z) || !std::isfinite(range_m)) {
        return false;
    }
    if (sensor_z <= 0.0 || dir_w.z >= -1.0e-6) {
        return false;
    }

    const double hit_z = sensor_z + range_m * dir_w.z;
    if (std::abs(hit_z) <= kGroundHitHeightToleranceM) {
        return true;
    }

    const double expected_ground_range_m = sensor_z / -dir_w.z;
    const double expected_range_tol_m =
        std::max(kGroundExpectedRangeAbsToleranceM,
                 std::abs(expected_ground_range_m) * kGroundExpectedRangeRelTolerance);
    return std::abs(range_m - expected_ground_range_m) <= expected_range_tol_m;
}

} // namespace

LocalMapObservation MatrixLidarLocalMapObservationSource::collect(const NavPose2d& pose,
                                                                const RobotState& est,
                                                                const TimePointUs now) {
    LocalMapObservation out{};

    if (!est.has_matrix_lidar || !est.matrix_lidar.valid) {
        return out;
    }

    if (!finitePose(pose)) {
        return out;
    }

    const std::size_t cols = est.matrix_lidar.cols;
    const std::size_t rows = est.matrix_lidar.rows;
    if (cols == 0 || rows == 0 || cols * rows > MatrixLidarFrame::kMaxCells) {
        return out;
    }

    const auto model = static_cast<physics_sim::MatrixLidarModel>(est.matrix_lidar.model);
    if (model != physics_sim::MatrixLidarModel::Matrix64x8 &&
        model != physics_sim::MatrixLidarModel::Vl53l7Cx8x8) {
        return out;
    }
    if (!matrix_lidar_geom::expectedGridShape(model, cols, rows)) {
        return out;
    }

    out.timestamp_us = est.matrix_lidar.timestamp_us.value != 0 ? est.matrix_lidar.timestamp_us : now;

    double fov_h = matrix_lidar_geom::kMatrix64x8FovHRad;
    double fov_v = matrix_lidar_geom::kMatrix64x8FovVRad;
    matrix_lidar_geom::fovForModel(model, fov_h, fov_v);

    Vec3 optical_axis{};
    Vec3 optical_right{};
    Vec3 optical_up{};
    fillOpticalBasis(optical_axis, optical_right, optical_up);

    double roll = 0.0;
    double pitch = 0.0;
    const double yaw = pose.yaw_rad;
    if (est.has_body_twist_state) {
        roll = est.body_twist_state.twist_pos_rad.x;
        pitch = est.body_twist_state.twist_pos_rad.y;
    }
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
        return out;
    }

    const Mat3 R_bw = Mat3::rotZ(yaw) * Mat3::rotY(pitch) * Mat3::rotX(roll);
    const Vec3 offset_b{matrix_lidar_geom::kSensorOffsetForwardM,
                        matrix_lidar_geom::kSensorOffsetLeftM,
                        matrix_lidar_geom::kSensorOffsetUpM};
    const Vec3 offset_w = R_bw * offset_b;
    const double sensor_x = pose.x_m + offset_w.x;
    const double sensor_y = pose.y_m + offset_w.y;
    double sensor_z = offset_w.z;
    if (est.has_body_twist_state) {
        sensor_z += est.body_twist_state.body_trans_m.z;
    }

    out.samples.reserve(std::min<std::size_t>(cols * rows, MatrixLidarFrame::kMaxCells));
    out.samples.push_back(LocalMapObservationSample{pose.x_m, pose.y_m, LocalMapCellState::Free});
    out.samples.push_back(LocalMapObservationSample{sensor_x, sensor_y, LocalMapCellState::Free});

    for (std::size_t row = 0; row < rows; ++row) {
        for (std::size_t col = 0; col < cols; ++col) {
            const std::size_t idx = row * cols + col;
            const std::uint16_t mm = est.matrix_lidar.ranges_mm[idx];

            const Vec3 dir_b = cellDirectionBody(optical_axis,
                                                   optical_right,
                                                   optical_up,
                                                   static_cast<int>(row),
                                                   static_cast<int>(rows),
                                                   static_cast<int>(col),
                                                   static_cast<int>(cols),
                                                   fov_h,
                                                   fov_v);
            const Vec3 dir_w = R_bw * dir_b;
            if (!std::isfinite(dir_w.x) || !std::isfinite(dir_w.y)) {
                continue;
            }

            if (mm == physics_sim::kMatrixLidarInvalidMm) {
                const double clear_range_m =
                    static_cast<double>(matrix_lidar_geom::maxRangeMmForModel(model)) * 1.0e-3;
                appendRaySamples(out, sensor_x, sensor_y, sensor_z, dir_w, clear_range_m, false, 0.0,
                                 std::numeric_limits<double>::quiet_NaN(), false);
                continue;
            }

            const std::uint16_t mm_clamped = matrix_lidar_geom::clampRangeMm(model, mm);
            const double range_m = static_cast<double>(mm_clamped) * 1.0e-3;
            if (!std::isfinite(range_m) || range_m <= 0.0) {
                continue;
            }
            if (isGroundReturn(sensor_z, dir_w, range_m)) {
                appendRaySamples(out, sensor_x, sensor_y, sensor_z, dir_w, range_m, false, 0.0,
                                 std::numeric_limits<double>::quiet_NaN(), true);
                continue;
            }
            appendRaySamples(out, sensor_x, sensor_y, sensor_z, dir_w, range_m, true, range_m,
                             sensor_z + range_m * dir_w.z, false);
        }
    }

    return out;
}
